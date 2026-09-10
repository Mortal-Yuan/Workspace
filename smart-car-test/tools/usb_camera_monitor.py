#!/usr/bin/env python3
"""Smart-car USB camera and telemetry monitor.

The ESP32-S3 keeps the UVC camera on its USB-host port. This program receives
the car's processed 160x120 preview and normal STATUS/EVENT records through the
CP210x UART connection. No Pillow or OpenCV dependency is required.
"""

from __future__ import annotations

import argparse
import queue
import re
import struct
import sys
import threading
import time
import zlib
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import serial
from serial.tools import list_ports


NORMAL_BAUD = 115200
PREVIEW_BAUD = 460800
MAGIC = bytes((0xA5, 0x5A, 0xC3, 0x3C)) + b"SCV1"
HEADER = struct.Struct("<8s8BIIhhhHI")
HEADER_SIZE = HEADER.size
MAX_PAYLOAD = 160 * 120
READY_MARKER = b"USB_PREVIEW_READY baud=460800 version=1"

FLAG_FRAME_VALID = 1 << 0
FLAG_LINE_DETECTED = 1 << 1
FLAG_FINISH_DETECTED = 1 << 2
FLAG_BALL_CANDIDATE = 1 << 3
FLAG_BALL_DETECTED = 1 << 4

OBSTACLE_NAMES = (
    "传感器检查", "巡线", "等待障碍移开", "刹停", "左横移",
    "左移后稳定", "中间前进", "前进后稳定", "右横移",
    "最终前进", "已完成", "安全停车",
)
LINE_NAMES = ("空闲", "直行", "普通转弯", "向左找线", "向右找线", "重捕获")
STARTUP_NAMES = (
    "等待安全放行", "前进20 cm", "前进后停车",
    "右转约60度", "右转后停车", "进入巡线",
)

BALL_APPROACH_NAMES = (
    "空闲", "向左找球", "向右找球", "搜索停车观察", "候选停车确认",
    "对齐判断", "对齐短脉冲", "对齐后停车观察", "接近目标球",
    "入夹停车复核", "已入夹并停止", "安全停车",
)

BALL_APPROACH_REASON_NAMES = (
    "None", "Started", "Search reverse", "Candidate", "Confirmed",
    "Blue goal missing", "Route shift", "Route aligned",
    "Route best effort", "Alignment pulse", "Aligned", "Realign",
    "Target ball lost", "Capture seen", "Capture confirmed", "Push started",
    "Goal seen", "Goal reached", "Camera stale", "Acquire timeout",
    "Approach timeout", "Push timeout", "Total timeout",
)


BALL_APPROACH_NAMES = (
    "Idle", "Search left", "Search right", "Search settle",
    "Confirm red ball", "Search goal left", "Search goal right",
    "Goal search settle", "Route align",
    "Route lateral pulse", "Route settle", "Align red ball",
    "Red alignment pulse", "Red alignment settle", "Approach red ball",
    "Verify ball in clip", "Align blue goal", "Goal alignment pulse",
    "Goal alignment settle", "Fixed 500 ms kick", "Legacy verify (unused)",
    "Done", "Failsafe", "Anomaly wait",
)

BALL_MISSION_NAMES = (
    "Idle", "Red to left goal", "Transition settle",
    "Green to right goal", "Done", "Failsafe",
    "Right turn about 60 deg", "Turn settle",
    "Reverse about 8 cm", "Reverse settle",
)


@dataclass(frozen=True)
class PreviewFrame:
    width: int
    height: int
    flags: int
    threshold: int
    contrast: int
    sequence: int
    timestamp_ms: int
    center: int
    far_center: int
    steering: int
    pixels: bytes


class FrameStreamParser:
    """Separates CRC-protected binary frames from interleaved text telemetry."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.text_buffer = bytearray()
        self.crc_errors = 0

    def _feed_text(self, data: bytes, lines: list[str]) -> None:
        for value in data:
            if value == 10:
                text = self.text_buffer.decode("utf-8", errors="ignore").strip()
                self.text_buffer.clear()
                if text:
                    lines.append(text)
            elif value == 13:
                continue
            elif value == 9 or 32 <= value <= 126:
                if len(self.text_buffer) < 4096:
                    self.text_buffer.append(value)

    def feed(self, data: bytes) -> tuple[list[PreviewFrame], list[str]]:
        self.buffer.extend(data)
        frames: list[PreviewFrame] = []
        lines: list[str] = []
        while self.buffer:
            marker = self.buffer.find(MAGIC)
            if marker < 0:
                safe = len(self.buffer) - (len(MAGIC) - 1)
                if safe > 0:
                    self._feed_text(bytes(self.buffer[:safe]), lines)
                    del self.buffer[:safe]
                break
            if marker > 0:
                self._feed_text(bytes(self.buffer[:marker]), lines)
                del self.buffer[:marker]
            if len(self.buffer) < HEADER_SIZE:
                break
            fields = HEADER.unpack_from(self.buffer)
            (
                magic, version, width, height, pixel_format, flags,
                threshold, contrast, _reserved, sequence, timestamp_ms,
                center, far_center, steering, payload_size, expected_crc,
            ) = fields
            if (
                magic != MAGIC
                or version != 1
                or pixel_format != 1
                or width <= 0
                or height <= 0
                or payload_size != width * height
                or payload_size > MAX_PAYLOAD
            ):
                del self.buffer[0]
                continue
            packet_size = HEADER_SIZE + payload_size
            if len(self.buffer) < packet_size:
                break
            payload = bytes(self.buffer[HEADER_SIZE:packet_size])
            if (zlib.crc32(payload) & 0xFFFFFFFF) != expected_crc:
                self.crc_errors += 1
                del self.buffer[0]
                continue
            frames.append(PreviewFrame(
                width=width,
                height=height,
                flags=flags,
                threshold=threshold,
                contrast=contrast,
                sequence=sequence,
                timestamp_ms=timestamp_ms,
                center=center,
                far_center=far_center,
                steering=steering,
                pixels=payload,
            ))
            del self.buffer[:packet_size]
        return frames, lines


def _open_serial(port: str, baud: int) -> serial.Serial:
    connection = serial.Serial()
    connection.port = port
    connection.baudrate = baud
    connection.timeout = 0.10
    connection.write_timeout = 0.50
    connection.dtr = False
    connection.rts = False
    connection.open()
    return connection


def discover_port(requested: Optional[str]) -> str:
    ports = list(list_ports.comports())
    if requested:
        if not any(item.device.upper() == requested.upper() for item in ports):
            available = ", ".join(item.device for item in ports) or "无"
            raise RuntimeError(f"找不到串口 {requested}；当前串口：{available}")
        return requested
    for item in ports:
        if item.vid == 0x10C4 and item.pid == 0xEA60:
            return item.device
    if len(ports) == 1:
        return ports[0].device
    available = ", ".join(item.device for item in ports) or "无"
    raise RuntimeError(f"未找到 CP210x；当前串口：{available}")


def connect_preview(port: str, preview_command: bytes = b"u") -> serial.Serial:
    """Attach without ever transmitting at an unknown baud rate."""

    # A previous monitor may still own an active high-rate stream. Listen for
    # one fully valid packet before sending anything at PREVIEW_BAUD.
    probe = _open_serial(port, PREVIEW_BAUD)
    parser = FrameStreamParser()
    deadline = time.monotonic() + 1.2
    active = False
    while time.monotonic() < deadline:
        frames, _ = parser.feed(probe.read(4096))
        if frames:
            active = True
            break
    if active:
        probe.reset_input_buffer()
        probe.write(preview_command)
        return probe
    probe.close()

    # If an orphaned stream existed but happened to be between packets, its
    # 3-second firmware heartbeat timeout safely restores NORMAL_BAUD.
    time.sleep(3.2)
    connection = _open_serial(port, NORMAL_BAUD)
    connection.reset_input_buffer()
    response = bytearray()
    # Opening some CP210x adapters can reset the board even with DTR/RTS
    # deasserted.  Keep requesting preview until camera/UART initialization has
    # completed instead of losing the only request during the boot log.
    deadline = time.monotonic() + 10.0
    next_request = 0.0
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_request:
            connection.write(preview_command)
            next_request = now + 0.5
        response.extend(connection.read(512))
        if READY_MARKER in response:
            connection.baudrate = PREVIEW_BAUD
            time.sleep(0.08)
            connection.reset_input_buffer()
            connection.write(preview_command)
            return connection
        if len(response) > 8192:
            del response[:-4096]
    connection.close()

    # The firmware changes baud immediately after its acknowledgement. If a
    # USB-UART driver transition discarded that short text line, accept a
    # complete CRC-valid high-rate frame as proof that the command succeeded.
    probe = _open_serial(port, PREVIEW_BAUD)
    parser = FrameStreamParser()
    deadline = time.monotonic() + 1.2
    while time.monotonic() < deadline:
        frames, _ = parser.feed(probe.read(4096))
        if frames:
            probe.reset_input_buffer()
            probe.write(preview_command)
            return probe
    probe.close()
    raise RuntimeError(
        "小车没有确认 USB 画面模式。请按 RESET，等待摄像头启动后重试。"
    )


class SerialWorker(threading.Thread):
    def __init__(self, port: str, connection: serial.Serial,
                 output: queue.Queue[tuple[str, object]], preview_command: bytes = b"u") -> None:
        super().__init__(name="smart-car-usb-reader", daemon=True)
        self.port = port
        self.preview_command = preview_command
        self.connection = connection
        self.output = output
        self.parser = FrameStreamParser()
        self.stop_event = threading.Event()
        self.write_lock = threading.Lock()
        self.last_heartbeat = time.monotonic()
        self.last_valid_data = time.monotonic()
        self.reconnecting = threading.Event()

    def send(self, command: bytes) -> None:
        if (self.stop_event.is_set() or self.reconnecting.is_set()
                or not self.connection.is_open):
            return
        with self.write_lock:
            self.connection.write(command)

    def _emit(self, item: tuple[str, object]) -> None:
        try:
            self.output.put_nowait(item)
        except queue.Full:
            try:
                self.output.get_nowait()
            except queue.Empty:
                pass
            try:
                self.output.put_nowait(item)
            except queue.Full:
                pass

    def _reconnect(self) -> bool:
        self.reconnecting.set()
        self._emit(("connection", "检测到小车复位，正在自动重新连接……"))
        try:
            with self.write_lock:
                if self.connection.is_open:
                    self.connection.close()
            while not self.stop_event.is_set():
                try:
                    replacement = connect_preview(self.port, self.preview_command)
                    if self.stop_event.is_set():
                        replacement.write(b"z")
                        replacement.close()
                        return False
                    with self.write_lock:
                        self.connection = replacement
                    self.parser = FrameStreamParser()
                    self.last_valid_data = time.monotonic()
                    self.last_heartbeat = self.last_valid_data
                    self._emit(("connection", "USB画面已自动恢复"))
                    return True
                except (RuntimeError, serial.SerialException, OSError) as error:
                    self._emit(("connection", f"等待小车主程序：{error}"))
                    if self.stop_event.wait(1.0):
                        return False
            return False
        finally:
            self.reconnecting.clear()

    def run(self) -> None:
        try:
            while not self.stop_event.is_set():
                try:
                    data = self.connection.read(8192)
                except (serial.SerialException, OSError) as error:
                    self._emit(("error", str(error)))
                    if not self._reconnect():
                        return
                    continue
                if data:
                    frames, lines = self.parser.feed(data)
                    for frame in frames:
                        self._emit(("frame", frame))
                    for line in lines:
                        self._emit(("line", line))
                    if frames or any(
                            line.startswith(("STATUS ", "EVENT ", "USB_PREVIEW_"))
                            for line in lines):
                        self.last_valid_data = time.monotonic()
                    if self.parser.crc_errors:
                        self._emit(("crc", self.parser.crc_errors))
                now = time.monotonic()
                if now - self.last_valid_data >= 2.0:
                    if not self._reconnect():
                        return
                    continue
                if now - self.last_heartbeat >= 1.0:
                    self.send(self.preview_command)
                    self.last_heartbeat = now
        except (serial.SerialException, OSError) as error:
            self._emit(("error", str(error)))

    def close(self) -> None:
        if self.stop_event.is_set():
            return
        self.stop_event.set()
        try:
            with self.write_lock:
                if self.connection.is_open:
                    self.connection.write(b"z")
                    time.sleep(0.15)
                    self.connection.close()
        except (serial.SerialException, OSError):
            pass


RGB332_TABLE = tuple(bytes((
    ((value >> 5) & 0x07) * 255 // 7,
    ((value >> 2) & 0x07) * 255 // 7,
    (value & 0x03) * 255 // 3,
)) for value in range(256))


class MonitorWindow:
    def __init__(self, root, port: str, worker: SerialWorker,
                 messages: queue.Queue[tuple[str, object]],
                 auto_autonomous_test: bool = False,
                 auto_ball_mission_test: bool = False,
                 log_file: Optional[Path] = None) -> None:
        import tkinter as tk
        from tkinter import ttk

        self.tk = tk
        self.root = root
        self.port = port
        self.worker = worker
        self.messages = messages
        self.last_frame_time = 0.0
        self.frame_times: list[float] = []
        self.photo = None
        self.latest_frame: Optional[PreviewFrame] = None
        self.latest_status: dict[str, str] = {}
        self.log_file = log_file
        self.mission_safety_armed = False
        self.mission_stop_sent = False
        if self.log_file is not None:
            self.log_file.parent.mkdir(parents=True, exist_ok=True)
            self.log_file.write_text("", encoding="utf-8")

        root.title("小车 USB 摄像头与状态监控")
        root.configure(bg="#15191f")
        root.protocol("WM_DELETE_WINDOW", self.close)

        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame", background="#15191f")
        style.configure("TLabel", background="#15191f", foreground="#e6edf3")
        style.configure("Title.TLabel", font=("Microsoft YaHei UI", 15, "bold"))
        style.configure("State.TLabel", font=("Consolas", 11), justify="left")

        outer = ttk.Frame(root, padding=12)
        outer.grid(sticky="nsew")
        root.rowconfigure(0, weight=1)
        root.columnconfigure(0, weight=1)

        left = ttk.Frame(outer)
        left.grid(row=0, column=0, sticky="n")
        right = ttk.Frame(outer, padding=(16, 0, 0, 0))
        right.grid(row=0, column=1, sticky="nsew")

        ttk.Label(left, text="摄像头识别画面", style="Title.TLabel").pack(anchor="w")
        self.image_label = tk.Label(
            left, width=640, height=480, bg="#050607",
            highlightthickness=1, highlightbackground="#475569")
        self.image_label.pack(pady=(8, 6))
        legend = tk.Label(
            left,
            text=("红：黑线阈值   黄：巡线ROI   绿/蓝：线路中心   "
                  "橙：球候选   洋红：已确认目标球"),
            bg="#15191f", fg="#cbd5e1", font=("Microsoft YaHei UI", 10))
        legend.pack(anchor="w")

        ttk.Label(right, text="小车实时状态", style="Title.TLabel").pack(anchor="w")
        self.connection_var = tk.StringVar(value=f"USB：{port} @ {PREVIEW_BAUD}")
        self.frame_var = tk.StringVar(value="等待第一帧……")
        self.state_var = tk.StringVar(value="等待 STATUS……")
        ttk.Label(right, textvariable=self.connection_var).pack(anchor="w", pady=(8, 0))
        ttk.Label(right, textvariable=self.frame_var).pack(anchor="w")
        ttk.Separator(right).pack(fill="x", pady=8)
        ttk.Label(right, textvariable=self.state_var, style="State.TLabel",
                  width=48).pack(anchor="w")

        ball_row = ttk.Frame(right)
        ball_row.pack(fill="x", pady=(12, 0))
        self.ball_button = tk.Button(
            ball_row, text="开始找球并推入蓝区  b",
            command=lambda: worker.send(b"b"),
            bg="#047857", fg="white", activebackground="#059669",
            font=("Microsoft YaHei UI", 11, "bold"), padx=14, pady=7)
        self.ball_button.pack(side="left")

        self.mission_button = tk.Button(
            ball_row, text="红球→左蓝区，绿球→右蓝区  n",
            command=lambda: worker.send(b"n"),
            bg="#b45309", fg="white", activebackground="#d97706",
            font=("Microsoft YaHei UI", 10, "bold"), padx=10, pady=7)
        self.mission_button.pack(side="left", padx=8)

        self.autonomous_button = tk.Button(
            ball_row, text="巡线避障 + 3秒 + 双球  f",
            command=self.start_autonomous_test,
            bg="#1d4ed8", fg="white", activebackground="#2563eb",
            font=("Microsoft YaHei UI", 11, "bold"), padx=14, pady=7)
        self.autonomous_button.pack(side="left")

        button_row = ttk.Frame(right)
        button_row.pack(fill="x", pady=(8, 8))
        self.stop_button = tk.Button(
            button_row, text="紧急停车  x", command=lambda: worker.send(b"x"),
            bg="#b91c1c", fg="white", activebackground="#dc2626",
            font=("Microsoft YaHei UI", 12, "bold"), padx=14, pady=8)
        self.stop_button.pack(side="left")
        tk.Button(
            button_row, text="清零编码器  c", command=lambda: worker.send(b"c"),
            bg="#334155", fg="white", activebackground="#475569",
            font=("Microsoft YaHei UI", 10), padx=10, pady=8).pack(side="left", padx=8)
        tk.Button(
            button_row, text="刷新画面  u", command=lambda: worker.send(b"u"),
            bg="#334155", fg="white", activebackground="#475569",
            font=("Microsoft YaHei UI", 10), padx=10, pady=8).pack(side="left")

        ttk.Label(right, text="事件与原始状态").pack(anchor="w")
        self.log = tk.Text(
            right, width=62, height=13, bg="#0b0f14", fg="#cbd5e1",
            insertbackground="white", wrap="word", font=("Consolas", 9))
        self.log.pack(fill="both", expand=True, pady=(4, 0))
        self.log.configure(state="disabled")

        self.last_status_log = 0.0
        self.root.after(30, self.poll)
        if auto_autonomous_test:
            self.root.after(2000, self.start_autonomous_test)
        if auto_ball_mission_test:
            self.root.after(2000, self.start_ball_mission_test)

    def start_autonomous_test(self) -> None:
        """Clear encoders and start the combined line/avoidance/ball task."""
        self.append_log(
            "HOST complete task: line/avoidance, 3 s stop, then two-ball mission")
        self.worker.send(b"x")
        self.root.after(200, lambda: self.worker.send(b"c"))
        self.root.after(700, lambda: self.worker.send(b"f"))

    def start_ball_mission_test(self) -> None:
        """Start one bounded two-ball mission with GUI-visible telemetry."""
        self.append_log("HOST mission test: stop, then send n")
        self.worker.send(b"x")
        self.mission_safety_armed = True
        self.mission_stop_sent = False
        self.root.after(500, lambda: self.worker.send(b"n"))
        self.root.after(100500, self.stop_mission_on_timeout)

    def stop_mission_on_timeout(self) -> None:
        if self.mission_safety_armed and not self.mission_stop_sent:
            self.append_log("HOST SAFETY: 100 s mission timeout; send x")
            self.worker.send(b"x")
            self.mission_stop_sent = True

    def append_log(self, line: str) -> None:
        self.log.configure(state="normal")
        self.log.insert("end", line + "\n")
        line_count = int(self.log.index("end-1c").split(".")[0])
        if line_count > 120:
            self.log.delete("1.0", f"{line_count - 100}.0")
        self.log.see("end")
        self.log.configure(state="disabled")
        if self.log_file is not None:
            with self.log_file.open("a", encoding="utf-8") as stream:
                stream.write(line + "\n")

    @staticmethod
    def _named(value: str, names: tuple[str, ...]) -> str:
        try:
            index = int(value)
            return f"{names[index]} ({index})" if 0 <= index < len(names) else value
        except (ValueError, TypeError):
            return value

    def update_status(self, line: str) -> None:
        values = dict(re.findall(r"(\w+)=([^\s]+)", line))
        self.latest_status = values
        obstacle = self._named(values.get("obstacle", "?"), OBSTACLE_NAMES)
        line_state = self._named(values.get("line", "?"), LINE_NAMES)
        startup = self._named(values.get("startup", "?"), STARTUP_NAMES)
        ball_control_fields = values.get("ballctl", "?").split("/")
        ball_control = self._named(
            ball_control_fields[0] if ball_control_fields else "?",
            BALL_APPROACH_NAMES)
        ball_reason = self._named(
            ball_control_fields[5] if len(ball_control_fields) > 5 else "?",
            BALL_APPROACH_REASON_NAMES)
        mission = self._named(values.get("mission", "?"), BALL_MISSION_NAMES)
        if (self.mission_safety_armed and not self.mission_stop_sent and
                values.get("mode") == "MISSION"):
            if values.get("fresh") == "0" or values.get("mission") == "5":
                self.append_log("HOST SAFETY: vision/failsafe; send x")
                self.worker.send(b"x")
                self.mission_stop_sent = True
            elif values.get("mission") == "4":
                self.append_log("HOST mission complete; send x")
                self.worker.send(b"x")
                self.mission_stop_sent = True
        ball_fields = values.get("BALL", "?").split("/")
        if len(ball_fields) == 4:
            ball_state = ("已确认" if ball_fields[2] == "1" else
                          "候选" if ball_fields[1] == "1" else "未识别")
            ball_text = f"{ball_state} ({values.get('BALL', '?')})"
        else:
            ball_text = values.get("BALL", "?")
        ball_text += (
            f"  GREEN={values.get('GREEN', '?')}"
            f" xy={values.get('gxy', '?')}"
            f" pixels={values.get('gpix', '?')}"
            f" rgb={values.get('grgb', '?')}"
            f" box={values.get('gbbox', '?')}\n"
            f"  LEFT_TARGET={values.get('LEFT_TARGET', '?')}"
            f" xy={values.get('lxy', '?')}"
            f" pixels={values.get('lpix', '?')}"
            f" box={values.get('lbbox', '?')}\n"
            f"           RIGHT_TARGET={values.get('RIGHT_TARGET', '?')}"
            f" xy={values.get('rxy', '?')}"
            f" pixels={values.get('rpix', '?')}"
            f" box={values.get('rbbox', '?')}"
        )
        self.state_var.set(
            f"模式       {values.get('mode', '?')}\n"
            f"避障状态   {obstacle}\n"
            f"启动动作   {startup}\n"
            f"双球任务   {mission}\n"
            f"推球控制   {ball_control}  color/err/cap/goal="
            f"{ball_control_fields[1] if len(ball_control_fields) > 1 else '?'}/"
            f"{ball_control_fields[2] if len(ball_control_fields) > 2 else '?'}/"
            f"{ball_control_fields[3] if len(ball_control_fields) > 3 else '?'}/"
            f"{ball_control_fields[4] if len(ball_control_fields) > 4 else '?'}"
            f"  reason={ball_reason}\n"
            f"巡线状态   {line_state}   CAM={values.get('CAM', '?')}\n"
            f"线路几何   pos={values.get('pos', '?')}  far={values.get('far', '?')}\n"
            f"           head={values.get('head', '?')}  steer={values.get('steer', '?')}\n"
            f"黑线参数   black={values.get('black', '?')}  thr={values.get('thr', '?')}  "
            f"contrast={values.get('contrast', '?')}\n"
            f"摄像头     fresh={values.get('fresh', '?')}  seq={values.get('seq', '?')}  "
            f"drop/err={values.get('cam_drop', '?')}/{values.get('cam_err', '?')}\n"
            f"目标球     {ball_text}  far={values.get('bfar', '?')}  "
            f"xy={values.get('bxy', '?')}\n"
            f"           seed/highlight={values.get('bseed', '?')}  box(新版‰)={values.get('bbox', '?')}\n"
            f"超声波     {values.get('us', '?')} mm  q={values.get('q', '?')}\n"
            f"电机 A/B/C {values.get('cmd', '?')}\n"
            f"编码器     {values.get('encoder', '?')}\n"
            f"控制诊断   overrun={values.get('overrun', '?')}  drops={values.get('drops', '?')}"
        )

    def update_frame(self, frame: PreviewFrame) -> None:
        self.latest_frame = frame
        rgb = b"".join(RGB332_TABLE[value] for value in frame.pixels)
        ppm = f"P6\n{frame.width} {frame.height}\n255\n".encode("ascii") + rgb
        image = self.tk.PhotoImage(data=ppm, format="PPM")
        scale = max(1, min(8, 640 // frame.width, 480 // frame.height))
        self.photo = image.zoom(scale, scale)
        self.image_label.configure(image=self.photo, width=frame.width * scale,
                                   height=frame.height * scale)

        now = time.monotonic()
        self.frame_times.append(now)
        cutoff = now - 2.0
        self.frame_times = [item for item in self.frame_times if item >= cutoff]
        fps = 0.0
        if len(self.frame_times) > 1:
            fps = (len(self.frame_times) - 1) / (
                self.frame_times[-1] - self.frame_times[0])
        line_text = "检测到黑线" if frame.flags & FLAG_LINE_DETECTED else "未检测到黑线"
        ball_text = ("已确认目标球" if frame.flags & FLAG_BALL_DETECTED else
                     "目标球候选" if frame.flags & FLAG_BALL_CANDIDATE else
                     "未识别目标球")
        self.frame_var.set(
            f"画面：{frame.width}×{frame.height}  {fps:.1f} FPS  "
            f"帧 #{frame.sequence}  {line_text}  {ball_text}  CRC正常"
        )

    def poll(self) -> None:
        try:
            while True:
                kind, value = self.messages.get_nowait()
                if kind == "frame":
                    self.update_frame(value)  # type: ignore[arg-type]
                elif kind == "line":
                    line = str(value)
                    if line.startswith("STATUS "):
                        self.update_status(line)
                        now = time.monotonic()
                        if now - self.last_status_log >= 0.5:
                            self.append_log(line)
                            self.last_status_log = now
                    else:
                        self.append_log(line)
                elif kind == "crc":
                    self.connection_var.set(
                        f"USB：{self.port} @ {PREVIEW_BAUD}  CRC丢帧={value}")
                elif kind == "connection":
                    self.connection_var.set(str(value))
                    self.append_log(str(value))
                elif kind == "error":
                    self.connection_var.set(f"USB连接错误：{value}")
                    self.append_log(f"ERROR {value}")
        except queue.Empty:
            pass
        if not self.worker.stop_event.is_set():
            self.root.after(30, self.poll)

    def close(self) -> None:
        self.worker.close()
        self.root.destroy()


def run_self_test() -> None:
    payload = bytes(index & 0xFF for index in range(MAX_PAYLOAD))
    crc = zlib.crc32(payload) & 0xFFFFFFFF
    header = HEADER.pack(
        MAGIC, 1, 160, 120, 1,
        FLAG_FRAME_VALID | FLAG_LINE_DETECTED | FLAG_BALL_CANDIDATE |
        FLAG_BALL_DETECTED,
        99, 123, 0, 42, 31415, -100, 200, -50, len(payload), crc,
    )
    parser = FrameStreamParser()
    frames: list[PreviewFrame] = []
    lines: list[str] = []
    stream = b"STATUS mode=IDLE cmd=0,0,0\n" + header + payload + b"EVENT ok\n"
    for offset in range(0, len(stream), 137):
        parsed_frames, parsed_lines = parser.feed(stream[offset:offset + 137])
        frames.extend(parsed_frames)
        lines.extend(parsed_lines)
    assert len(frames) == 1
    assert frames[0].sequence == 42 and frames[0].pixels == payload
    assert frames[0].flags & FLAG_BALL_DETECTED
    assert any(line.startswith("STATUS") for line in lines)
    assert parser.crc_errors == 0
    assert (frames[0].width, frames[0].height) == (160, 120)
    # Backward compatibility with existing 80x60 firmware and CRC recovery.
    small_payload = payload[:80 * 60]
    small_header = HEADER.pack(
        MAGIC, 1, 80, 60, 1, 0, 99, 123, 0, 43, 31416,
        0, 0, 0, len(small_payload), zlib.crc32(small_payload) & 0xFFFFFFFF)
    bad_payload = bytes([payload[0] ^ 1]) + payload[1:]
    recovered, _ = parser.feed(header + bad_payload + small_header + small_payload)
    assert parser.crc_errors == 1
    assert len(recovered) == 1 and recovered[0].width == 80
    print("usb camera monitor parser: PASS")


def run_live_probe(connection: serial.Serial, seconds: float) -> bool:
    parser = FrameStreamParser()
    frame_count = 0
    status_count = 0
    first_sequence: Optional[int] = None
    last_sequence: Optional[int] = None
    deadline = time.monotonic() + seconds
    heartbeat = time.monotonic()
    try:
        while time.monotonic() < deadline:
            frames, lines = parser.feed(connection.read(8192))
            for frame in frames:
                frame_count += 1
                if first_sequence is None:
                    first_sequence = frame.sequence
                last_sequence = frame.sequence
            status_count += sum(line.startswith("STATUS ") for line in lines)
            now = time.monotonic()
            if now - heartbeat >= 1.0:
                connection.write(b"u")
                heartbeat = now
    finally:
        try:
            connection.write(b"z")
            time.sleep(0.15)
        finally:
            connection.close()
    print(
        f"live preview probe: frames={frame_count} status={status_count} "
        f"seq={first_sequence}->{last_sequence} crc_errors={parser.crc_errors}"
    )
    return frame_count >= max(2, int(seconds)) and parser.crc_errors == 0


def main() -> int:
    argument_parser = argparse.ArgumentParser(
        description="小车 USB 摄像头与状态监控窗口")
    argument_parser.add_argument("--port", help="串口，例如 COM3；默认自动识别 CP210x")
    argument_parser.add_argument("--self-test", action="store_true",
                                 help="仅运行协议解析自测")
    argument_parser.add_argument("--probe-seconds", type=float,
                                 help="连接实车并命令行验证指定秒数，不打开窗口")
    argument_parser.add_argument(
        "--autonomous-test", action="store_true",
        help="Open the window and automatically start the complete line test")
    argument_parser.add_argument(
        "--ball-mission-test", action="store_true",
        help="Open the window and automatically start one bounded two-ball mission")
    argument_parser.add_argument(
        "--log-file", type=Path,
        help="Write GUI event/status lines to this UTF-8 file")
    args = argument_parser.parse_args()
    if args.self_test:
        run_self_test()
        return 0

    try:
        port = discover_port(args.port)
        connection = connect_preview(port)
    except (RuntimeError, serial.SerialException, OSError) as error:
        print(f"无法启动监控：{error}", file=sys.stderr)
        return 1

    if args.probe_seconds is not None:
        return 0 if run_live_probe(connection, args.probe_seconds) else 2

    import tkinter as tk

    messages: queue.Queue[tuple[str, object]] = queue.Queue(maxsize=128)
    worker = SerialWorker(port, connection, messages)
    root = tk.Tk()
    MonitorWindow(root, port, worker, messages,
                  auto_autonomous_test=args.autonomous_test,
                  auto_ball_mission_test=args.ball_mission_test,
                  log_file=args.log_file)
    worker.start()
    try:
        root.mainloop()
    finally:
        worker.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
