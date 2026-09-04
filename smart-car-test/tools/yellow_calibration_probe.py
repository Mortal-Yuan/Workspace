#!/usr/bin/env python3
"""Capture RGB332 preview frames and report colored-object statistics."""

from __future__ import annotations

import argparse
import json
import statistics
import struct
import time
import zlib
from collections import Counter, deque
from pathlib import Path

from usb_camera_monitor import FrameStreamParser, RGB332_TABLE, connect_preview


def is_target(value: int, color: str) -> bool:
    red, green, blue = RGB332_TABLE[value]
    if color == "green":
        # Firmware paints its 3x3 line marker as exact RGB332 green (0x1c).
        # Ignore that synthetic value while retaining natural green shades.
        return (
            value != 0x1C
            and green >= 90
            and green - red >= 20
            and green - blue >= 20
            and green * 100 >= max(red, blue) * 125
        )
    return (
        red >= 105
        and green >= 70
        and blue <= 170
        and red - blue >= 35
        and green - blue >= 20
        and green * 100 >= red * 42
        and red * 100 >= green * 65
    )


def is_line_roi_overlay(x: int, y: int) -> bool:
    # Current 80x60 preview maps the configured 250..750 / 467..797 ROI to
    # x=20..59 and y=28..46. Its border is painted pure yellow by firmware.
    return ((y in (28, 46) and 20 <= x <= 59) or
            (x in (20, 59) and 28 <= y <= 46))


def components(frame, color: str) -> list[dict[str, object]]:
    width, height = frame.width, frame.height
    mask = [False] * (width * height)
    for y in range(height):
        for x in range(width):
            index = y * width + x
            mask[index] = (
                not is_line_roi_overlay(x, y)
                and is_target(frame.pixels[index], color)
            )

    seen = [False] * len(mask)
    results: list[dict[str, object]] = []
    for seed, enabled in enumerate(mask):
        if not enabled or seen[seed]:
            continue
        queue = deque([seed])
        seen[seed] = True
        pixels: list[int] = []
        xs: list[int] = []
        ys: list[int] = []
        while queue:
            index = queue.popleft()
            x, y = index % width, index // width
            pixels.append(frame.pixels[index])
            xs.append(x)
            ys.append(y)
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    nx, ny = x + dx, y + dy
                    neighbor = ny * width + nx
                    if (dx == 0 and dy == 0) or not (0 <= nx < width and 0 <= ny < height):
                        continue
                    if mask[neighbor] and not seen[neighbor]:
                        seen[neighbor] = True
                        queue.append(neighbor)
        box_width = max(xs) - min(xs) + 1
        box_height = max(ys) - min(ys) + 1
        area = len(pixels)
        fill = area / (box_width * box_height)
        aspect = min(box_width, box_height) / max(box_width, box_height)
        if area < 3 or fill < 0.25 or aspect < 0.35:
            continue
        channels = [RGB332_TABLE[value] for value in pixels]
        results.append({
            "area": area,
            "box": [min(xs), min(ys), max(xs), max(ys)],
            "center": [round(statistics.mean(xs), 1), round(statistics.mean(ys), 1)],
            "fill": round(fill, 3),
            "aspect": round(aspect, 3),
            "mean_rgb": [
                round(statistics.mean(channel[i] for channel in channels), 1)
                for i in range(3)
            ],
            "min_rgb": [min(channel[i] for channel in channels) for i in range(3)],
            "max_rgb": [max(channel[i] for channel in channels) for i in range(3)],
            "rgb332": Counter(pixels).most_common(8),
        })
    return sorted(results, key=lambda item: item["area"], reverse=True)


def save_ppm(frame, destination: Path) -> None:
    rgb = b"".join(RGB332_TABLE[value] for value in frame.pixels)
    header = f"P6\n{frame.width} {frame.height}\n255\n".encode("ascii")
    destination.write_bytes(header + rgb)


def save_png(frame, destination: Path, scale: int = 8) -> None:
    def chunk(kind: bytes, data: bytes) -> bytes:
        body = kind + data
        return struct.pack(">I", len(data)) + body + struct.pack(
            ">I", zlib.crc32(body) & 0xFFFFFFFF)

    rows = []
    for y in range(frame.height):
        row = frame.pixels[y * frame.width:(y + 1) * frame.width]
        expanded = b"".join(
            RGB332_TABLE[value] * scale for value in row)
        rows.extend([b"\0" + expanded] * scale)
    data = b"".join(rows)
    destination.write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB",
                                     frame.width * scale,
                                     frame.height * scale,
                                     8, 2, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(data, 9))
        + chunk(b"IEND", b"")
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM3")
    parser.add_argument("--seconds", type=float, default=6.0)
    parser.add_argument("--color", choices=("yellow", "green"),
                        default="yellow")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.output is None:
        args.output = Path(f"build/{args.color}-calibration")
    args.output.mkdir(parents=True, exist_ok=True)

    connection = connect_preview(args.port)
    stream = FrameStreamParser()
    frames = []
    deadline = time.monotonic() + args.seconds
    heartbeat = time.monotonic()
    try:
        while time.monotonic() < deadline:
            parsed, _ = stream.feed(connection.read(8192))
            frames.extend(parsed)
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

    if not frames:
        raise RuntimeError("no valid preview frame received")
    sampled = frames[-min(12, len(frames)):]
    report = {
        "frames_received": len(frames),
        "crc_errors": stream.crc_errors,
        "sequence": [frames[0].sequence, frames[-1].sequence],
        "samples": [],
    }
    for frame in sampled:
        found = components(frame, args.color)
        report["samples"].append({
            "sequence": frame.sequence,
            "components": found[:5],
        })
    latest = frames[-1]
    save_ppm(latest, args.output / f"{args.color}-latest.ppm")
    save_png(latest, args.output / f"{args.color}-latest.png")
    (args.output / f"{args.color}-report.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    print(f"saved={args.output / f'{args.color}-latest.ppm'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
