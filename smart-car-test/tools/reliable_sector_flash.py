#!/usr/bin/env python3
"""Write and verify one image through independent short ROM flash sessions."""

from __future__ import annotations

import argparse
import gc
import hashlib
import subprocess
import time
from pathlib import Path

from esptool.cmds import detect_chip
from esptool.util import FatalError
from serial import SerialException


def restart_device(instance_id: str) -> None:
    subprocess.run(
        ["pnputil", "/restart-device", instance_id],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
        check=False,
    )
    time.sleep(3.0)


def connect(port: str, baud: int):
    esp = detect_chip(port, baud, connect_mode="default_reset")
    if esp.CHIP_NAME != "ESP32-S3":
        esp._port.close()
        raise RuntimeError(f"expected ESP32-S3, found {esp.CHIP_NAME}")
    esp.flash_spi_attach(0)
    return esp


def write_sector(esp, address: int, data: bytes,
                 block_size: int, delay_seconds: float) -> str:
    esp.FLASH_WRITE_SIZE = block_size
    blocks = esp.flash_begin(len(data), address, logging=False)
    for sequence in range(blocks):
        start = sequence * block_size
        block = data[start:start + block_size]
        if len(block) < block_size:
            block += b"\xff" * (block_size - len(block))
        esp.flash_block(block, sequence)
        time.sleep(delay_seconds)
    expected = hashlib.md5(data).hexdigest()
    actual = esp.flash_md5sum(address, len(data))
    if actual != expected:
        raise RuntimeError(
            f"MD5 mismatch at 0x{address:x}: expected {expected}, got {actual}")
    esp.flash_finish(reboot=False)
    return actual


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Reliably flash an ESP32-S3 image one sector per session")
    parser.add_argument("--port", default="COM3")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--address", type=lambda value: int(value, 0), required=True)
    parser.add_argument("--sector-size", type=int, default=4096)
    parser.add_argument("--block-size", type=int, default=128)
    parser.add_argument("--block-delay-ms", type=float, default=100.0)
    parser.add_argument("--attempts", type=int, default=4)
    parser.add_argument("--start-sector", type=int, default=0)
    parser.add_argument("--sector-count", type=int)
    parser.add_argument("--restart-device")
    parser.add_argument("image", type=Path)
    args = parser.parse_args()
    if args.address % args.sector_size != 0:
        raise SystemExit("--address must be sector aligned")
    if args.sector_size <= 0 or args.block_size <= 0:
        raise SystemExit("sector and block sizes must be positive")
    if args.sector_size % args.block_size != 0 or args.block_size % 4 != 0:
        raise SystemExit("sector size must be divisible by a four-byte block size")
    if args.attempts <= 0 or args.block_delay_ms < 0:
        raise SystemExit("attempts must be positive and delay cannot be negative")
    if args.start_sector < 0:
        raise SystemExit("start sector cannot be negative")
    if args.sector_count is not None and args.sector_count <= 0:
        raise SystemExit("sector count must be positive")

    image = args.image.read_bytes()
    original_size = len(image)
    padded_size = (
        (original_size + args.sector_size - 1) // args.sector_size
    ) * args.sector_size
    image += b"\xff" * (padded_size - original_size)
    total_sector_count = padded_size // args.sector_size
    end_sector = total_sector_count
    if args.sector_count is not None:
        end_sector = min(
            args.start_sector + args.sector_count, total_sector_count)
    if args.start_sector >= total_sector_count:
        raise SystemExit(
            f"start sector {args.start_sector} is outside the image "
            f"({total_sector_count} sectors)")
    print(
        f"image={args.image} bytes={original_size} "
        f"sectors={args.start_sector + 1}-{end_sector}/{total_sector_count} "
        f"address=0x{args.address:x}",
        flush=True,
    )

    for sector in range(args.start_sector, end_sector):
        address = args.address + sector * args.sector_size
        data = image[
            sector * args.sector_size:(sector + 1) * args.sector_size
        ]
        for attempt in range(1, args.attempts + 1):
            esp = None
            failure = None
            try:
                esp = connect(args.port, args.baud)
                digest = write_sector(
                    esp, address, data, args.block_size,
                    args.block_delay_ms / 1000.0,
                )
                print(
                    f"sector {sector + 1}/{total_sector_count} "
                    f"0x{address:08x} verified {digest}",
                    flush=True,
                )
            except (
                FatalError,
                SerialException,
                OSError,
                RuntimeError,
                StopIteration,
            ) as error:
                failure = error
                print(
                    f"sector {sector + 1}/{total_sector_count} attempt "
                    f"{attempt}/{args.attempts} failed: {error}",
                    flush=True,
                )
            finally:
                if esp is not None and esp._port.is_open:
                    esp._port.close()
                esp = None
                gc.collect()

            if failure is None:
                break
            if attempt == args.attempts:
                raise failure
            if args.restart_device:
                restart_device(args.restart_device)
            else:
                time.sleep(2.0)

    print(
        f"sectors {args.start_sector + 1}-{end_sector}/"
        f"{total_sector_count} verified; "
        f"file_sha256={hashlib.sha256(image[:original_size]).hexdigest().upper()}",
        flush=True,
    )


if __name__ == "__main__":
    main()
