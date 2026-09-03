#!/usr/bin/env python3
"""Run esptool with conservative ROM write blocks for an unstable USB-UART."""

from __future__ import annotations

import argparse
import time

import esptool
from esptool.loader import ESPLoader


def main() -> None:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--rom-block-size", type=int, default=256)
    parser.add_argument("--block-delay-ms", type=float, default=50.0)
    options, esptool_arguments = parser.parse_known_args()
    if options.rom_block_size <= 0 or options.rom_block_size % 4 != 0:
        raise SystemExit("--rom-block-size must be a positive multiple of four")
    if options.block_delay_ms < 0:
        raise SystemExit("--block-delay-ms cannot be negative")

    ESPLoader.FLASH_WRITE_SIZE = options.rom_block_size
    original_flash_block = ESPLoader.flash_block

    def delayed_flash_block(self, *args, **kwargs):
        result = original_flash_block(self, *args, **kwargs)
        time.sleep(options.block_delay_ms / 1000.0)
        return result

    ESPLoader.flash_block = delayed_flash_block
    esptool.main(esptool_arguments)


if __name__ == "__main__":
    main()
