#!/usr/bin/env python3

import argparse
import os
import re
import sys
from typing import Optional, Tuple


def sanitize_symbol(name: str) -> str:
    cleaned = re.sub(r"[^0-9a-zA-Z_]", "_", name)
    if not cleaned:
        return "startupImage"
    if cleaned[0].isdigit():
        cleaned = f"img_{cleaned}"
    return cleaned


def symbol_macro_prefix(symbol: str) -> str:
    prefix = re.sub(r"[^0-9a-zA-Z_]", "_", symbol).upper()
    if not prefix:
        return "IMAGE"
    if prefix[0].isdigit():
        prefix = f"IMG_{prefix}"
    return prefix


def convert_image(
    input_path: str, width: Optional[int], height: Optional[int], threshold: int
) -> Tuple[bytes, int, int]:
    try:
        from PIL import Image
    except ImportError as exc:
        raise RuntimeError(
            "Pillow is required. Install with: pip install pillow"
        ) from exc

    img = Image.open(input_path).convert("L")
    source_width, source_height = img.size

    target_width = source_width if width is None else width
    target_height = source_height if height is None else height

    if target_width <= 0 or target_height <= 0:
        raise ValueError("Width and height must be greater than zero")

    if (target_width, target_height) != (source_width, source_height):
        img = img.resize((target_width, target_height), Image.Resampling.LANCZOS)

    pixels = img.load()
    bytes_per_row = (target_width + 7) // 8
    packed = bytearray(bytes_per_row * target_height)

    for y in range(target_height):
        row_offset = y * bytes_per_row
        for x in range(target_width):
            val = pixels[x, y]
            bit_on = val < threshold
            if bit_on:
                packed[row_offset + (x // 8)] |= (0x80 >> (x % 8))

    return bytes(packed), target_width, target_height


def write_header(output_path: str, symbol: str, width: int, height: int, packed: bytes):
    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)

    bytes_per_line = 16
    lines = []
    for i in range(0, len(packed), bytes_per_line):
        chunk = packed[i : i + bytes_per_line]
        text = ", ".join(f"0x{b:02X}" for b in chunk)
        lines.append(f"    {text},")

    macro_prefix = symbol_macro_prefix(symbol)
    header_lines = [
        "#pragma once",
        "#include <Arduino.h>",
        "",
        f"#define {macro_prefix}_WIDTH {width}",
        f"#define {macro_prefix}_HEIGHT {height}",
        "",
        f"static const uint8_t PROGMEM {symbol}Bitmap[] = {{",
        *lines,
        "};",
        "",
        f"#define {macro_prefix}_BITMAP {symbol}Bitmap",
    ]

    if symbol == "startupImage":
        header_lines.extend(
            [
                "",
                "#define STARTUP_IMAGE_WIDTH STARTUPIMAGE_WIDTH",
                "#define STARTUP_IMAGE_HEIGHT STARTUPIMAGE_HEIGHT",
                "#define STARTUP_IMAGE_BITMAP STARTUPIMAGE_BITMAP",
            ]
        )

    header_lines.append("")

    content = "\n".join(header_lines)

    with open(output_path, "w", encoding="utf-8") as f:
        f.write(content)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Convert an image into a 1-bit packed bitmap header for MagTag e-ink drawBitmap()."
    )
    parser.add_argument("--input", required=True, help="Input image path (png/jpg/etc)")
    parser.add_argument("--output", required=True, help="Output header path")
    parser.add_argument("--symbol", default="startupImage", help="Base C symbol name")
    parser.add_argument(
        "--width",
        type=int,
        default=None,
        help="Target width (default: source image width)",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=None,
        help="Target height (default: source image height)",
    )
    parser.add_argument("--threshold", type=int, default=128, help="Black/white threshold (0-255)")

    args = parser.parse_args()

    if not os.path.isfile(args.input):
        print(f"Input file not found: {args.input}", file=sys.stderr)
        return 2

    symbol = sanitize_symbol(args.symbol)

    try:
        packed, out_width, out_height = convert_image(
            args.input, args.width, args.height, args.threshold
        )
        write_header(args.output, symbol, out_width, out_height, packed)
    except Exception as exc:
        print(f"Conversion failed: {exc}", file=sys.stderr)
        return 1

    print(
        f"Wrote {args.output} ({out_width}x{out_height}, {len(packed)} bytes, symbol {symbol}Bitmap)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
