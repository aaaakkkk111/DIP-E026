"""Fail if an Intel HEX file contains data outside the board-safe Flash range."""

from pathlib import Path
import argparse


FLASH_START = 0x08000000
FLASH_END_EXCLUSIVE = 0x08010000


def data_range(path: Path) -> tuple[int, int]:
    upper = 0
    first = None
    last = None
    for number, raw in enumerate(path.read_text(encoding="ascii").splitlines(), 1):
        line = raw.strip()
        if not line:
            continue
        if not line.startswith(":") or len(line) < 11:
            raise ValueError(f"invalid Intel HEX record at line {number}")
        size = int(line[1:3], 16)
        offset = int(line[3:7], 16)
        record_type = int(line[7:9], 16)
        if record_type == 4:
            upper = int(line[9:13], 16) << 16
        elif record_type == 0 and size:
            start = upper + offset
            end = start + size - 1
            first = start if first is None else min(first, start)
            last = end if last is None else max(last, end)
    if first is None or last is None:
        raise ValueError("HEX contains no data records")
    return first, last


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("hex_file", type=Path)
    args = parser.parse_args()
    first, last = data_range(args.hex_file)
    print(f"data range: 0x{first:08X}..0x{last:08X}")
    if first < FLASH_START or last >= FLASH_END_EXCLUSIVE:
        print("FAIL: data falls outside 0x08000000..0x0800FFFF")
        return 2
    print(f"PASS: {FLASH_END_EXCLUSIVE - last - 1} bytes remain below 0x08010000")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
