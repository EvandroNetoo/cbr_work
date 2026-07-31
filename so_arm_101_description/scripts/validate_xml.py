#!/usr/bin/env python3
"""Validate XML/xacro/URDF files for well-formedness."""
import sys
import xml.etree.ElementTree as ET


def validate_file(filepath: str) -> bool:
    try:
        ET.parse(filepath)
        return True
    except ET.ParseError as e:
        print(f"FAIL: {filepath}: {e}")
        return False


def main() -> int:
    files = sys.argv[1:]
    if not files:
        return 0
    failed = [f for f in files if not validate_file(f)]
    if failed:
        print(f"\n{len(failed)} file(s) failed XML validation.")
        return 1
    print(f"OK: {len(files)} file(s) validated.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
