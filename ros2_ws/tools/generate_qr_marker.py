#!/usr/bin/env python3
import argparse
from pathlib import Path
from urllib.parse import urlencode
from urllib.request import urlopen


def parse_args():
    parser = argparse.ArgumentParser(
        description="Generate a high-contrast QR texture for Gazebo."
    )
    parser.add_argument("--data", default="NAV:HOME")
    parser.add_argument("--output", required=True)
    parser.add_argument("--size", type=int, default=400)
    return parser.parse_args()


def main():
    args = parse_args()
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)

    query = urlencode(
        {
            "data": args.data,
            "size": f"{args.size}x{args.size}",
            "ecc": "H",
            "margin": "20",
        }
    )
    url = f"https://api.qrserver.com/v1/create-qr-code/?{query}"
    with urlopen(url, timeout=20) as response:
        content = response.read()
    output.write_bytes(content)

    print(f"QR_DATA={args.data}")
    print(f"QR_OUTPUT={output}")
    print(f"QR_BYTES={len(content)}")


if __name__ == "__main__":
    main()
