#!/usr/bin/env python3
"""Обновляет include/firmware_manifest.json после успешной OTA-сборки."""

from __future__ import annotations

import json
import sys
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
MANIFEST = ROOT / "include" / "firmware_manifest.json"


def main() -> int:
    if len(sys.argv) != 8:
        print(
            "usage: update-firmware-manifest.py "
            "<device_id> <sketch_dir> <version> <partition> "
            "<ota_file> <bytes> <sha256>",
            file=sys.stderr,
        )
        return 2

    device_id, sketch_dir, version, partition, ota_file, size_s, sha256 = sys.argv[1:]

    if not MANIFEST.exists():
        print(f"manifest not found: {MANIFEST}", file=sys.stderr)
        return 1

    data = json.loads(MANIFEST.read_text(encoding="utf-8"))
    devices = data.setdefault("devices", {})
    entry = devices.setdefault(device_id, {})

    build_date = ota_file.rsplit("-", 1)[-1].removesuffix(".bin")
    entry.update(
        {
            "sketch": sketch_dir,
            "version": version,
            "partition": partition,
            "last_build": {
                "date": build_date,
                "file": ota_file,
                "bytes": int(size_s),
                "sha256": sha256,
            },
        }
    )

    data["updated_at"] = datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")
    MANIFEST.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"[manifest] {device_id} -> {version} ({ota_file})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
