"""The session record's bridge half: the bridge's own model calls (object grounding).

Writes back end/captures/YYYY-MM-DD/recording/grounding.jsonl, beside ai.jsonl
(the web app's model traffic and events, "AI Controlled WebApp/scripts/SessionLog.ts")
and the video (SessionRecorder.py). Images inside a record are stored once as
images/<sha1>.<ext>, the same names the web app's log uses, and replaced by
{"image_file": ..., "bytes": ...}. Each /locate's raw depth is kept as a 16-bit
depth/<sha1>.png (StoreDepth). Recording is best effort: it never raises.
SO101_CAPTURES_ROOT points it elsewhere (tests).
"""

from __future__ import annotations

import base64
import binascii
import hashlib
import json
import os
import re
import sys
import threading
import time
from pathlib import Path
from typing import Any

BackendRoot = Path(__file__).resolve().parent
DataUrl = re.compile(r"^data:image/([a-z0-9.+-]+);base64,", re.IGNORECASE)
_Lock = threading.Lock()


def CapturesRoot() -> Path:
    return Path(os.environ.get("SO101_CAPTURES_ROOT") or BackendRoot / "captures")


def RecordingDirectory(When: float | None = None) -> Path:
    return CapturesRoot() / time.strftime("%Y-%m-%d", time.localtime(When)) / "recording"


def StoreImages(Value: Any, Directory: Path) -> Any:
    """Replace every data-URL image with a reference to a file named by its hash."""
    if isinstance(Value, str):
        Match = DataUrl.match(Value)
        if not Match:
            return Value
        try:
            Bytes = base64.b64decode(Value[Match.end():], validate=False)
        except (binascii.Error, ValueError):
            return Value
        Extension = "jpg" if Match.group(1).lower() == "jpeg" else re.sub(r"[^a-z0-9]", "", Match.group(1).lower())
        Name = f"{hashlib.sha1(Bytes).hexdigest()}.{Extension}"
        Target = Directory / "images" / Name
        if not Target.exists():
            Target.parent.mkdir(parents=True, exist_ok=True)
            # Write then rename, so the web app storing the same image never sees half a file.
            Temporary = Target.with_name(f"{Name}.{os.getpid()}.tmp")
            Temporary.write_bytes(Bytes)
            try:
                os.replace(Temporary, Target)
            except OSError:
                Temporary.unlink(missing_ok=True)
        return {"image_file": f"images/{Name}", "bytes": len(Bytes)}
    if isinstance(Value, list):
        return [StoreImages(Item, Directory) for Item in Value]
    if isinstance(Value, dict):
        return {Key: StoreImages(Item, Directory) for Key, Item in Value.items()}
    return Value


def StoreDepth(DepthMm: Any) -> dict[str, Any] | None:
    """Save raw depth (uint16 mm) as a 16-bit PNG in recording/depth/<sha1>.png.

    The 2026-09-28 review had to rebuild depth from the colourized PNGs sent to the
    model; about 1 MB per /locate makes every later review exact. Best effort, like
    Append: returns {"depth_file", "bytes"} for the record, or None.
    """
    try:
        import cv2
        import numpy as np

        Ok, Encoded = cv2.imencode(".png", np.ascontiguousarray(DepthMm, dtype=np.uint16))
        if not Ok:
            return None
        Bytes = Encoded.tobytes()
        Name = f"{hashlib.sha1(Bytes).hexdigest()}.png"
        Target = RecordingDirectory() / "depth" / Name
        if not Target.exists():
            Target.parent.mkdir(parents=True, exist_ok=True)
            Temporary = Target.with_name(f"{Name}.{os.getpid()}.tmp")
            Temporary.write_bytes(Bytes)
            try:
                os.replace(Temporary, Target)
            except OSError:
                Temporary.unlink(missing_ok=True)
        return {"depth_file": f"depth/{Name}", "depth_bytes": len(Bytes)}
    except Exception as Exc:  # recording must never break the bridge
        print(f"[session-log] could not store depth: {Exc}", file=sys.stderr)
        return None


def Append(Name: str, Record: dict[str, Any]) -> None:
    """Append one record, stamped with the time, to the day's recording/<Name>."""
    Now = time.time()
    try:
        Directory = RecordingDirectory(Now)
        Directory.mkdir(parents=True, exist_ok=True)
        Line = {**StoreImages(Record, Directory), "t": round(Now, 3),
                "time": time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(Now)) + f".{int(Now % 1 * 1000):03d}Z"}
        with _Lock, open(Directory / Name, "a", encoding="utf-8") as File:
            File.write(json.dumps(Line, default=str) + "\n")
    except Exception as Exc:  # recording must never break the bridge
        print(f"[session-log] could not record {Record.get('kind')}: {Exc}", file=sys.stderr)
