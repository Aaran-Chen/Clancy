"""Object localization: qwen names the pixels, calibration does the geometry.

`locate_objects` sends the workspace photo to the local Ollama vision model and
asks ONLY for tight bounding boxes (semantics). The VLM never estimates
coordinates; the calibrated camera geometry does:

* With the depth camera (a capture that names its depth frame, and a
  depth-camera calibration), the pixels inside each box become 3D points and
  the object's centre, top, height and footprint are measured directly
  (DepthGeometry.MeasureBox).
* Without depth, the box's bottom-centre pixel (where the object meets the
  table) is mapped onto the table plane (`calibration.pixel_to_plane`). With
  depth, a box that cannot be measured is withheld, never guessed.

Before any model call the capture's own depth is checked against the camera
calibration (CameraDrift); a camera that moved, or cannot be checked, blocks
the whole request. A desk that moved against the arm (the cart was rolled) while
the gripper tape shows the camera has not, is measured again on the capture and
saved as the new desk baseline. The robot's own parts, and measurements that
cannot be real, are withheld with a reason.

With a task object ({"phrase", "label"} from the web app), one strict query
asks for that object alone; the measured objects its box lands on are flagged
task_match, and a box that lands on none is measured as its own entry
(task_query_only).

That is the "vlm" path. By default (LOCATE_MODES "single_call", OllamaOptions.json
locate_mode) a capture with depth takes ONE model call instead: every object as a
box, the task's object named in the same reply with its outline; depth measures
each box, withholds a box with nothing standing in it, and checks and grows the
task's box (_DepthRefine). The strict query runs only when that call misses the
task's object.
"""

from __future__ import annotations

import base64
import json
import math
import re
import sys
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any

import os
import time

import cv2
import numpy as np

import calibration
import CameraDrift
import DeclaredSurfaces
import DepthGeometry
import SessionLog
import SceneGeometry
try:
    import SceneSafety
except ModuleNotFoundError as Exc:
    if Exc.name != "SceneSafety":
        raise
    SceneSafety = None  # Older bridges can show measurements, never register new scene motions.

# Same override the web app's Vite proxy honours.
OLLAMA_BASE = os.environ.get("OLLAMA_BASE_URL", "http://127.0.0.1:11434").rstrip("/")
LOCATE_TIMEOUT_S = 180


def _LoadOllamaOptions() -> dict[str, Any]:
    """num_ctx, num_batch and keep_alive from OllamaOptions.json, which the web app reads too.

    Every caller must send the same num_ctx and num_batch through the native /api/chat, or
    Ollama reloads the model (8-15 s) each time the other one calls. /v1/chat/completions
    ignored num_ctx and loaded the desktop app's 262144 default, leaving under 1 GiB of VRAM.
    """
    Saved: dict[str, Any] = {}
    try:
        Saved = json.loads((Path(__file__).resolve().parent / "OllamaOptions.json").read_text(encoding="utf-8"))
    except (OSError, ValueError) as Exc:
        print(f"[object_locator] OllamaOptions.json unreadable ({Exc}); using its defaults", file=sys.stderr)
    return {"num_ctx": int(Saved.get("num_ctx", 32768)), "num_batch": int(Saved.get("num_batch", 1024)),
            "keep_alive": str(Saved.get("keep_alive", "30m")), "locate_mode": str(Saved.get("locate_mode", "single_call"))}


OLLAMA_OPTIONS = _LoadOllamaOptions()

# How /locate finds objects (OllamaOptions.json "locate_mode"):
# * "single_call" (default): ONE vision-model call names every object with a box (the task's object first, with
#   its outline), and depth does the rest: inside each box the pixels above the measured support nearest the
#   box centre are the object (MeasureBox), so a box with nothing standing in it is withheld (what the voted
#   samples guarded against) and a loose or shifted box is trimmed or grown by depth (what zoom-refine did).
#   Only when that call gives the task's object no box, or its box holds no object in depth, one strict task
#   query follows. The 2026-10-03 and 10-14 recordings took 3-13 model calls (2.2-53 s) per /locate.
# * "vlm": the earlier path: up to LOCATE_SAMPLES voted full-frame samples, the strict task query, zoom-refines.
# A capture without depth, a request for named labels, or a carry refresh always takes "vlm".
# SO101_LOCATE_MODE (environment) overrides the file; locate_objects(locate_mode=...) overrides both.
LOCATE_MODES = ("single_call", "vlm")


def _LocateMode(Requested: str | None = None) -> str:
    for Value in (Requested, os.environ.get("SO101_LOCATE_MODE"), OLLAMA_OPTIONS.get("locate_mode")):
        if Value in LOCATE_MODES:
            return Value
    return "single_call"
# Bounds a runaway reply (the /v1 max_tokens it replaces); detections are a few hundred tokens.
GROUNDING_NUM_PREDICT = 4096

# Multi-sample voting: the VLM occasionally draws a wildly wrong box (e.g. one
# that swallows the gripper), which silently poisons the measured position. We
# query it up to LOCATE_SAMPLES times and take the per-label MEDIAN contact
# pixel, so a single bad box is outvoted. If the first two samples boxed the
# same spots, whatever they called them, the third is skipped to save latency;
# an object only a third sample would have found is then missed.
LOCATE_SAMPLES = 3
SAMPLE_AGREE_PX = 25.0     # first two samples within this -> skip third sample
SAMPLE_UNSTABLE_PX = 45.0  # spread beyond this -> object flagged "unstable"
MAX_BOX_AREA_FRACTION = 0.85  # boxes covering most of the image are garbage

# The task query: its box picks a measured object when its centre lies inside that
# object's box or they overlap this much. Nouns that name no kind of object.
TASK_MATCH_IOU = 0.3
TASK_PHRASE_MAX_CHARS = 80
# Task matches whose centres lie this close are one object under several labels (the web app's SameSpotCm,
# TaskObject.ts): at 16:35:13 UTC of 2026-10-14 "White holder" and "white device" were 0.1 cm apart.
TASK_TWIN_CM = 1.5
GENERIC_TASK_WORDS = frozenset({"thing", "object", "item", "one", "stuff", "piece"})
# The web app's colour words (TaskObject.ts), and the words a label's own noun ends at ("black object on tape").
TASK_COLOUR_WORDS = frozenset({"black", "white", "red", "blue", "green", "yellow", "orange", "purple", "pink", "grey", "gray", "brown",
                               "silver", "gold", "golden", "dark", "light", "beige", "clear", "transparent"})
LABEL_PREPOSITIONS = frozenset({"in", "into", "on", "onto", "of", "with", "near", "next", "under", "over", "at", "by", "beside",
                                "behind", "inside", "from", "above", "below"})

# The robot's own parts, matched as whole words: the prompt asks the model to ignore
# the arm, yet it boxed "robot arm" or "white robotic arm" in 6 of 8 locates on
# 2026-09-28. Whole words, so "binder", "alarm", "armchair" and "cartridge" pass.
ROBOT_PART_WORDS = frozenset({"robot", "robotic", "arm", "gripper", "claw", "servo", "cart", "camera"})
# Robot words that also name loose parts on the desk: a task that asks for "the black servo" (2026-09-30 05:24)
# means one lying there, so its box is measured; the arm's own servos are masked out of the depth (ArmExclusion).
LOOSE_PART_WORDS = frozenset({"servo"})
# The robot's own basket ("black bin" at 19:02:01) is never measured as an object.
BIN_WORDS = frozenset({"bin", "basket"})
# Measurements outside these limits are garbage (far background, the room floor), not
# objects: "white box" 553 cm away, "paper" at X 195. Pose errors stay inside them.
MAX_DISTANCE_CM = 110.0
MAX_ABS_X_CM = 60.0
Y_RANGE_CM = (-5.0, 110.0)
HEIGHT_RANGE_CM = (0.3, 25.0)
MAX_FOOTPRINT_CM = 35.0
# Scene tripwire: desk-area objects whose measured surroundings disagree with the
# calibrated desk (at 19:00:57 seven desk objects sat a shared 4.6 cm low). One object
# alone is usually on something thin (a notebook, a cutting board), which recalibrating
# cannot clear; it trips only while the desk check itself reads half its limits or more.
TRIPWIRE_SINGLE_CM = 1.0
TRIPWIRE_SINGLE_DESK_FRACTION = 0.5
TRIPWIRE_SHARED_CM = 1.5
TRIPWIRE_SHARED_COUNT = 3
# With no calibrated desk in view and a camera the gripper tip could not check ("assumed": still measured and planned,
# Aaran's decision 4, audit IH1) the desk readings above cannot trip: there is no desk under the objects (on the
# recorded 2026-10-06 cabinet frame with the model turned 4.16 deg the bottle measured 4.2 cm off and nothing tripped,
# 2026-10-08 review). Such a capture's tripwire is the basket rim instead (RobotReference.RimCheck, which every
# no-desk capture computes): it trips when the rim positively reads moved against the calibration's own frames. On
# the recorded frames: 15:01 (the camera where it was calibrated) ok; the 15:01 frame as a camera turned, drooped or
# rolled on the robot 1-6 deg: moved in 14 of 24 cases, ok in 7, not judgeable in 3; 23:01 (the real 4.16 deg turn,
# a hand on the lens) not judgeable (.claude/Stage5/logs/RimTripwire.txt). False: the desk readings only, as before.
TRIPWIRE_ASSUMED_RIM = True


def _http_json(url: str, payload: dict[str, Any] | None = None, timeout: float = 10) -> Any:
    data = json.dumps(payload).encode("utf-8") if payload is not None else None
    request = urllib.request.Request(
        url,
        data=data,
        headers={"Content-Type": "application/json"} if data else {},
        method="POST" if data else "GET",
    )
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


_VISION_CACHE: dict[str, bool] = {}


def _has_vision(name: str) -> bool:
    """Whether Ollama reports image input for a model (/api/show capabilities)."""
    if name not in _VISION_CACHE:
        try:
            capabilities = _http_json(f"{OLLAMA_BASE}/api/show", {"model": name}).get("capabilities") or []
            _VISION_CACHE[name] = any(str(item).lower() == "vision" for item in capabilities)
        except (urllib.error.URLError, OSError, json.JSONDecodeError, TimeoutError, AttributeError):
            return False
    return _VISION_CACHE[name]


def _pick_model(requested: str | None) -> str | None:
    """Prefer the caller's model, then a loaded vision model, then the largest installed one.

    Only vision models can ground boxes. Ollama lists installed models by recency,
    so the first one can be a text or embedding model that cannot see the image.
    """
    if requested:
        return requested
    for path in ("/api/ps", "/api/tags"):
        try:
            models = _http_json(f"{OLLAMA_BASE}{path}").get("models") or []
        except (urllib.error.URLError, OSError, json.JSONDecodeError, TimeoutError):
            continue
        for entry in sorted(models, key=lambda item: -int(item.get("size") or 0)):
            name = entry.get("model") or entry.get("name")
            if name and _has_vision(name):
                return name
    return None


def _decode_image(image_data_url: str) -> np.ndarray | None:
    """Decode the sent image (BGR), for size checks and zoom-refine crops."""
    try:
        _, b64 = image_data_url.split(",", 1)
        buffer = np.frombuffer(base64.b64decode(b64), dtype=np.uint8)
        return cv2.imdecode(buffer, cv2.IMREAD_COLOR)
    except (ValueError, cv2.error):
        return None


def _extract_json_array(text: str) -> list[Any] | None:
    """Pull the first complete JSON array out of model text (fences etc. tolerated)."""
    depth = 0
    start = None
    in_string = False
    escaped = False
    for index, char in enumerate(text):
        if in_string:
            if escaped:
                escaped = False
            elif char == "\\":
                escaped = True
            elif char == '"':
                in_string = False
            continue
        if char == '"':
            in_string = True
        elif char == "[":
            if depth == 0:
                start = index
            depth += 1
        elif char == "]" and depth > 0:
            depth -= 1
            if depth == 0 and start is not None:
                try:
                    parsed = json.loads(text[start : index + 1])
                    if isinstance(parsed, list):
                        return parsed
                except json.JSONDecodeError:
                    start = None
    return None


def _extract_json_object(text: str) -> dict[str, Any] | None:
    """The first complete JSON object in model text (fences etc. tolerated), as _extract_json_array reads arrays."""
    depth, start, in_string, escaped = 0, None, False, False
    for index, char in enumerate(text):
        if in_string:
            if escaped:
                escaped = False
            elif char == "\\":
                escaped = True
            elif char == '"':
                in_string = False
            continue
        if char == '"':
            in_string = True
        elif char == "{":
            if depth == 0:
                start = index
            depth += 1
        elif char == "}" and depth > 0:
            depth -= 1
            if depth == 0 and start is not None:
                try:
                    parsed = json.loads(text[start : index + 1])
                    if isinstance(parsed, dict):
                        return parsed
                except json.JSONDecodeError:
                    start = None
    return None


def _grounding_prompt(labels: list[str] | None) -> str:
    target = (
        f"Find each of these objects: {', '.join(labels)}."
        if labels
        else "Find every distinct movable object in the workspace, including on platforms or shelves (ignore the robot arm itself)."
    )
    return "\n".join([
        "You are an object localizer for a robot workspace camera.",
        target,
        "Reply with ONLY a JSON array, no prose, no code fences:",
        '[{"label": "<short name>", "box": [x1, y1, x2, y2]}]',
        "x1,y1 = top-left and x2,y2 = bottom-right of a TIGHT bounding box,",
        "as integers on a 0-1000 grid over the image: 0,0 is the top-left corner and 1000,1000 the bottom-right corner.",
        "The box must hug the object tightly, including its base where it touches its supporting surface.",
        "If an object is not visible, omit it.",
    ])


def _task_query_prompt(phrase: str) -> str:
    """The strict task query: one object, and [] allowed. The labels wording above is a
    forced choice (a box for 72 of 72 absent objects), so it cannot say the object is missing."""
    return "\n".join([
        "You are an object localizer for a robot workspace camera.",
        f"The user asked for: {phrase}.",
        "The image may NOT contain it. Only box an object if it clearly is what the user asked for; "
        "never box a different kind of object instead.",
        "Reply with ONLY a JSON array, no prose, no code fences: [] when it is not in the image, otherwise",
        '[{"label": "<short name>", "box": [x1, y1, x2, y2]}]',
        "x1,y1 = top-left and x2,y2 = bottom-right of a TIGHT bounding box,",
        "as integers on a 0-1000 grid over the image: 0,0 is the top-left corner and 1000,1000 the bottom-right corner.",
    ])


def _single_call_prompt(phrase: str | None) -> str:
    """The single-call locate (LOCATE_MODES): every object as a compact row, the task's object first with its outline.

    Rows, not keyed objects: a box is 5 values, so the reply is ~12 tokens an object (the samples' keyed objects were
    ~25). Only the task's row carries an outline, which costs ~60 tokens; depth trims every box anyway."""
    Grid = ("x1,y1,x2,y2 is a TIGHT bounding box, integers on a 0-1000 grid over the image: 0,0 is the top-left corner "
            "and 1000,1000 the bottom-right corner.")
    Lines = ["You are an object localizer for a robot workspace camera.",
             "List every distinct movable object on the desk and in the bin. Ignore the robot arm, its gripper, the basket "
             "and the desk itself."]
    if phrase:
        return "\n".join([Lines[0],
            "Reply with ONLY one line of JSON, no prose, no code fences:",
            '{"objects": [["<short name>", x1, y1, x2, y2], ...], "task": <number>, "outline": [[x, y], ...]}',
            '"objects" is a complete inventory: EVERY distinct movable object on the desk and in the bin, scanning the '
            "image from top to bottom (a desk usually holds 5 to 15), each named by what it really is, not by the "
            "request below. Ignore the robot arm, its gripper, the basket and the desk itself.",
            Grid,
            f"The user asked for: {phrase}. The image may NOT contain it.",
            '"task" is the number of that object in the list, counting from 1, only if it clearly is what the user asked '
            "for: never a different kind of object (a can is not a bottle, a cup is not a box). null when none is. If "
            "several fit, the one nearest the robot (lowest in the image).",
            '"outline" is that object\'s outline: 6 to 8 [x, y] points on the same grid along its visible edge.'])
    return "\n".join(Lines + [
        "Reply with ONLY a JSON array of arrays on one line, no prose, no code fences:",
        '[["<short name>", x1, y1, x2, y2], ...]',
        Grid])


OUTLINE_ROW_IOU = 0.3


def _OutlinedRow(rows: list[Any], outline: list[Any], index: int) -> int:
    """The row whose box the task's outline lies on (the bounding box of its points overlapping it most, at least
    OUTLINE_ROW_IOU), else index. Rows and outline in the reply's own numbers."""
    try:
        Points = np.array([[float(P[0]), float(P[1])] for P in outline if isinstance(P, (list, tuple)) and len(P) == 2])
    except (TypeError, ValueError):
        return index
    if len(Points) < 3:
        return index
    Drawn = (*Points.min(axis=0), *Points.max(axis=0))
    Scores = []
    for Number, Row in enumerate(rows):
        try:
            Box = tuple(float(Value) for Value in Row[1:5]) if isinstance(Row, list) and len(Row) >= 5 else None
        except (TypeError, ValueError):
            Box = None
        if Box is not None:
            Scores.append((_Overlap(Drawn, Box), Number))
    Best = max(Scores, default=(0.0, index))
    return Best[1] if Best[0] >= OUTLINE_ROW_IOU else index


def _ParseRows(content: str, width: int, height: int, has_task: bool) -> tuple[list[dict[str, Any]], bool] | dict[str, Any]:
    """The single call's reply as detections ({"label", "box", "raw_box", "u", "v", "outline", "task_row"}) and whether
    the task's row was null (the object not in view); {"error": ...} when no array is in the reply.

    A row is ["label", x1, y1, x2, y2] or ["label", x1, y1, x2, y2, [[x, y], ...]]; the samples' {"label", "box"} objects
    are read too. The scale is read per box as _detect_parsed reads it (the 0-1000 grid, fractions, or pixels above
    1000), and its outline on the same scale."""
    Reply = _extract_json_object(content or "") if has_task else None
    if isinstance(Reply, dict) and (isinstance(Reply.get("task"), list) or "others" in Reply
                                    or (Reply.get("task") is None and isinstance(Reply.get("objects"), list) and "outline" not in Reply)):
        # {"task": row or null, "objects" (or "others"): [the other rows]}.
        Others = Reply.get("objects") if isinstance(Reply.get("objects"), list) else Reply.get("others")
        Rows = [Reply.get("task") if isinstance(Reply.get("task"), list) else None, *(Others if isinstance(Others, list) else [])]
    elif isinstance(Reply, dict) and isinstance(Reply.get("objects"), list):
        # {"objects": [rows], "task": its number from 1 or null, "outline": [[x, y], ...]}: the task's row first, its
        # outline appended, as the array form has it. A number that names no row is no task row.
        Rows = [Row for Row in Reply["objects"]]
        Number = Reply.get("task")
        Index = int(Number) - 1 if isinstance(Number, (int, float)) and not isinstance(Number, bool) and float(Number).is_integer() else -1
        Outline = Reply.get("outline") if isinstance(Reply.get("outline"), list) else []
        if 0 <= Index < len(Rows):
            # The outline is drawn on the object; a count is easy to get wrong ("task": 2 with the outline on the 3rd
            # row, 2026-10-03 15:48:40): the row the outline lies on is the task's, else the numbered one.
            Index = _OutlinedRow(Rows, Outline, Index)
        if 0 <= Index < len(Rows) and isinstance(Rows[Index], list) and len(Rows[Index]) >= 5:
            Task = [*Rows.pop(Index)[:5], Outline]
            Rows = [Task, *Rows]
        else:
            Rows = [None, *Rows]
    else:
        Rows = _extract_json_array(content or "")
    if Rows is None:
        return {"error": "Model reply contained no JSON array.", "raw": (content or "")[:400]}
    TaskAbsent = bool(has_task and Rows and Rows[0] is None)
    Detections: list[dict[str, Any]] = []
    for Index, Row in enumerate(Rows):
        Outline = None
        if isinstance(Row, dict):
            Label, Box = Row.get("label"), Row.get("box") or Row.get("bbox") or Row.get("bbox_2d")
            Outline = Row.get("outline")
        elif isinstance(Row, list) and len(Row) >= 5:
            Label, Box = Row[0], Row[1:5]
            Outline = Row[5] if len(Row) > 5 else None
        else:
            continue
        if not isinstance(Label, str) or not Label.strip() or not isinstance(Box, list) or len(Box) != 4:
            continue
        try:
            X1, Y1, X2, Y2 = (float(Value) for Value in Box)
        except (TypeError, ValueError):
            continue
        Raw = [X1, Y1, X2, Y2]
        Peak = max(X1, Y1, X2, Y2)
        ScaleX, ScaleY = (1.0, 1.0) if Peak <= 1.5 else (1000.0, 1000.0) if Peak <= 1000 else (float(width), float(height))
        X1, X2 = sorted((min(max(X1 / ScaleX, 0.0), 1.0), min(max(X2 / ScaleX, 0.0), 1.0)))
        Y1, Y2 = sorted((min(max(Y1 / ScaleY, 0.0), 1.0), min(max(Y2 / ScaleY, 0.0), 1.0)))
        if X2 - X1 < 1e-6 or Y2 - Y1 < 1e-6 or (X2 - X1) * (Y2 - Y1) > MAX_BOX_AREA_FRACTION:
            continue
        Points = None
        if isinstance(Outline, list):
            try:
                Points = [(min(max(float(P[0]) / ScaleX, 0.0), 1.0), min(max(float(P[1]) / ScaleY, 0.0), 1.0))
                          for P in Outline if isinstance(P, (list, tuple)) and len(P) == 2]
            except (TypeError, ValueError):
                Points = None
            Points = Points if Points and len(Points) >= 3 else None
        Detections.append({"label": Label.strip(), "box": (X1, Y1, X2, Y2), "raw_box": Raw, "u": (X1 + X2) / 2 * width,
                           "v": Y2 * height, "outline": Points, "task_row": bool(has_task and Index == 0)})
    return Detections, TaskAbsent


def _detect_single(image_data_url: str, task: dict[str, str] | None, model: str, width: int, height: int):
    """The single call through Ollama's native /api/chat, kept in the session record (grounding.jsonl, kind
    "single_call"): (detections, task absent) as _ParseRows reads them, or {"error": ...}."""
    started = time.time()
    prompt = _single_call_prompt(task["phrase"] if task else None)
    payload = {
        "model": model, "stream": False, "think": False, "keep_alive": OLLAMA_OPTIONS["keep_alive"],
        "options": {"num_ctx": OLLAMA_OPTIONS["num_ctx"], "num_batch": OLLAMA_OPTIONS["num_batch"],
                    "temperature": 0, "num_predict": GROUNDING_NUM_PREDICT},
        "messages": _chat_messages(prompt, image_data_url),
    }
    content, stats = None, {}
    try:
        response = _http_json(f"{OLLAMA_BASE}/api/chat", payload, timeout=LOCATE_TIMEOUT_S)
        content = response["message"]["content"]
        stats = {key: response.get(key) for key in ("done_reason", "prompt_eval_count", "eval_count")}
        if isinstance(response.get("load_duration"), (int, float)):
            stats["load_ms"] = round(response["load_duration"] / 1e6)
        result = _ParseRows(content, width, height, task is not None)
    except (urllib.error.URLError, OSError, TimeoutError) as exc:
        result = {"error": f"Ollama request failed: {exc}"}
    except (KeyError, IndexError, TypeError, json.JSONDecodeError):
        result = {"error": "Ollama returned an unreadable response."}
    SessionLog.Append("grounding.jsonl", {
        "kind": "single_call", "model": model, "labels": None, "temperature": 0, "image_size": [width, height],
        "prompt": prompt, "image": image_data_url, "reply": content,
        "result": result if isinstance(result, dict) else {"detections": result[0], "task_absent": result[1]},
        "duration_ms": round((time.time() - started) * 1000), "endpoint": "/api/chat", "task_object": task, **stats,
    })
    return result


def _chat_messages(prompt: str, image_data_url: str) -> list[dict[str, Any]]:
    """The prompt, then the image (bare base64), as separate messages: how /v1 split the
    content parts. Ollama puts a message's images before its text, so one merged message
    moved the image first (1098 prompt tokens, not 1103) and on the recorded frames gave
    fenced replies with duplicate keys that dropped objects."""
    return [{"role": "user", "content": prompt},
            {"role": "user", "content": "", "images": [image_data_url.split(",", 1)[-1]]}]


def _detect_once(
    image_data_url: str,
    labels: list[str] | None,
    model: str,
    width: int,
    height: int,
    temperature: float,
    prompt: str | None = None,
    record: dict[str, Any] | None = None,
) -> list[dict[str, Any]] | dict[str, Any]:
    """One VLM detection pass, kept in the session record (grounding.jsonl).

    prompt replaces the grounding prompt (the task query); record adds to, or
    overrides, the record's fields ({"kind": "task_query", ...}).
    Returns detections, or {'error': ...} on failure.
    """
    started = time.time()
    result, content, stats = _detect_parsed(image_data_url, labels, model, width, height, temperature, prompt)
    SessionLog.Append("grounding.jsonl", {
        "kind": "grounding", "model": model, "labels": labels, "temperature": temperature,
        "image_size": [width, height], "prompt": prompt or _grounding_prompt(labels), "image": image_data_url,
        "reply": content, "result": result, "duration_ms": round((time.time() - started) * 1000),
        "endpoint": "/api/chat", **stats, **(record or {}),
    })
    return result


def _detect_parsed(
    image_data_url: str,
    labels: list[str] | None,
    model: str,
    width: int,
    height: int,
    temperature: float,
    prompt: str | None = None,
) -> tuple[list[dict[str, Any]] | dict[str, Any], str | None, dict[str, Any]]:
    """One VLM detection pass through Ollama's native /api/chat:
    (detections or {'error': ...}, the model's raw reply, Ollama's token and load stats)."""
    payload = {
        "model": model,
        "stream": False,
        # Skip hidden reasoning: each detection is several passes, and thinking
        # multiplies the latency of every one of them (~4x measured).
        "think": False,
        "keep_alive": OLLAMA_OPTIONS["keep_alive"],
        "options": {"num_ctx": OLLAMA_OPTIONS["num_ctx"], "num_batch": OLLAMA_OPTIONS["num_batch"],
                    "temperature": temperature, "num_predict": GROUNDING_NUM_PREDICT},
        "messages": _chat_messages(prompt or _grounding_prompt(labels), image_data_url),
    }
    try:
        response = _http_json(f"{OLLAMA_BASE}/api/chat", payload, timeout=LOCATE_TIMEOUT_S)
        content = response["message"]["content"]
    except (urllib.error.URLError, OSError, TimeoutError) as exc:
        return {"error": f"Ollama request failed: {exc}"}, None, {}
    except (KeyError, IndexError, TypeError, json.JSONDecodeError):
        return {"error": "Ollama returned an unreadable response."}, None, {}
    stats = {key: response.get(key) for key in ("done_reason", "prompt_eval_count", "eval_count")}
    if isinstance(response.get("load_duration"), (int, float)):
        stats["load_ms"] = round(response["load_duration"] / 1e6)

    boxes = _extract_json_array(content or "")
    if boxes is None:
        return {"error": "Model reply contained no JSON array.", "raw": (content or "")[:400]}, content, stats

    detections: list[dict[str, Any]] = []
    for item in boxes:
        if not isinstance(item, dict):
            continue
        label = item.get("label")
        box = item.get("box") or item.get("bbox") or item.get("bbox_2d")
        if not isinstance(label, str) or not isinstance(box, list) or len(box) != 4:
            continue
        try:
            x1, y1, x2, y2 = (float(value) for value in box)
        except (TypeError, ValueError):
            continue
        raw = [x1, y1, x2, y2]
        # The prompt asks for Qwen-VL's native 0-1000 grid. Guessing the scale
        # per box read any box under the image height (720 px) as pixels, which
        # put every small object about 20% too far left and down. Fractions, and
        # pixels above 1000, are still recognised here; a refine pass, which knows
        # roughly where the object is, re-checks the reading (_ReadRefineBox).
        peak = max(x1, y1, x2, y2)
        if peak <= 1.5:
            pass  # fractions
        elif peak <= 1000:
            x1, y1, x2, y2 = x1 / 1000, y1 / 1000, x2 / 1000, y2 / 1000
        else:
            x1, x2 = x1 / width, x2 / width
            y1, y2 = y1 / height, y2 / height
        x1, x2 = sorted((min(max(x1, 0.0), 1.0), min(max(x2, 0.0), 1.0)))
        y1, y2 = sorted((min(max(y1, 0.0), 1.0), min(max(y2, 0.0), 1.0)))
        if x2 - x1 < 1e-6 or y2 - y1 < 1e-6:
            continue
        if (x2 - x1) * (y2 - y1) > MAX_BOX_AREA_FRACTION:
            continue
        detections.append({
            "label": label.strip(),
            "box": (x1, y1, x2, y2),
            "raw_box": raw,
            # Bottom-center of the box = where the object meets the table plane.
            "u": (x1 + x2) / 2 * width,
            "v": y2 * height,
        })
    return detections, content, stats


def _label_key(label: str) -> str:
    return " ".join(label.lower().split())


# Zoom-refine: VLM boxes on the FULL frame can carry a systematic shift of
# ~100 px when the scene is busy (observed: box drawn onto the gripper next to
# the object). Boxes on a tight crop are far more accurate, so after the coarse
# pass we crop around an object and re-detect in the zoomed view. Each crop is
# one more sequential model call, so with a task object only the task object,
# and objects whose samples disagree, are refined (locate_objects).
REFINE_EXPAND = 1.0      # expand the coarse box by 100% on each side
REFINE_MIN_CROP_PX = 220  # never crop tighter than this (context helps)


def _Overlap(A, B) -> float:
    """Intersection over union of two (x1, y1, x2, y2) boxes."""
    W = min(A[2], B[2]) - max(A[0], B[0])
    H = min(A[3], B[3]) - max(A[1], B[1])
    if W <= 0 or H <= 0:
        return 0.0
    Union = (A[2] - A[0]) * (A[3] - A[1]) + (B[2] - B[0]) * (B[3] - B[1]) - W * H
    return W * H / Union if Union > 0 else 0.0


def _ReadRefineBox(Raw, Parsed, Expected, CropWidth: int, CropHeight: int):
    """The reading of a refine reply (0-1000 grid, fractions or crop pixels) that overlaps
    where the coarse pass put the object inside the crop; None when no reading does.

    A crop is at most a few hundred pixels, so a model answering in crop pixels gives
    numbers under 1000 that the grid default would silently shrink toward the corner.
    """
    if not Raw:
        return Parsed if _Overlap(Parsed, Expected) > 0 else None
    X1, Y1, X2, Y2 = Raw
    Readings = [tuple(Parsed)]
    if max(Raw) > 1.5:
        Readings.append((X1 / CropWidth, Y1 / CropHeight, X2 / CropWidth, Y2 / CropHeight))
    Scored = []
    for Box in Readings:
        Box = (*sorted((Box[0], Box[2])), *sorted((Box[1], Box[3])))
        Box = (Box[0], Box[2], Box[1], Box[3])
        if Box[2] > 1.0 + 1e-6 or Box[3] > 1.0 + 1e-6 or Box[0] < -1e-6 or Box[1] < -1e-6:
            continue
        Scored.append((_Overlap(Box, Expected), Box))
    if not Scored:
        return None
    Best = max(Scored, key=lambda Item: Item[0])  # ties keep the grid reading, listed first
    return Best[1] if Best[0] > 0 else None


def _refine_detection(
    image: np.ndarray,
    label: str,
    coarse_box_px: tuple[float, float, float, float],
    model: str,
) -> tuple[float, float, tuple[float, float, float, float]] | None:
    """Re-detect `label` in a crop around the coarse box. Returns a refined
    full-frame contact pixel (u, v) and the refined normalized box, or None
    if the refine pass fails."""
    height, width = image.shape[:2]
    x1, y1, x2, y2 = coarse_box_px
    pad_u = max((x2 - x1) * REFINE_EXPAND, (REFINE_MIN_CROP_PX - (x2 - x1)) / 2)
    pad_v = max((y2 - y1) * REFINE_EXPAND, (REFINE_MIN_CROP_PX - (y2 - y1)) / 2)
    u1 = int(max(0, x1 - pad_u))
    v1 = int(max(0, y1 - pad_v))
    u2 = int(min(width, x2 + pad_u))
    v2 = int(min(height, y2 + pad_v))
    if u2 - u1 < 40 or v2 - v1 < 40:
        return None

    crop = image[v1:v2, u1:u2]
    ok, encoded = cv2.imencode(".jpg", crop, [cv2.IMWRITE_JPEG_QUALITY, 92])
    if not ok:
        return None
    crop_url = "data:image/jpeg;base64," + base64.b64encode(encoded.tobytes()).decode()

    result = _detect_once(crop_url, [label], model, u2 - u1, v2 - v1, temperature=0)
    if isinstance(result, dict) or not result:
        return None
    # Take the detection whose label matches; fall back to the first one.
    match = next(
        (d for d in result if _label_key(d["label"]) == _label_key(label)),
        result[0],
    )
    crop_w, crop_h = u2 - u1, v2 - v1
    expected = ((x1 - u1) / crop_w, (y1 - v1) / crop_h, (x2 - u1) / crop_w, (y2 - v1) / crop_h)
    refined = _ReadRefineBox(match.get("raw_box"), match["box"], expected, crop_w, crop_h)
    if refined is None:
        return None
    bx1, by1, bx2, by2 = refined
    contact_u = u1 + (bx1 + bx2) / 2 * crop_w
    contact_v = v1 + by2 * crop_h
    box = ((u1 + bx1 * crop_w) / width, (v1 + by1 * crop_h) / height,
           (u1 + bx2 * crop_w) / width, (v1 + by2 * crop_h) / height)
    return (contact_u, contact_v, box)


def _median(values: list[float]) -> float:
    ordered = sorted(values)
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return (ordered[middle - 1] + ordered[middle]) / 2


ROBOT_PARTS_REASON = "the robot's own parts are not objects"


def _RobotPartReason(label: str, task_label: str | None = None) -> str | None:
    """Why a label names the robot itself (whole words, simple plurals), else None. A loose-part word
    the task's own label asks for (LOOSE_PART_WORDS) does not."""
    words = set(re.findall(r"[a-z]+", label.lower()))
    words |= {word[:-1] for word in words if word.endswith("s")}
    asked = {_Singular(word) for word in re.findall(r"[a-z]+", (task_label or "").lower())} & LOOSE_PART_WORDS
    if words & (ROBOT_PART_WORDS - asked):
        return ROBOT_PARTS_REASON
    if words & BIN_WORDS:
        return "the robot's own bin is not measured as an object"
    return None


def _SamplesAgree(first: list[dict[str, Any]], second: list[dict[str, Any]]) -> bool:
    """Whether two samples boxed the same objects, whatever they called them: every object box
    of each has one in the other whose contact pixel is within SAMPLE_AGREE_PX.

    The temperature-0.5 sample renames objects ("rubber eraser"), so the identical label sets
    once required never happened (0 of 21 locates on 2026-09-28/29). The robot's own parts are
    left out (they are withheld anyway), and samples with no object do not agree: a third may
    still find one.
    """
    spots = [[(d["u"], d["v"]) for d in sample if not _RobotPartReason(d["label"])] for sample in (first, second)]
    if not spots[0] or not spots[1]:
        return False

    def covered(these, those) -> bool:
        return all(any(math.hypot(u - u2, v - v2) <= SAMPLE_AGREE_PX for u2, v2 in those) for u, v in these)

    return covered(spots[0], spots[1]) and covered(spots[1], spots[0])


def _Singular(word: str) -> str:
    """The web app's Singular (AiMeasurements.ts): boxes -> box, glasses -> glass, pens -> pen."""
    return re.sub(r"([^s])s$", r"\1", re.sub(r"(?:x|s|ch|sh)es$", lambda match: match.group(0)[:-2], word))


def _TaskHead(label: str) -> str | None:
    """The noun that names the task object ("eraser" in "red eraser"); None for "black thing"."""
    words = [_Singular(word) for word in re.findall(r"[a-z]+", label.lower())]
    return words[-1] if words and words[-1] not in GENERIC_TASK_WORDS else None


def _NamesTask(label: str, head: str | None) -> bool:
    """Whether a detected label names the task object, as a whole word like the web app's matcher."""
    return bool(head) and head in {_Singular(word) for word in re.findall(r"[a-z]+", label.lower())}


def _StandIn(label: str, task_label: str) -> bool:
    """Whether a label may be the task object under another name, when no label names it: it says one of
    the task's colours, or its noun (before any preposition) names no kind of thing. The 2026-09-30 05:24
    "black servo motor thing" was "small black object", "black object on tape" or "small black box", and,
    left unrefined, reached the model as [UNRELIABLE: coarse box, not refined]."""
    words = [_Singular(word) for word in re.findall(r"[a-z]+", label.lower())]
    colours, asked = set(words) & TASK_COLOUR_WORDS, set(re.findall(r"[a-z]+", task_label.lower())) & TASK_COLOUR_WORDS
    if colours & asked:
        return True
    if colours and asked:
        return False  # "white object" is not the black servo
    cut = next((index for index, word in enumerate(words) if word in LABEL_PREPOSITIONS), len(words))
    nouns = [word for word in words[:cut] if word not in TASK_COLOUR_WORDS]
    return not nouns or nouns[-1] in GENERIC_TASK_WORDS


def _LandsOn(query_box, box) -> bool:
    """A task-query box picks a measured box when its centre lies inside it or they overlap (IoU)."""
    cx, cy = (query_box[0] + query_box[2]) / 2, (query_box[1] + query_box[3]) / 2
    return (box[0] <= cx <= box[2] and box[1] <= cy <= box[3]) or _Overlap(query_box, box) >= TASK_MATCH_IOU


def _TaskObject(task_object: Any) -> dict[str, str] | None:
    """{"phrase", "label"} with whitespace collapsed and lengths capped, or None."""
    if not isinstance(task_object, dict):
        return None
    phrase = " ".join(str(task_object.get("phrase") or "").split())[:TASK_PHRASE_MAX_CHARS]
    label = " ".join(str(task_object.get("label") or "").split())[:TASK_PHRASE_MAX_CHARS] or phrase
    return {"phrase": phrase, "label": label} if phrase else None


def _ImplausibleReason(measured: dict[str, Any]) -> str | None:
    """Why a depth measurement cannot be a real workspace object, else None."""
    size = max(measured["size_cm"])
    if measured["distance_cm"] > MAX_DISTANCE_CM:
        return f"not measurable: {measured['distance_cm']:.0f} cm from the camera (limit {MAX_DISTANCE_CM:g})"
    if abs(measured["x"]) > MAX_ABS_X_CM or not Y_RANGE_CM[0] <= measured["y"] <= Y_RANGE_CM[1]:
        return f"not measurable: X {measured['x']:.0f} Y {measured['y']:.0f} is outside the workspace"
    if not HEIGHT_RANGE_CM[0] <= measured["height_cm"] <= HEIGHT_RANGE_CM[1]:
        return f"not measurable: {measured['height_cm']:.1f} cm tall"
    if size > MAX_FOOTPRINT_CM:
        return f"not measurable: {size:.0f} cm across"
    return None


def _Hint(hints: dict[str, float], label: str) -> dict[str, float]:
    """{"undeclared_support_z": z} for a withheld entry measured just now with that hint (popped), else {}."""
    return {"undeclared_support_z": hints.pop(label)} if label in hints else {}


def _OutsideCalibrationZ(model: dict[str, Any] | None, measured: dict[str, Any]) -> float | None:
    """How far (cm) an object's base or top lies outside the heights the calibration is known good at:
    from the desk (its objects measure well, 10 cm under the tips) to the highest calibration tip."""
    bounds = ((model or {}).get("observed_tip_bounds_cm") or {}).get("z")
    if not bounds or len(bounds) != 2:
        return None
    low = min(float(bounds[0]), float((model or {}).get("table_z", bounds[0])))
    return round(max(0.0, low - measured["z"], measured["top_z"] - float(bounds[1])), 1)


def _Distinct(diagnostics: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """The measured boxes with each object once: a box measured from the task query alone (task_query_only)
    that lands on a box the samples measured is that object again (the samples' group was withheld, so the
    query's box landed on no measured entry and was measured a second time under the task's label)."""
    kept: list[dict[str, Any]] = []
    for item in diagnostics:
        box = item.get("box")
        if item.get("task_query_only") and box and any(other.get("box") and _LandsOn(box, other["box"])
                                                      for other in kept if not other.get("task_query_only")):
            continue
        kept.append(item)
    return kept


def _OneTaskObject(matches: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """The task's matches with same-spot twins folded into one: the grounding named one object differently between
    samples, so each label became its own entry, all under the task query's box, and no scene was registered ("a unique
    task object ... not measured"; every task stopped scene_unverified). The kept one has a measured support, then a
    refined box, then the most samples; the others stay listed as ordinary objects (task_match false, task_twin_of its
    label), since the web app fails a reply whose task match does not carry the scene. Matches at several spots stay."""
    if len(matches) < 2 or any(math.hypot(item["x"] - other["x"], item["y"] - other["y"]) > TASK_TWIN_CM
                               for item in matches for other in matches):
        return matches
    kept = max(matches, key=lambda item: (bool(item.get("support_geometry")), bool(item.get("refined")),
                                          item.get("samples") or 0, item.get("valid_fraction") or 0.0))
    for item in matches:
        if item is not kept:
            item.update(task_match=False, task_twin_of=kept["label"])
    return [kept]


def _SceneTripwire(diagnostics: list[dict[str, Any]], camera_check: dict[str, Any]) -> tuple[str, str] | None:
    """(reason, one sentence for the user) when desk-area supports, or for an "assumed" camera with no desk in view
    the basket rim (TRIPWIRE_ASSUMED_RIM), say the camera moved, else None.

    Only measured surroundings count: a base that fell back to the table height says
    nothing about where the desk really is. A single object off the desk keeps its
    measurement (taken on the surface it stands on) unless the desk check is near its limits.
    """
    if (TRIPWIRE_ASSUMED_RIM and camera_check.get("alignment_source") == "robot" and camera_check.get("robot_check") == "assumed"
            and camera_check.get("rim_check") == "moved"):
        yaw, shift = camera_check.get("rim_yaw_deg"), camera_check.get("rim_shift_cm") or []
        return (f"no calibrated desk in view and the gripper tip could not check the camera; the basket rim reads {yaw} deg, "
                f"{' / '.join(f'{value:+.2f}' for value in shift)} cm against the calibration's frames",
                "The camera may have moved on the robot: the basket rim is not where the calibration saw it, and the "
                "gripper tip could not check it. Lock the camera head, then run Automatic calibration.")
    angle, desk_offset = camera_check.get("angle_deg") or 0.0, camera_check.get("offset_cm") or 0.0
    desk_near_limit = (angle >= TRIPWIRE_SINGLE_DESK_FRACTION * CameraDrift.MaxNormalChangeDeg
                       or abs(desk_offset) >= TRIPWIRE_SINGLE_DESK_FRACTION * CameraDrift.MaxOffsetChangeCm)
    for item in diagnostics:
        offset = item.get("desk_offset_cm")
        if (desk_near_limit and item.get("desk_area") and item.get("support_branch") == "desk" and item.get("support_measured")
                and offset is not None and abs(offset) > TRIPWIRE_SINGLE_CM):
            return (f"'{item['label']}' stands on a desk surface {offset:+.1f} cm from the calibrated desk "
                    f"(limit {TRIPWIRE_SINGLE_CM:g} cm) while the desk reads {angle:.2f} deg / {desk_offset:+.2f} cm, "
                    f"over half its limits",
                    f"The camera may have moved since calibration: the desk is near its limit and the "
                    f"{item['label']} sits {abs(offset):.1f} cm off it. If it is not standing on something, "
                    "lock the camera head, then run Automatic calibration.")
    # Objects over a declared surface (DeclaredSurfaces.json) are not on the desk: three cans on a shelf
    # 5 cm under the desk line read a shared -5 cm. MeasureBox leaves them out of the desk's area already.
    # Confident automatic supports are likewise separate from the desk, but are
    # enabled only after this capture has passed the independent camera gate.
    # Each object counts once: a withheld task object is measured again from the task query's box
    # (task_query_only, its box on the samples' one), and was counted twice.
    offsets = [item["desk_ring_offset_cm"] for item in _Distinct(diagnostics)
               if item.get("desk_area") and not item.get("declared_surface") and not item.get("automatic_surface")
               and item.get("desk_ring_offset_cm") is not None]
    for sign in (1.0, -1.0):
        beyond = [value for value in offsets if sign * value > TRIPWIRE_SHARED_CM]
        if len(beyond) >= TRIPWIRE_SHARED_COUNT:
            middle = float(np.median(beyond))
            shared = [value for value in beyond if abs(value - middle) <= 1.0]
            if len(shared) >= TRIPWIRE_SHARED_COUNT:
                return (f"{len(shared)} desk-area objects sit on a surface {middle:+.1f} cm from the calibrated desk "
                        f"(limit {TRIPWIRE_SHARED_CM:g} cm for {TRIPWIRE_SHARED_COUNT})",
                        "The camera or the cart moved since calibration: objects on the desk do not sit on the "
                        "calibrated desk. Lock the camera head, then run Automatic calibration. If they stand on a "
                        "platform or a shelf, declare it in back end/DeclaredSurfaces.json.")
    return None


def _Record(result: dict[str, Any], labels, depth: dict[str, Any] | None, diagnostics: list[dict[str, Any]],
            task_object: dict[str, str] | None = None) -> None:
    """The locate record (grounding.jsonl) with the raw depth beside it, for exact review."""
    stored = SessionLog.StoreDepth(depth["depth_mm"]) if depth is not None else None
    SessionLog.Append("grounding.jsonl", {"kind": "locate", "labels": labels, "task_object": task_object, **result,
                                          "diagnostics": diagnostics, "depth_seq": (depth or {}).get("seq"), **(stored or {})})


# The single call's depth work (LOCATE_MODES). An outline is grown OUTLINE_GROW_PX before it limits the object's
# pixels: the model's edge points are a few pixels loose either way. Every box is measured as the samples' boxes
# were (tight); the task's box is also checked (_DepthRefine): padded DEPTH_PAD_FRACTION (at least DEPTH_PAD_MIN_PX),
# an object that reaches a side of it was cut by the box, which grows there by half the box (at least
# DEPTH_GROW_MIN_PX), up to DEPTH_REFINE_ROUNDS times and DEPTH_GROW_MAX_AREA times the padded box's area. A task object
# that ends inside its box is whole ("depth_refined") and reads as refined, as a zoom-refined one did.
OUTLINE_GROW_PX = 8
OUTLINE_MIN_BOX_SHARE = 0.25
DEPTH_PAD_FRACTION = 0.15
DEPTH_PAD_MIN_PX = 6
DEPTH_GROW_MIN_PX = 16
DEPTH_REFINE_ROUNDS = 3
DEPTH_GROW_MAX_AREA = 4.0
# Errors that mean no object stands in the box at all (the single call's guard against a box on nothing).
NO_DEPTH_OBJECT = ("nothing stands above", "object region too small", "no depth inside the box", "of the box has depth within")


def _NoDepthObject(reason: str | None) -> bool:
    return bool(reason) and any(text in reason for text in NO_DEPTH_OBJECT)


def _OutlineMask(outline, box, width: int, height: int, mirrored: bool) -> np.ndarray | None:
    """The model's outline (capture fractions) as a native full-frame mask grown OUTLINE_GROW_PX, or None when it is
    degenerate or covers under OUTLINE_MIN_BOX_SHARE of its box (it then limits nothing)."""
    if not outline:
        return None
    Points = np.array([[(1.0 - X if mirrored else X) * width, Y * height] for X, Y in outline], dtype=np.float64)
    Mask = np.zeros((height, width), np.uint8)
    cv2.fillPoly(Mask, [np.round(Points).astype(np.int32)], 1)
    U0, V0, U1, V1 = DepthGeometry.BoxToNative(box, width, height, mirrored)
    Inside = int(Mask[V0:V1, U0:U1].sum())
    if Inside < OUTLINE_MIN_BOX_SHARE * (U1 - U0) * (V1 - V0):
        return None
    Size = 2 * OUTLINE_GROW_PX + 1
    return cv2.dilate(Mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (Size, Size))).astype(bool)


def _CaptureBox(native, width: int, height: int, mirrored: bool) -> tuple[float, float, float, float]:
    """A native pixel box [u0, v0, u1, v1) as a normalized capture box (DepthGeometry.BoxToNative's inverse)."""
    U0, V0, U1, V1 = native
    X1, X2 = U0 / width, U1 / width
    if mirrored:
        X1, X2 = 1.0 - X2, 1.0 - X1
    return (max(0.0, X1), max(0.0, V0 / height), min(1.0, X2), min(1.0, V1 / height))


def _Grown(native, padding, width: int, height: int, sides=(True, True, True, True)):
    U0, V0, U1, V1 = native
    return (int(max(0, U0 - padding[0])) if sides[0] else U0, int(max(0, V0 - padding[1])) if sides[1] else V0,
            int(min(width, U1 + padding[0])) if sides[2] else U1, int(min(height, V1 + padding[1])) if sides[3] else V1)


def _DepthRefine(measure, label: str, box, entry: dict[str, Any], note: dict[str, Any], width: int, height: int,
                 mirrored: bool, diagnostics: list[dict[str, Any]], hints: dict[str, float], only=None):
    """(box, entry, whole), the depth's zoom-refine for the task's object (module notes).

    A tight box always touches its object, so the check measures the box padded DEPTH_PAD_FRACTION: an object that ends
    inside that on every side was whole in the box, and the tight box's measurement stands (whole). One that reaches
    the padded box's side (not the image's edge) was cut by a box off the object: those sides grow by half the box and
    it is measured again, until it ends inside (whole: the grown box's measurement, if its object's centre is within
    the first box grown by half) or DEPTH_GROW_MAX_AREA is reached. Anything else keeps the tight box's measurement,
    not whole. Only the measurement kept stays in diagnostics, and the hints of the ones tried are dropped."""
    Start, Saved = len(diagnostics), dict(hints)
    Tight = DepthGeometry.BoxToNative(box, width, height, mirrored)
    Size = (Tight[2] - Tight[0], Tight[3] - Tight[1])
    Native = _Grown(Tight, (max(DEPTH_PAD_MIN_PX, DEPTH_PAD_FRACTION * Size[0]), max(DEPTH_PAD_MIN_PX, DEPTH_PAD_FRACTION * Size[1])),
                    width, height)
    Allowed = _Grown(Tight, (0.5 * Size[0], 0.5 * Size[1]), width, height)
    Limit = DEPTH_GROW_MAX_AREA * (Native[2] - Native[0]) * (Native[3] - Native[1])
    Kept = (box, entry, False)
    for Round in range(DEPTH_REFINE_ROUNDS + 1):
        Box = _CaptureBox(Native, width, height, mirrored)
        Trace: dict[str, Any] = {}
        Again, _ = measure(label, (Box[0] + Box[2]) / 2 * width, Box[3] * height, Box,
                           {**note, "refine": "depth check" if Round == 0 else "depth grown"}, only=only if Round == 0 else None, trace=Trace)
        Component = Trace.get("component_px")
        if Again is None or not Component or max(Again.get("size_cm") or [0.0]) > MAX_FOOTPRINT_CM:
            break
        U0, V0, U1, V1 = Native
        Touched = (Component[0] <= U0 + 1 and U0 > 0, Component[1] <= V0 + 1 and V0 > 0,
                   Component[2] >= U1 - 1 and U1 < width, Component[3] >= V1 - 1 and V1 < height)
        if not any(Touched):
            Centre = ((Component[0] + Component[2]) / 2, (Component[1] + Component[3]) / 2)
            if Round == 0:
                Kept = (box, entry, True)
            elif Allowed[0] <= Centre[0] <= Allowed[2] and Allowed[1] <= Centre[1] <= Allowed[3]:
                Kept = (Box, Again, True)
            break
        Native = _Grown(Native, (max(DEPTH_GROW_MIN_PX, 0.5 * Size[0]), max(DEPTH_GROW_MIN_PX, 0.5 * Size[1])), width, height, Touched)
        if (Native[2] - Native[0]) * (Native[3] - Native[1]) > Limit:
            break
    if Kept[1] is entry:
        del diagnostics[Start:]  # the tight box's measurement (appended before) stands
    else:
        del diagnostics[Start - 1:-1]  # the grown box's, the last one
    hints.clear()
    hints.update(Saved)
    return Kept


def _MeasureRow(measure, row: dict[str, Any], label: str, width: int, height: int, mirrored: bool,
                diagnostics: list[dict[str, Any]], hints: dict[str, float], note: dict[str, Any]):
    """(entry, reason, box, whole) for one row of the single call: its box, inside its outline when it has one; for the
    task's row the depth refine (_DepthRefine). Only the measurement kept stays in diagnostics."""
    Before = len(diagnostics)
    Box = tuple(row["box"])
    Only = _OutlineMask(row.get("outline"), Box, width, height, mirrored)
    Note = {**note, "outline": Only is not None}
    Entry, Reason = measure(label, row["u"], row["v"], Box, Note, only=Only)
    if Entry is None and Only is not None and _NoDepthObject(Reason):
        # The outline missed the object's pixels: the box alone.
        del diagnostics[Before:]
        hints.pop(label, None)
        Only, Note = None, {**note, "outline": False, "outline_missed": True}
        Entry, Reason = measure(label, row["u"], row["v"], Box, Note)
    Whole = False
    if Entry is not None and row.get("task_row"):
        Box, Entry, Whole = _DepthRefine(measure, label, Box, Entry, Note, width, height, mirrored, diagnostics, hints, Only)
    if diagnostics[Before:]:
        diagnostics[-1]["depth_refined"] = Whole
    return Entry, Reason, Box, Whole


def locate_objects(
    image_data_url: str,
    labels: list[str] | None = None,
    model: str | None = None,
    depth: dict[str, Any] | None = None,
    mirrored: bool | None = None,
    arm: dict[str, Any] | None = None,
    task_object: dict[str, Any] | None = None,
    locate_mode: str | None = None,
) -> dict[str, Any]:
    """Detect objects with the VLM and measure them in the AI frame.

    depth, when given, is {"depth_mm": native depth aligned to this capture,
    "intrinsics": the depth camera's, "mirrored": whether the capture is flipped,
    "seq": its frame number}. mirrored says whether the capture is flipped
    left-right (None: as calibrated). arm is the bridge's CaptureArmPose for the
    capture ({"ok", "joints", "servos_raw"}): it checks the camera against the
    gripper's tape and masks the arm out of the measurements. task_object is the
    task's {"phrase": "black thing on the desk", "label": "black thing"}: every
    object gets task_match (the strict task query's box lands on it) and
    task_query_only (measured only because the query found it), both false without one.
    locate_mode picks the path (LOCATE_MODES, _LocateMode): "single_call" by default.
    """
    Started = time.perf_counter()
    Previous = None
    CarryError = None
    Carrying = isinstance(task_object, dict) and task_object.get("carrying") is True
    if Carrying:
        try:
            if SceneSafety is None or not arm or arm.get("ok") is not True or not arm.get("joints"):
                raise ValueError("a carry refresh requires a trusted current arm pose")
            Previous = SceneSafety.Previous(task_object.get("carry_scene_id"), calibration.camera_model())
        except ValueError as Exc:
            CarryError = str(Exc)
    # Starting any fresh perception attempt revokes the previous snapshot, even
    # if decoding, calibration, depth or scene extraction subsequently fails.
    Generation = SceneSafety.Invalidate() if SceneSafety is not None else None
    calibration_status = calibration.status()
    image = _decode_image(image_data_url)
    if image is None:
        return {"ok": False, "error": "Could not decode the image."}
    height, width = image.shape[:2]
    camera_model = calibration.camera_model()
    use_depth = bool(
        depth is not None
        and DepthGeometry.ModelUsesDepthCamera(camera_model, depth.get("intrinsics"))
        and depth["depth_mm"].shape[:2] == (height, width)
    )

    # The camera gate, on this capture's own depth and before any model call: a camera
    # that moved since calibration, or cannot be checked (no calibration, a manual or
    # webcam one without a baseline), makes every position wrong.
    native = cv2.flip(image, 1) if use_depth and depth.get("mirrored") else image
    camera_check = CameraDrift.CheckCapture(
        depth["depth_mm"] if use_depth else None, depth["intrinsics"] if use_depth else None, camera_model,
        (depth or {}).get("seq"), native if use_depth else None, arm)
    # The cart was rolled against the desk and the gripper tape says the camera is still where it
    # was on the arm: the desk measured on this capture becomes the baseline, saved for the captures
    # after it and used for this one's supports whether or not the save succeeds.
    remeasured = CameraDrift.TakeRemeasuredReference(camera_check)
    if remeasured is not None:
        saved, why = calibration.save_remeasured_desk(remeasured, (camera_model.get("drift_reference") or {}).get("sha256"),
                                                      camera_check)
        camera_check["desk_saved"] = saved
        if not saved:
            camera_check["desk_save_error"] = why
        camera_model = {**camera_model, "drift_reference": remeasured}
    task = _TaskObject(task_object)
    if not camera_check["ok"]:
        blocked = {"ok": False, "error_code": camera_check["error_code"], "error": camera_check["error"],
                   "camera_check": camera_check}
        _Record(blocked, labels, depth, [], task)
        return blocked

    # A failure after the gate is recorded too: its depth and gate numbers are what a review needs.
    picked_model = _pick_model(model)
    if not picked_model:
        failed = {"ok": False, "error": "No Ollama model available for localization."}
        _Record({**failed, "camera_check": camera_check}, labels, depth if use_depth else None, [], task)
        return failed

    # Sample the detector up to LOCATE_SAMPLES times (temperature 0 first for
    # the model's best single answer, then >0 so reruns can outvote a fluke).
    # The single call needs depth to trim its boxes; named labels and a carry refresh keep the earlier path.
    mode = _LocateMode(locate_mode)
    if mode == "single_call" and (not use_depth or labels or Carrying):
        mode = "vlm"
    checked_ms = round((time.perf_counter() - Started) * 1000, 1)
    samples: list[list[dict[str, Any]]] = []
    for index in range(LOCATE_SAMPLES if mode == "vlm" else 0):
        result = _detect_once(
            image_data_url, labels, picked_model, width, height,
            temperature=0 if index == 0 else 0.5,
        )
        if isinstance(result, dict):
            if not samples:
                failed = {"ok": False, **result}
                _Record({**failed, "camera_check": camera_check, "model": picked_model}, labels,
                        depth if use_depth else None, [], task)
                return failed
            break  # keep whatever good samples we already have
        samples.append(result)

        # Early exit: the first two samples boxed the same objects at the same
        # spots, so a third opinion adds nothing.
        if len(samples) == 2 and _SamplesAgree(samples[0], samples[1]):
            break

    # The strict task query: one call at temperature 0 that may answer [], so its box
    # says which measured object the task means, whatever the samples called it.
    query_boxes: list[tuple[float, float, float, float]] = []
    query_names: list[str] = []  # the query's own name for each box ("USB drive")
    query_answered = False
    if task and mode == "vlm":
        answer = _detect_once(image_data_url, None, picked_model, width, height, temperature=0,
                              prompt=_task_query_prompt(task["phrase"]),
                              record={"kind": "task_query", "task_object": task})
        query_answered = isinstance(answer, list)
        if query_answered:
            query_boxes = [tuple(detection["box"]) for detection in answer]
            query_names = [detection["label"] for detection in answer]

    # Group detections across samples by label and vote with the median.
    groups: dict[str, dict[str, Any]] = {}
    for sample in samples:
        for detection in sample:
            group = groups.setdefault(
                _label_key(detection["label"]),
                {"label": detection["label"], "detections": []},
            )
            group["detections"].append(detection)

    # The surfaces besides the desk declared by hand (DeclaredSurfaces.json, read again when it changes):
    # an object on one is measured on it; on a surface nobody declared it is withheld as before.
    # With no calibrated desk in view the camera check passed against the gripper tip instead (alignment_source
    # "robot"): the old desk is no support there, so each object stands on the level surface measured around it,
    # the bin floor or a declared surface, and the reply says which check it was (no_desk). A camera the tip
    # verified skips the desk's tripwire; one it could not check ("assumed") still measures and plans, but is
    # NOT treated as verified: the tripwire runs on its desk readings and its scene is not "checked", so the
    # app keeps its [UNRELIABLE] tags (Aaran's decision, 2026-10-07 audit IH1).
    desk_verified = camera_check.get("alignment_source") != "robot"
    assumed = not desk_verified and camera_check.get("robot_check") == "assumed"
    surfaces = DepthGeometry.WorkspaceSurfaces(CameraDrift.DisplayDeskPlane(camera_model), camera_model.get("table_z"),
                                               Declared=[DeclaredSurfaces.SurfaceToDisplay(Surface) for Surface in DeclaredSurfaces.Load()],
                                               Automatic=use_depth and camera_check["ok"], Desk=desk_verified,
                                               DeskDiagnostics=desk_verified or assumed)
    exclude = DepthGeometry.ArmExclusion(arm["joints"]) if arm and arm.get("ok") and arm.get("joints") else None
    # The arm's own link boxes where it stood for the capture: the measured scene leaves them out (SceneGeometry.Build).
    arm_boxes = arm.get("link_boxes") if arm and arm.get("ok") else None
    objects: list[dict[str, Any]] = []
    withheld: list[dict[str, Any]] = []
    diagnostics: list[dict[str, Any]] = []
    hints: dict[str, float] = {}  # label -> undeclared_support_z of its withheld measurement

    def measure(label: str, u: float, v: float, box, note: dict[str, Any], only=None,
                trace: dict[str, Any] | None = None) -> tuple[dict[str, Any] | None, str | None]:
        """(entry, None); (None, why it is withheld); or (None, None) when nothing places it.
        only and trace go to MeasureBox (its Only and Trace), and only when given."""
        mapped = calibration.pixel_to_plane(u, v, mirrored)
        if use_depth:
            extra = {**({"Only": only} if only is not None else {}), **({"Trace": trace} if trace is not None else {})}
            measured = DepthGeometry.MeasureBox(depth["depth_mm"], box, depth["intrinsics"], camera_model,
                                                float(camera_model["table_z"]), bool(depth["mirrored"]),
                                                Surfaces=surfaces, Exclude=exclude, **extra)
            diagnostics.append({"label": label, "box": [round(value, 4) for value in box], **note, **measured})
            # Depth exists, so a failed measurement is withheld: the table-plane guess put
            # binders standing beside the cart onto the desk.
            if measured.get("robot_arm"):
                reason = "the robot's own arm fills the box"
            elif not measured.get("ok"):
                reason = f"not measurable: {measured.get('error', 'no depth')}"
            else:
                reason = _ImplausibleReason(measured)
            if reason:
                diagnostics[-1]["withheld"] = reason
                # A level surface nobody declared that seems to hold it (DepthGeometry: a hint, never a
                # support): the web app names it when the task's own object is withheld.
                if measured.get("undeclared_support_z") is not None:
                    hints[label] = measured["undeclared_support_z"]
                return None, reason
            entry = {
                "label": label,
                "x": measured["x"],
                "y": measured["y"],
                "z": measured["z"],
                "top_z": measured["top_z"],
                "height_cm": measured["height_cm"],
                "size_cm": measured["size_cm"],
                "distance_cm": measured["distance_cm"],
                "method": "depth",
                "support_measured": bool(measured["support_measured"]),
                "support_z_measured": measured["support_z_measured"],
                "valid_fraction": measured["valid_fraction"],
                "support_branch": measured["support_branch"],
            }
            # On a declared surface (DeclaredSurfaces.json): which, and how far its heights are from the
            # band the calibration tips covered (the tape was held 9-17 cm above the desk; desk objects
            # 10 cm under it measure well): never checked there, so the reading says so.
            if measured["support_branch"] == "surface":
                entry.update(support_surface=measured.get("support_surface"), support_kind=measured.get("support_kind"),
                             outside_calibration_z_cm=_OutsideCalibrationZ(camera_model, measured))
            Geometry = measured.get("support_geometry") or DepthGeometry.LocalSupportGeometry(
                depth["depth_mm"], box, depth["intrinsics"], camera_model, bool(depth["mirrored"]), measured, exclude)
            if Geometry is not None:
                entry["support_geometry"] = Geometry
            if measured.get("bounds_cm"):
                entry["bounds_cm"] = measured["bounds_cm"]
            # The share of the box's depth the arm's mask took (only with the arm masked). The mask is
            # 3 cm around the link centre lines, so the open moving jaw stays in a box it is in: the web
            # app takes a measurement whose top is the gripper's underside for the gripper itself
            # (lib/ArmMadeObjects.ts) only when the arm is a large part of its box. The 16:41:36 "phone"
            # of 2026-10-03 was 0.356; a real can in front of the raised gripper read up to 0.407.
            if measured.get("arm_fraction") is not None:
                entry["arm_fraction"] = measured["arm_fraction"]
        elif mapped.get("ok"):
            entry = {
                "label": label,
                "x": round(mapped["x"], 2),
                "y": round(mapped["y"], 2),
                "z": round(mapped["z"], 2),
                "method": "table-plane",
            }
        else:
            return None, None
        # A depth measurement is judged by where the calibration tips were (display frame XY):
        # the table-plane pixel test compares the desk contact with the pixels of the elevated
        # tape and flagged every object of 2026-09-29. Fits without a tip region keep that test.
        region = calibration.depth_point_extrapolated(camera_model, entry["x"], entry["y"]) if use_depth else None
        if region is not None:
            entry.update(region)
        else:
            entry["extrapolated"] = bool(mapped.get("extrapolated", False)) if mapped.get("ok") else False
        return entry, None

    prepared = []
    robot_boxes = []  # where the samples boxed the robot itself
    for group in groups.values():
        reason = _RobotPartReason(group["label"], task["label"] if task else None)
        if reason:
            withheld.append({"label": group["label"], "reason": reason})
            if reason == ROBOT_PARTS_REASON:
                robot_boxes += [d["box"] for d in group["detections"]]
            continue
        detections = group["detections"]
        u = _median([d["u"] for d in detections])
        v = _median([d["v"] for d in detections])
        spread = max(
            ((d["u"] - u) ** 2 + (d["v"] - v) ** 2) ** 0.5 for d in detections
        )
        # Representative box: the sampled box whose contact sits closest to the
        # median contact (a synthetic median box could mix mismatched corners).
        best = min(
            detections,
            key=lambda d: ((d["u"] - u) ** 2 + (d["v"] - v) ** 2) ** 0.5,
        )
        prepared.append((group, u, v, spread, best))

    # Which groups to zoom-refine (one sequential model call each; 94% of the refine time on
    # 2026-09-28/29 went to objects the task never used). With a task object: its groups
    # (named by the task's noun, or under the query's box), plus groups whose samples
    # disagree, which only a refine settles. The rest keep their first-pass box and
    # refined: false. A query that answered [] with no label naming the object means
    # it is not in view under that name, so only the stand-ins below are. Without a task object, or when
    # the query failed and no label names it, every group is refined as before.
    head = _TaskHead(task["label"]) if task else None
    task_groups = {id(group) for group, _, _, _, best in prepared
                   if _NamesTask(group["label"], head) or any(_LandsOn(query, best["box"]) for query in query_boxes)}
    refine_all = not task or not (task_groups or query_answered)
    # No group named or under the query's box: the groups that may be the task object under another name
    # (_StandIn) are refined too, so the one the web app binds is not left a coarse box.
    stand_ins = ({id(group) for group, _, _, _, _ in prepared if _StandIn(group["label"], task["label"])}
                 if task and not task_groups and not refine_all else set())
    landed: set[int] = set()
    refine_skipped: list[str] = []
    for group, u, v, spread, best in prepared:
        detections = group["detections"]
        why = ("all" if refine_all else "task" if id(group) in task_groups else "stand-in" if id(group) in stand_ins
               else "unstable" if spread > SAMPLE_UNSTABLE_PX else None)
        # Zoom-refine: re-detect in a crop around the coarse box. Full-frame
        # boxes carry systematic shifts in busy scenes; cropped ones do not.
        refined = None
        if why:
            box_px = (
                best["box"][0] * width, best["box"][1] * height,
                best["box"][2] * width, best["box"][3] * height,
            )
            refined = _refine_detection(image, group["label"], box_px, picked_model)
        else:
            refine_skipped.append(group["label"])
        box = best["box"]
        if refined is not None:
            u, v, box = refined

        entry, reason = measure(group["label"], u, v, box, {"refine": why or "skipped"})
        if reason:
            withheld.append({"label": group["label"], "reason": reason, **_Hint(hints, group["label"])})
            continue
        if entry is None:
            continue
        hits = {index for index, query in enumerate(query_boxes) if _LandsOn(query, best["box"]) or _LandsOn(query, box)}
        landed |= hits
        entry.update({
            "unstable": bool(spread > SAMPLE_UNSTABLE_PX and refined is None),
            "refined": refined is not None,
            "samples": len(detections),
            "box": [round(value, 4) for value in box],
            "task_match": bool(hits),
            "task_query_only": False,
        })
        objects.append(entry)

    # Query boxes on no measured object: the task object the samples missed or could not
    # measure (at 23:09:57 UTC only 1 of 3 samples named the USB drive). Each is measured
    # like a group, under the task's label; the same spot answered twice is one object.
    orphans: list[int] = []
    for index, query in enumerate(query_boxes):
        if index not in landed and not any(_LandsOn(query, query_boxes[other]) for other in orphans):
            orphans.append(index)
    for index in orphans:
        query, label = query_boxes[index], task["label"]
        # The robot and bin check reads the label, not the phrase: "eraser in the bin" is an eraser.
        reason = _RobotPartReason(label, label)
        # Without the arm masked out of depth (no pose, or no depth at all), nothing tells the
        # arm from an object on it: a query box on a spot the samples called the robot is withheld.
        if not reason and (exclude is None or not use_depth) and any(_LandsOn(query, part) for part in robot_boxes):
            reason = "on the robot's own arm, which this capture cannot mask out"
        refined = None
        if not reason:
            # Refined under the query's own name: "task object" names nothing in a crop.
            refined = _refine_detection(image, query_names[index] or label, (query[0] * width, query[1] * height,
                                                                              query[2] * width, query[3] * height), picked_model)
            u, v, box = refined if refined is not None else ((query[0] + query[2]) / 2 * width, query[3] * height, query)
            entry, reason = measure(label, u, v, box, {"refine": "task", "task_query_only": True})
        if reason:
            withheld.append({"label": label, "reason": reason, "task_query_only": True, **_Hint(hints, label)})
            continue
        if entry is None:
            continue
        entry.update({
            "unstable": False,
            "refined": refined is not None,
            "samples": 0,
            "box": [round(value, 4) for value in box],
            "task_match": True,
            "task_query_only": True,
        })
        objects.append(entry)
    review = {"task_query_boxes": [[round(value, 4) for value in query] for query in query_boxes],
              "refine_skipped": refine_skipped} if task else {}
    calls = len(samples) + (1 if task and mode == "vlm" else 0)
    model_ms = None
    if mode == "single_call":
        # One model call names the objects; depth measures, trims and checks every box (LOCATE_MODES).
        called = time.perf_counter()
        single = _detect_single(image_data_url, task, picked_model, width, height)
        calls, model_ms = 1, round((time.perf_counter() - called) * 1000, 1)
        if isinstance(single, dict):
            failed = {"ok": False, **single}
            _Record({**failed, "camera_check": camera_check, "model": picked_model, "locate_mode": mode}, labels, depth, [], task)
            return failed
        rows, task_absent = single
        native_mirrored = bool(depth["mirrored"])
        task_box = next((tuple(row["box"]) for row in rows if row["task_row"]), None)
        task_missing: str | None = "not in the reply" if task and task_box is None else None
        listed: list[tuple[float, float, float, float]] = []
        for row in rows:
            label = row["label"]
            reason = _RobotPartReason(label, task["label"] if task else None)
            if reason:
                withheld.append({"label": label, "reason": reason})
                if reason == ROBOT_PARTS_REASON:
                    robot_boxes.append(tuple(row["box"]))
                if row["task_row"]:
                    task_missing = None  # the task names the robot's own part: nothing to look for again
                continue
            # A later row on an earlier row's spot is that object listed again (the task's row is first).
            if any(_Overlap(earlier, row["box"]) > 0.5 for earlier in listed):
                continue
            listed.append(tuple(row["box"]))
            entry, reason, box, whole = _MeasureRow(measure, row, label, width, height, native_mirrored, diagnostics, hints,
                                                    {"refine": "depth", "task_row": row["task_row"]})
            if row["task_row"] and entry is None and _NoDepthObject(reason):
                task_missing = f"its box holds no object in depth ({reason})"
            if reason:
                withheld.append({"label": label, "reason": reason, **_Hint(hints, label)})
                continue
            if entry is None:
                continue
            entry.update({"unstable": False, "refined": bool(whole and row["task_row"]), "samples": 1,
                          "box": [round(value, 4) for value in box], "task_match": row["task_row"], "task_query_only": False,
                          "depth_refined": whole, "outline": bool(diagnostics and diagnostics[-1].get("outline"))})
            objects.append(entry)
        # The single call missed the task's object (no row, a null row, or a box with nothing in it): one strict query.
        retried = None
        if task and task_missing:
            retried = task_missing
            called = time.perf_counter()
            answer = _detect_once(image_data_url, None, picked_model, width, height, temperature=0,
                                  prompt=_task_query_prompt(task["phrase"]), record={"kind": "task_query", "task_object": task,
                                                                                    "why": task_missing})
            calls += 1
            model_ms = round(model_ms + (time.perf_counter() - called) * 1000, 1)
            for detection in (answer[:1] if isinstance(answer, list) else []):
                query = tuple(detection["box"])
                query_boxes.append(query)
                hit = next((item for item in objects if _LandsOn(query, item["box"])), None)
                if hit is not None:
                    hit["task_match"] = True
                    continue
                label = task["label"]
                reason = _RobotPartReason(label, label)
                if not reason and any(_LandsOn(query, part) for part in robot_boxes) and exclude is None:
                    reason = "on the robot's own arm, which this capture cannot mask out"
                entry = None
                if not reason:
                    row = {**detection, "outline": None, "task_row": True}
                    entry, reason, box, whole = _MeasureRow(measure, row, label, width, height, native_mirrored, diagnostics, hints,
                                                            {"refine": "depth", "task_query_only": True})
                if reason:
                    withheld.append({"label": label, "reason": reason, "task_query_only": True, **_Hint(hints, label)})
                elif entry is not None:
                    entry.update({"unstable": False, "refined": whole, "samples": 0, "box": [round(value, 4) for value in box],
                                  "task_match": True, "task_query_only": True, "depth_refined": whole, "outline": False})
                    objects.append(entry)
        review ={"task_query_boxes": [[round(value, 4) for value in query] for query in query_boxes],
                  "task_absent": task_absent, "task_retry": retried} if task else {}

    # Supports that disagree with the calibrated desk: the camera moved after all. On a desk-verified capture, and on
    # an "assumed" one, whose camera nothing checked (audit IH1); a camera the gripper tip verified skips it.
    tripped = _SceneTripwire(diagnostics, camera_check) if desk_verified or assumed else None
    if tripped:
        camera_check = {**camera_check, "ok": False, "error_code": "camera_moved", "reason": f"scene tripwire: {tripped[0]}",
                        "error": tripped[1]}
        blocked = {"ok": False, "error_code": "camera_moved", "error": camera_check["error"], "camera_check": camera_check,
                   "model": picked_model}
        _Record({**blocked, "objects": objects, "withheld": withheld, **review}, labels, depth, diagnostics, task)
        return blocked

    result = {
        "ok": True,
        "objects": objects,
        "withheld": withheld,
        "camera_check": camera_check,
        "model": picked_model,
        "image_size": [width, height],
        "samples_run": len(samples),
        "rms_cm": calibration_status["rms_cm"],
        "depth": use_depth,
        # Which path measured (LOCATE_MODES), its model calls, and where the time went (ms; model only for single_call).
        "locate_mode": mode,
        "model_calls": calls,
        "timing_ms": {"before_model": checked_ms, "model": model_ms,
                      "total": round((time.perf_counter() - Started) * 1000, 1)},
    }
    if not desk_verified:
        # Measured without the desk: how the camera was checked ("verified" by the gripper tip, or "assumed").
        result["no_desk"] = {"robot_check": camera_check.get("robot_check"), "supports": "measured"}
    if use_depth and task and Carrying:
        try:
            if Previous is None:
                raise ValueError(CarryError or "the previous carrying scene is unavailable")
            Scene = SceneGeometry.Build(depth["depth_mm"], depth["intrinsics"], camera_model, {},
                                        Previous["support"], exclude, depth.get("seq"),
                                        TaskEnvelope=Previous["task_object"], RemoveTarget=False, ArmBoxes=arm_boxes)
            Scene["carrying"] = True
            result["scene"] = SceneSafety.Register(Scene, camera_model, capture_time=depth.get("time"), generation=Generation)
        except ValueError as Exc:
            result["scene_error"] = str(Exc)
    elif use_depth and task:
        Matches = [Object for Object in objects if Object.get("task_match")]
        if not Matches:
            Matches = [Object for Object in objects if _label_key(Object["label"]) == _label_key(task["label"])]
        Matches = _OneTaskObject(Matches)
        if len(Matches) == 1 and Matches[0].get("support_geometry"):
            Target = Matches[0]
            try:
                Scene = SceneGeometry.Build(depth["depth_mm"], depth["intrinsics"], camera_model, Target,
                                            Target["support_geometry"], exclude, depth.get("seq"), ArmBoxes=arm_boxes)
                if SceneSafety is None:
                    raise ValueError("this bridge has no measured-scene safety registry")
                Scene = SceneSafety.Register(Scene, camera_model, capture_time=depth.get("time"), generation=Generation)
                Target["support_geometry"] = Scene["support"]
                Target["scene_id"] = Scene["id"]
                Target["localization_frame"] = "calibrated_depth_3d"
                # Not with an "assumed" camera (audit IH1): the app then keeps the measurements' [UNRELIABLE] tags.
                Target["scene_checked"] = not assumed
                result["scene"] = Scene
            except ValueError as Exc:
                result["scene_error"] = str(Exc)
        else:
            result["scene_error"] = "a unique task object and its supporting surface were not measured"
    _Record({**result, **review}, labels, depth if use_depth else None, diagnostics, task)
    return result
