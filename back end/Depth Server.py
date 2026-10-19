"""UniDepth metric-depth server for the SO-101 AI arm vision loop.

Runs on the GPU PC (192.168.1.20) alongside Ollama and the arm bridge. It
turns a single webcam frame into metric depth so the vision pipeline can:

  POST /depth         Run UniDepth on an image. Returns a colorized depth image
                      (to send to qwen3-vl) plus depth stats, the estimated
                      camera intrinsics, and a frame_id. Optionally also returns
                      metric camera-frame XYZ for pixel points given in the call.

  POST /depth/lookup  Given a frame_id from a previous /depth call and one or
                      more pixel points, return metric depth (cm) and camera-
                      frame XYZ (cm) at those points. This is the precise path:
                      the VLM looks at the depth image, names a target pixel,
                      and we resolve it to real metric coordinates here.

  GET  /health        Liveness + which model/device is loaded.

COORDINATE NOTE (important)
  The XYZ this server returns is in the CAMERA frame, OpenCV convention:
      +X right, +Y down, +Z forward (away from the camera), in centimeters.
  This is NOT the arm's DISPLAY frame (+X left, +Y farther, +Z up). v1 has no
  hand-eye calibration: the VLM estimates DISPLAY-space XYZ qualitatively from
  the image + depth, and the human approves every move. The metric camera XYZ
  is provided as a grounding aid (e.g. "that object is ~32 cm away").

SETUP (fresh GPU box)
  1. Install PyTorch matching your CUDA, e.g.:
       pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121
  2. Install UniDepth from source:
       git clone https://github.com/lpiccinelli-eth/UniDepth
       cd UniDepth && pip install -e .
  3. Install this server's light deps:
       pip install -r "back end/requirements-depth.txt"
  4. Run it (bind 0.0.0.0 so the laptop can reach it over the LAN):
       python "Depth Server.py"
     NOTE: this file's name has a space, so the uvicorn module form
     (uvicorn module:app) and --reload do NOT work. Run it directly as above;
     the __main__ block reads DEPTH_HOST/DEPTH_PORT (default 0.0.0.0:8001).

CONFIG (environment variables)
  DEPTH_MODEL_ID   HuggingFace id. Default lpiccinelli/unidepth-v2-vitl14.
                   Use lpiccinelli/unidepth-v2-vits14 for a smaller/faster model.
  DEPTH_DEVICE     "cuda" or "cpu". Default: cuda if available, else cpu.
  DEPTH_HOST       Bind host. Default 0.0.0.0.
  DEPTH_PORT       Bind port. Default 8001.
  DEPTH_MAX_SIDE   Cap the longest image side before inference (speed). Default
                   924. Set 0 to disable resizing.
  DEPTH_ALLOWED_ORIGINS  Comma-separated CORS origins. Default covers the Vite
                   dev server on 127.0.0.1/localhost:5180-5181.
"""

from __future__ import annotations

import base64
import binascii
import io
import os
import threading
import uuid
from collections import OrderedDict
from typing import Any

import numpy as np
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from PIL import Image

# Torch / UniDepth are heavy and only needed at inference time. Import lazily in
# load_model() so the module can be imported (and syntax-checked) without them,
# and so a load failure is reported cleanly via /health instead of crashing boot.


# ----------------------------------------------------------------------------
# Config
# ----------------------------------------------------------------------------
MODEL_ID = os.environ.get("DEPTH_MODEL_ID", "lpiccinelli/unidepth-v2-vitl14")
DEVICE_ENV = os.environ.get("DEPTH_DEVICE", "").strip().lower()
HOST = os.environ.get("DEPTH_HOST", "0.0.0.0")
PORT = int(os.environ.get("DEPTH_PORT", "8001"))
MAX_SIDE = int(os.environ.get("DEPTH_MAX_SIDE", "924"))
FRAME_CACHE_SIZE = 8

_default_origins = [
    "http://127.0.0.1:5180",
    "http://localhost:5180",
    "http://127.0.0.1:5181",
    "http://localhost:5181",
]
ALLOWED_ORIGINS = [
    o.strip()
    for o in os.environ.get("DEPTH_ALLOWED_ORIGINS", ",".join(_default_origins)).split(",")
    if o.strip()
]


# ----------------------------------------------------------------------------
# Globals (model + a small cache of recent depth frames for /depth/lookup)
# ----------------------------------------------------------------------------
_model: Any = None
_torch: Any = None
_device: str = "cpu"
_model_error: str | None = None
_infer_lock = threading.Lock()  # serialize GPU inference

_frames: "OrderedDict[str, dict[str, Any]]" = OrderedDict()
_frames_lock = threading.Lock()


def load_model() -> None:
    """Load UniDepthV2 once. Records any failure in _model_error."""
    global _model, _torch, _device, _model_error
    try:
        import torch
        from unidepth.models import UniDepthV2

        _torch = torch
        if DEVICE_ENV in ("cuda", "cpu"):
            _device = DEVICE_ENV
        else:
            _device = "cuda" if torch.cuda.is_available() else "cpu"

        model = UniDepthV2.from_pretrained(MODEL_ID)
        model = model.to(_device).eval()
        _model = model
        _model_error = None
    except Exception as exc:  # pragma: no cover - depends on the GPU env
        _model = None
        _model_error = f"{type(exc).__name__}: {exc}"


# ----------------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------------
def _decode_image(image_field: str) -> Image.Image:
    """Accept a data URL ('data:image/png;base64,...') or bare base64 string."""
    if not isinstance(image_field, str) or not image_field.strip():
        raise HTTPException(status_code=400, detail="Missing 'image' (base64 or data URL).")
    raw_b64 = image_field.split(",", 1)[1] if image_field.startswith("data:") else image_field
    try:
        raw = base64.b64decode(raw_b64, validate=False)
    except (binascii.Error, ValueError) as exc:
        raise HTTPException(status_code=400, detail=f"Bad base64 image: {exc}") from exc
    try:
        return Image.open(io.BytesIO(raw)).convert("RGB")
    except Exception as exc:
        raise HTTPException(status_code=400, detail=f"Unreadable image: {exc}") from exc


def _resize_for_inference(img: Image.Image) -> Image.Image:
    if MAX_SIDE <= 0:
        return img
    w, h = img.size
    longest = max(w, h)
    if longest <= MAX_SIDE:
        return img
    scale = MAX_SIDE / float(longest)
    return img.resize((max(1, round(w * scale)), max(1, round(h * scale))), Image.BILINEAR)


def _intrinsics_tensor(intrinsics: Any):
    """Optional caller-supplied 3x3 intrinsics -> torch tensor, else None."""
    if intrinsics is None:
        return None
    arr = np.asarray(intrinsics, dtype=np.float32)
    if arr.shape != (3, 3):
        raise HTTPException(status_code=400, detail="intrinsics must be a 3x3 matrix.")
    return _torch.from_numpy(arr).to(_device)


def _run_depth(img: Image.Image, intrinsics: Any) -> tuple[np.ndarray, np.ndarray]:
    """Run UniDepth. Returns (depth_cm HxW float32, intrinsics 3x3 float32)."""
    if _model is None:
        raise HTTPException(status_code=503, detail=f"Depth model not loaded: {_model_error}")

    rgb_np = np.asarray(img, dtype=np.uint8)  # (H, W, 3)
    rgb = _torch.from_numpy(rgb_np).permute(2, 0, 1)  # (3, H, W) uint8
    intr_t = _intrinsics_tensor(intrinsics)

    with _infer_lock:
        with _torch.inference_mode():
            rgb_dev = rgb.to(_device)
            pred = _model.infer(rgb_dev, intr_t) if intr_t is not None else _model.infer(rgb_dev)

    depth_m = pred["depth"].squeeze().float().cpu().numpy()  # (H, W) meters
    intr_out = pred["intrinsics"].squeeze().float().cpu().numpy()  # (3, 3)
    depth_cm = (depth_m * 100.0).astype(np.float32)
    return depth_cm, intr_out.astype(np.float32)


def _colorize_depth(depth_cm: np.ndarray) -> tuple[str, float, float]:
    """Colorize depth with a turbo colormap. Returns (data_url, lo_cm, hi_cm).

    Near (small depth) maps to the low end of the colormap, far to the high end.
    Robust 2nd/98th-percentile clipping keeps contrast sane with outliers.
    """
    d = depth_cm.astype(np.float32)
    valid = np.isfinite(d) & (d > 0)
    if valid.any():
        lo = float(np.percentile(d[valid], 2))
        hi = float(np.percentile(d[valid], 98))
    else:
        lo, hi = 0.0, 1.0
    if hi - lo < 1e-6:
        hi = lo + 1e-6

    norm = np.clip((d - lo) / (hi - lo), 0.0, 1.0)
    norm[~valid] = 0.0

    cmap = None
    try:
        import matplotlib
        cmap = matplotlib.colormaps["turbo"]
    except Exception:
        try:
            from matplotlib import cm
            cmap = cm.get_cmap("turbo")
        except Exception:
            cmap = None

    if cmap is not None:
        rgb = (np.asarray(cmap(norm))[..., :3] * 255.0).astype(np.uint8)
    else:
        g = (norm * 255.0).astype(np.uint8)
        rgb = np.stack([g, g, g], axis=-1)

    buf = io.BytesIO()
    Image.fromarray(rgb, mode="RGB").save(buf, format="PNG")
    data_url = "data:image/png;base64," + base64.b64encode(buf.getvalue()).decode("ascii")
    return data_url, lo, hi


def _backproject(u: float, v: float, z_cm: float, intr: np.ndarray) -> list[float]:
    """Pixel (u,v) + metric depth -> camera-frame XYZ in cm (OpenCV convention)."""
    fx, fy = float(intr[0, 0]), float(intr[1, 1])
    cx, cy = float(intr[0, 2]), float(intr[1, 2])
    x = (u - cx) / fx * z_cm if fx else 0.0
    y = (v - cy) / fy * z_cm if fy else 0.0
    return [round(x, 2), round(y, 2), round(z_cm, 2)]


def _to_pixel(coord: Any, size: int, normalized: bool) -> int:
    val = float(coord)
    if normalized:
        val *= (size - 1)
    return int(max(0, min(size - 1, round(val))))


def _resolve_points(
    points: Any,
    depth_cm: np.ndarray,
    intr: np.ndarray,
    normalized: bool,
) -> list[dict[str, Any]]:
    if not points:
        return []
    if not isinstance(points, list):
        raise HTTPException(status_code=400, detail="'points' must be a list of [x, y].")
    h, w = depth_cm.shape
    out: list[dict[str, Any]] = []
    for pt in points:
        if not (isinstance(pt, (list, tuple)) and len(pt) == 2):
            raise HTTPException(status_code=400, detail="Each point must be [x, y].")
        u = _to_pixel(pt[0], w, normalized)
        v = _to_pixel(pt[1], h, normalized)
        z = float(depth_cm[v, u])
        out.append({
            "x_px": u,
            "y_px": v,
            "depth_cm": round(z, 2),
            "camera_xyz_cm": _backproject(u, v, z, intr),
        })
    return out


def _cache_frame(frame_id: str, depth_cm: np.ndarray, intr: np.ndarray) -> None:
    with _frames_lock:
        _frames[frame_id] = {"depth_cm": depth_cm, "intrinsics": intr,
                             "h": int(depth_cm.shape[0]), "w": int(depth_cm.shape[1])}
        _frames.move_to_end(frame_id)
        while len(_frames) > FRAME_CACHE_SIZE:
            _frames.popitem(last=False)


def _stats(depth_cm: np.ndarray) -> dict[str, float]:
    valid = depth_cm[np.isfinite(depth_cm) & (depth_cm > 0)]
    if valid.size == 0:
        return {"min_cm": 0.0, "max_cm": 0.0, "median_cm": 0.0}
    return {
        "min_cm": round(float(valid.min()), 2),
        "max_cm": round(float(valid.max()), 2),
        "median_cm": round(float(np.median(valid)), 2),
    }


# ----------------------------------------------------------------------------
# App
# ----------------------------------------------------------------------------
app = FastAPI(title="SO-101 UniDepth Server")
app.add_middleware(
    CORSMiddleware,
    allow_origins=ALLOWED_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.on_event("startup")
def _startup() -> None:
    load_model()


@app.get("/health")
def health() -> dict[str, Any]:
    return {
        "service": "so101-unidepth-server",
        "ok": _model is not None,
        "model_id": MODEL_ID,
        "device": _device,
        "loaded": _model is not None,
        "error": _model_error,
    }


@app.post("/depth")
def depth(payload: dict[str, Any]) -> dict[str, Any]:
    img = _decode_image(payload.get("image", ""))
    img = _resize_for_inference(img)
    depth_cm, intr = _run_depth(img, payload.get("intrinsics"))

    frame_id = uuid.uuid4().hex
    _cache_frame(frame_id, depth_cm, intr)

    data_url, lo, hi = _colorize_depth(depth_cm)
    h, w = depth_cm.shape
    normalized = bool(payload.get("normalized", True))
    points = _resolve_points(payload.get("points"), depth_cm, intr, normalized)

    return {
        "ok": True,
        "frame_id": frame_id,
        "width": w,
        "height": h,
        "depth_image": data_url,
        "colormap": {"name": "turbo", "near_cm": round(lo, 2), "far_cm": round(hi, 2),
                     "note": "near depth = blue end, far depth = red end"},
        "depth_stats_cm": _stats(depth_cm),
        "intrinsics": intr.tolist(),
        "camera_frame": "opencv: +X right, +Y down, +Z forward; centimeters",
        "points": points,
    }


@app.post("/depth/lookup")
def depth_lookup(payload: dict[str, Any]) -> dict[str, Any]:
    frame_id = str(payload.get("frame_id", ""))
    with _frames_lock:
        frame = _frames.get(frame_id)
        if frame is not None:
            _frames.move_to_end(frame_id)
    if frame is None:
        raise HTTPException(status_code=404, detail="Unknown or expired frame_id. Call /depth again.")

    normalized = bool(payload.get("normalized", True))
    points = _resolve_points(payload.get("points"), frame["depth_cm"], frame["intrinsics"], normalized)
    return {
        "ok": True,
        "frame_id": frame_id,
        "width": frame["w"],
        "height": frame["h"],
        "camera_frame": "opencv: +X right, +Y down, +Z forward; centimeters",
        "points": points,
    }


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host=HOST, port=PORT)
