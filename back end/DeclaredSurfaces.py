"""Support surfaces besides the desk, declared by hand in DeclaredSurfaces.json.

Why
---
Every object used to stand on the desk or in the bin: the depth measurement
looked for no other support (DepthGeometry), and the web app held every AI target
above one desk floor. A platform, a box top or a shelf board within the arm's
reach is a surface the camera can measure, but so is a desk that moved or a camera
that drooped (the 19:00:57 desk read 4.6 cm low): the depth alone cannot tell them
apart. So a surface other than the desk counts only where it is declared here, with
its height and extent, and the camera's measurement must agree with it.

Format (display frame, cm; the web app's lib/DeclaredSurfaces.ts reads the same file
with the same rules, TestDeclaredSurfaces checks they agree)
------
{"Surfaces": [{"Name": "box platform", "Kind": "platform" | "shelf", "Enabled": true,
  "TopZCm": 12.7, "XRangeCm": [-25, -3], "YRangeCm": [30, 44],
  "BoardAboveZCm": null (a shelf: the underside of the board above), "Frame": "cart"}]}

Frame "cart" is the arm's display frame with the cart parked, the only one today: a
cart that drives will need its surfaces moved with it (SurfaceToDisplay). Entries
that break a rule are left out with a problem each (Load, Problems): an object on a
surface that is not declared is withheld, never measured on a guess.

Run:  python DeclaredSurfaces.py [file ...]   (prints each file's surfaces and problems as JSON)
"""

from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path
from typing import Any

BackendRoot = Path(__file__).resolve().parent
# SO101_DECLARED_SURFACES names another file: the test suites point it at none, so the surfaces
# declared for the real arm never change what their scenes measure.
SpecPath = Path(os.environ.get("SO101_DECLARED_SURFACES") or BackendRoot / "DeclaredSurfaces.json")
TableZCm = float(json.loads((BackendRoot / "WorkspaceFrame.json").read_text(encoding="utf-8"))["TableZCm"])

Kinds = ("platform", "shelf")
Frames = ("cart",)
# Where a surface may be (display Z): the arm's tip reaches about Z -13 to +43 with the cart parked.
MinTopZCm, MaxTopZCm = -15.0, 45.0
# A surface this near the desk height is the desk.
DeskSeparationCm = 1.5
# The smallest extent along X and Y, and the farthest a corner may be from the arm (sanity, not reach).
MinExtentCm = 2.0
MaxAbsXCm = 80.0
YLimitsCm = (-10.0, 120.0)
# A shelf's board above stands at least this far over the surface (the gripper is 5.5 cm tall at a level wrist).
MinClearHeightCm = 5.0
MaxNameChars = 40


def _Finite(Value) -> bool:
    return isinstance(Value, (int, float)) and not isinstance(Value, bool) and math.isfinite(Value)


def _Range(Value, Name: str, Low: float, High: float) -> tuple[list[float] | None, str | None]:
    if not isinstance(Value, list) or len(Value) != 2 or not all(_Finite(Item) for Item in Value):
        return None, f"{Name} must be [min, max] in cm"
    Start, End = float(Value[0]), float(Value[1])
    if End - Start < MinExtentCm:
        return None, f"{Name} {Start:g}..{End:g} must run from min to max over at least {MinExtentCm:g} cm"
    if Start < Low or End > High:
        return None, f"{Name} {Start:g}..{End:g} is outside {Low:g}..{High:g} cm"
    return [Start, End], None


def ValidateSurface(Entry: Any, Index: int) -> tuple[dict[str, Any] | None, str | None]:
    """One entry: the surface as the code uses it, or None and why it is left out (None, None: disabled)."""
    Where = f"surface {Index + 1}"
    if not isinstance(Entry, dict):
        return None, f"{Where} must be an object"
    Name = " ".join(str(Entry.get("Name") or "").split()) if isinstance(Entry.get("Name"), str) else ""
    if Name:
        Where = f"{Where} ({Name})"
    Enabled = Entry.get("Enabled", True)
    if not isinstance(Enabled, bool):
        return None, f"{Where}: Enabled must be true or false"
    if not Enabled:
        return None, None
    if not Name or len(Name) > MaxNameChars:
        return None, f"{Where}: Name must be 1 to {MaxNameChars} characters"
    Kind = Entry.get("Kind")
    if Kind not in Kinds:
        return None, f"{Where}: Kind must be one of {', '.join(Kinds)}"
    Frame = Entry.get("Frame", "cart")
    if Frame not in Frames:
        return None, f"{Where}: Frame must be one of {', '.join(Frames)} (a driving base will add others)"
    Top = Entry.get("TopZCm")
    if not _Finite(Top):
        return None, f"{Where}: TopZCm must be a number (cm, display Z)"
    Top = float(Top)
    if not MinTopZCm <= Top <= MaxTopZCm:
        return None, f"{Where}: TopZCm {Top:g} is outside {MinTopZCm:g}..{MaxTopZCm:g}, beyond the arm's reach"
    if abs(Top - TableZCm) < DeskSeparationCm:
        return None, f"{Where}: TopZCm {Top:g} is within {DeskSeparationCm:g} cm of the desk (Z {TableZCm:g}): that is the desk"
    XRange, Problem = _Range(Entry.get("XRangeCm"), "XRangeCm", -MaxAbsXCm, MaxAbsXCm)
    if Problem:
        return None, f"{Where}: {Problem}"
    YRange, Problem = _Range(Entry.get("YRangeCm"), "YRangeCm", *YLimitsCm)
    if Problem:
        return None, f"{Where}: {Problem}"
    Board = Entry.get("BoardAboveZCm")
    if Kind == "shelf":
        if not _Finite(Board):
            return None, f"{Where}: a shelf needs BoardAboveZCm, the underside of the board above (cm, display Z)"
        Board = float(Board)
        if Board < Top + MinClearHeightCm:
            return None, f"{Where}: BoardAboveZCm {Board:g} must be at least {MinClearHeightCm:g} cm above TopZCm {Top:g}"
    elif Board is not None:
        return None, f"{Where}: a platform has no board above (BoardAboveZCm null); declare a shelf for one"
    return {"Name": Name, "Kind": Kind, "TopZCm": Top, "XRangeCm": XRange, "YRangeCm": YRange,
            "BoardAboveZCm": Board if Kind == "shelf" else None, "Frame": Frame}, None


def Validate(Spec: Any) -> tuple[list[dict[str, Any]], list[str]]:
    """The usable surfaces of a parsed spec and the problems with the rest."""
    if not isinstance(Spec, dict):
        return [], ["the file must hold a JSON object with a Surfaces list"]
    Entries = Spec.get("Surfaces", [])
    if not isinstance(Entries, list):
        return [], ["Surfaces must be a list"]
    Surfaces: list[dict[str, Any]] = []
    Problems: list[str] = []
    for Index, Entry in enumerate(Entries):
        Surface, Problem = ValidateSurface(Entry, Index)
        if Problem:
            Problems.append(Problem)
        elif Surface is not None:
            if any(Other["Name"].lower() == Surface["Name"].lower() for Other in Surfaces):
                Problems.append(f"surface {Index + 1} ({Surface['Name']}): the name is used twice")
                continue
            Surfaces.append(Surface)
    return Surfaces, Problems


_Cache: dict[str, Any] = {"key": None, "surfaces": [], "problems": []}


def _Read(Path_: Path) -> tuple[list[dict[str, Any]], list[str]]:
    try:
        Spec = json.loads(Path_.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return [], []
    except (OSError, ValueError) as Exc:
        return [], [f"{Path_.name} is unreadable ({Exc}): no surface besides the desk is declared"]
    return Validate(Spec)


def Load(Path_: Path | None = None) -> list[dict[str, Any]]:
    """The declared surfaces, read again whenever the file changes (edit it with the bridge running)."""
    Path_ = Path(Path_) if Path_ is not None else SpecPath
    try:
        Stat = Path_.stat()
        Key = (str(Path_), Stat.st_mtime_ns, Stat.st_size)
    except OSError:
        Key = (str(Path_), None, None)
    if Key != _Cache["key"]:
        Surfaces, Problems = _Read(Path_)
        _Cache.update(key=Key, surfaces=Surfaces, problems=Problems)
        for Problem in Problems:
            print(f"[DeclaredSurfaces] {Problem}", file=sys.stderr)
    return [dict(Surface) for Surface in _Cache["surfaces"]]


def Problems(Path_: Path | None = None) -> list[str]:
    Load(Path_)
    return list(_Cache["problems"])


def SurfaceToDisplay(Surface: dict[str, Any]) -> dict[str, Any]:
    """A declared surface in the arm's display frame: Frame "cart" already is (a driving base would move it)."""
    if Surface.get("Frame", "cart") != "cart":
        raise ValueError(f"surface frame {Surface.get('Frame')!r} is not supported")
    return Surface


def Contains(Surface: dict[str, Any], X: float, Y: float, MarginCm: float = 0.0) -> bool:
    """Whether display X, Y lies over a surface's extent, grown by MarginCm."""
    (X0, X1), (Y0, Y1) = Surface["XRangeCm"], Surface["YRangeCm"]
    return X0 - MarginCm <= X <= X1 + MarginCm and Y0 - MarginCm <= Y <= Y1 + MarginCm


if __name__ == "__main__":
    Results = []
    for File in [Path(Name) for Name in sys.argv[1:]] or [SpecPath]:
        Found, Issues = _Read(File)
        Results.append({"Surfaces": Found, "Problems": Issues})
    print(json.dumps(Results))
