"""Offline SO-101 FK in the official URDF frame, SI units and radians.

Input angles are URDF joint angles, NOT raw servo steps or this app's GUI
angles. Upstream joint identifiers and library APIs retain their required case.
"""

import json as Json
import math as Math
from pathlib import Path
import sys as Sys
import xml.etree.ElementTree as Xml

import numpy as Np

OutputRoot = Path.home() / "Downloads" / "SO101Cad"
DefaultUrdf = OutputRoot / "Original" / "Simulation" / "SO101" / "so101_new_calib.urdf"
JointOrder = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")


def Vector(Text):
    return Np.array([float(Value) for Value in Text.split()], dtype=float)


def Rotation(Axis, Angle):
    Axis = Np.asarray(Axis, dtype=float)
    Axis = Axis / Np.linalg.norm(Axis)
    X, Y, Z = Axis
    Cross = Np.array([[0, -Z, Y], [Z, 0, -X], [-Y, X, 0]])
    return Np.eye(3) + Math.sin(Angle) * Cross + (1 - Math.cos(Angle)) * (Cross @ Cross)


def OriginTransform(Element):
    Result = Np.eye(4)
    if Element is None:
        return Result
    Result[:3, 3] = Vector(Element.get("xyz", "0 0 0"))
    Roll, Pitch, Yaw = Vector(Element.get("rpy", "0 0 0"))
    Result[:3, :3] = Rotation((0, 0, 1), Yaw) @ Rotation((0, 1, 0), Pitch) @ Rotation((1, 0, 0), Roll)
    return Result


def LoadModel(UrdfPath=DefaultUrdf):
    Root = Xml.parse(UrdfPath).getroot()
    Joints = []
    for Element in Root.findall("joint"):
        Joints.append({
            "Name": Element.get("name"),
            "Type": Element.get("type"),
            "Parent": Element.find("parent").get("link"),
            "Child": Element.find("child").get("link"),
            "Origin": OriginTransform(Element.find("origin")),
            "Axis": Vector(Element.find("axis").get("xyz", "0 0 1")),
            "Limits": dict(Element.find("limit").attrib) if Element.find("limit") is not None else {},
        })
    Children = {Joint["Child"] for Joint in Joints}
    Roots = {Element.get("name") for Element in Root.findall("link")} - Children
    if len(Roots) != 1:
        raise ValueError("Expected a single rooted URDF tree")
    return {"Xml": Root, "Joints": Joints, "Root": Roots.pop(), "Path": Path(UrdfPath)}


def ForwardKinematics(Model, Angles=None):
    Angles = Angles or {}
    Unknown = set(Angles) - {Joint["Name"] for Joint in Model["Joints"] if Joint["Type"] != "fixed"}
    if Unknown:
        raise ValueError(f"Unknown moving joints: {sorted(Unknown)}")
    Links = {Model["Root"]: Np.eye(4)}
    JointFrames = {}
    Pending = list(Model["Joints"])
    while Pending:
        Ready = [Joint for Joint in Pending if Joint["Parent"] in Links]
        if not Ready:
            raise ValueError("Disconnected or cyclic URDF tree")
        for Joint in Ready:
            Frame = Links[Joint["Parent"]] @ Joint["Origin"]
            Motion = Np.eye(4)
            Angle = float(Angles.get(Joint["Name"], 0.0))
            if not Math.isfinite(Angle):
                raise ValueError("Joint angles must be finite")
            if Joint["Type"] in ("revolute", "continuous"):
                Motion[:3, :3] = Rotation(Joint["Axis"], Angle)
            elif Joint["Type"] != "fixed":
                raise ValueError(f"Unsupported joint type: {Joint['Type']}")
            JointFrames[Joint["Name"]] = Frame
            Links[Joint["Child"]] = Frame @ Motion
            Pending.remove(Joint)
    return Links, JointFrames


def ToolPosition(Model, Angles=None):
    return ForwardKinematics(Model, Angles)[0]["gripper_frame_link"][:3, 3]


if __name__ == "__main__":
    Angles = Json.loads(Sys.argv[1]) if len(Sys.argv) > 1 else {}
    Model = LoadModel()
    Links, Frames = ForwardKinematics(Model, Angles)
    print(Json.dumps({
        "AngleConvention": "Official new-calibration URDF radians; no raw-servo mapping applied",
        "PositionFrame": Model["Root"],
        "ToolFrame": "gripper_frame_link",
        "ToolPositionMm": (ToolPosition(Model, Angles) * 1000).tolist(),
        "JointOriginsMm": {Name: (Frames[Name][:3, 3] * 1000).tolist() for Name in JointOrder},
    }, indent=2))
