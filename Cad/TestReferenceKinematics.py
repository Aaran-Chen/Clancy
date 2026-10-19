"""Independent FK verification using MuJoCo's URDF importer and upstream MJCF."""

import json as Json
from pathlib import Path
import unittest as Unittest

import mujoco as Mujoco
import numpy as Np
from scipy.spatial.transform import Rotation as ScipyRotation

from ReferenceKinematics import DefaultUrdf, ForwardKinematics, JointOrder, LoadModel, OriginTransform, OutputRoot, ToolPosition

Model = LoadModel()
UrdfModel = Mujoco.MjModel.from_xml_path(str(DefaultUrdf))
MjcfModel = Mujoco.MjModel.from_xml_path(str(DefaultUrdf.with_suffix(".xml")))
Results = {}


def MujocoFrames(SimModel, Angles):
    Data = Mujoco.MjData(SimModel)
    for Name, Angle in Angles.items():
        JointId = Mujoco.mj_name2id(SimModel, Mujoco.mjtObj.mjOBJ_JOINT, Name)
        Data.qpos[SimModel.jnt_qposadr[JointId]] = Angle
    Mujoco.mj_forward(SimModel, Data)
    return Data


class ReferenceKinematicsTests(Unittest.TestCase):
    def TestIndependentRotationConvention(Self):
        for Element in Model["Xml"].iter("origin"):
            Rpy = [float(Value) for Value in Element.get("rpy", "0 0 0").split()]
            Expected = ScipyRotation.from_euler("xyz", Rpy).as_matrix()
            Np.testing.assert_allclose(OriginTransform(Element)[:3, :3], Expected, atol=1e-12)

    def TestForwardKinematicsAgainstMujoco(Self):
        Generator = Np.random.default_rng(101)
        Limits = {Joint["Name"]: (float(Joint["Limits"]["lower"]), float(Joint["Limits"]["upper"]))
                  for Joint in Model["Joints"] if Joint["Type"] != "fixed"}
        Poses = [{Name: 0.0 for Name in JointOrder}]
        Poses += [{Name: Limits[Name][Side] for Name in JointOrder} for Side in (0, 1)]
        Poses += [{Name: float(Generator.uniform(*Limits[Name])) for Name in JointOrder} for Index in range(100)]
        MaximumUrdfError = 0.0
        MaximumMjcfError = 0.0
        for Angles in Poses:
            Links, Frames = ForwardKinematics(Model, Angles)
            UrdfData = MujocoFrames(UrdfModel, Angles)
            MjcfData = MujocoFrames(MjcfModel, Angles)
            for Name in JointOrder:
                UrdfId = Mujoco.mj_name2id(UrdfModel, Mujoco.mjtObj.mjOBJ_JOINT, Name)
                MjcfId = Mujoco.mj_name2id(MjcfModel, Mujoco.mjtObj.mjOBJ_JOINT, Name)
                Position = Frames[Name][:3, 3]
                MaximumUrdfError = max(MaximumUrdfError, float(Np.linalg.norm(UrdfData.xanchor[UrdfId] - Position)))
                MaximumMjcfError = max(MaximumMjcfError, float(Np.linalg.norm(MjcfData.xanchor[MjcfId] - Position)))
                Np.testing.assert_allclose(UrdfData.xanchor[UrdfId], Position, atol=1e-9)
                Np.testing.assert_allclose(MjcfData.xanchor[MjcfId], Position, atol=1e-5)
                BodyId = int(UrdfModel.jnt_bodyid[UrdfId])
                Child = next(Joint["Child"] for Joint in Model["Joints"] if Joint["Name"] == Name)
                Np.testing.assert_allclose(UrdfData.xmat[BodyId].reshape(3, 3), Links[Child][:3, :3], atol=1e-9)
            GripperId = Mujoco.mj_name2id(UrdfModel, Mujoco.mjtObj.mjOBJ_JOINT, "wrist_roll")
            BodyId = int(UrdfModel.jnt_bodyid[GripperId])
            ToolOffset = next(Joint["Origin"][:3, 3] for Joint in Model["Joints"] if Joint["Name"] == "gripper_frame_joint")
            ExpectedTool = UrdfData.xpos[BodyId] + UrdfData.xmat[BodyId].reshape(3, 3) @ ToolOffset
            Np.testing.assert_allclose(ToolPosition(Model, Angles), ExpectedTool, atol=1e-9)
        Results.update({"PoseCount": len(Poses), "MaximumUrdfJointErrorMm": MaximumUrdfError * 1000,
                        "MaximumMjcfJointErrorMm": MaximumMjcfError * 1000})

    def TestJointLengthsAndToolBehavior(Self):
        Links, Frames = ForwardKinematics(Model)
        Pairs = [("shoulder_lift", "elbow_flex"), ("elbow_flex", "wrist_flex")]
        Expected = [MathValue for MathValue in (Np.hypot(112.57, 28.0), Np.hypot(134.9, 5.2))]
        for Index, (Start, End) in enumerate(Pairs):
            Actual = Np.linalg.norm(Frames[End][:3, 3] - Frames[Start][:3, 3]) * 1000
            Self.assertAlmostEqual(float(Actual), float(Expected[Index]), places=9)
        Closed = ToolPosition(Model)
        Opened = ToolPosition(Model, {"gripper": 1.0})
        Np.testing.assert_allclose(Closed, Opened, atol=1e-12)
        Rolled = ToolPosition(Model, {"wrist_roll": Np.pi})
        Offset = next(Joint["Origin"][:3, 3] for Joint in Model["Joints"] if Joint["Name"] == "gripper_frame_joint")
        Self.assertAlmostEqual(float(Np.linalg.norm(Rolled - Closed)), float(2 * Np.linalg.norm(Offset[:2])), places=10)
        with Self.assertRaises(ValueError):
            ToolPosition(Model, {"wrong_joint": 0})
        with Self.assertRaises(ValueError):
            ToolPosition(Model, {"elbow_flex": float("nan")})


if __name__ == "__main__":
    Loader = Unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Result = Unittest.TextTestRunner(verbosity=2).run(Loader.loadTestsFromTestCase(ReferenceKinematicsTests))
    Results["Successful"] = Result.wasSuccessful()
    Results["TestsRun"] = Result.testsRun
    Results["MujocoVersion"] = Mujoco.__version__
    (OutputRoot / "KinematicsVerification.json").write_text(Json.dumps(Results, indent=2), encoding="utf-8")
    print(Json.dumps(Results, indent=2))
    raise SystemExit(0 if Result.wasSuccessful() else 1)
