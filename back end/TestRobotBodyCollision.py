"""Fixed mast/camera-head collision regressions. All motors are simulated."""
import copy as Copy
import hashlib as Hashlib
import json as Json
import math as Math
import unittest as Unittest
from pathlib import Path

import numpy as Np
import BasketCollision as Collision
import CadKinematics as Cad
import bridge_server as Bridge
from TestSafety import MakeController
from TestBasketCollision import Place

Root = Path(__file__).resolve().parent
# Both poses clear the collision and rearward checks. Their straight gated
# line would be clear without the mast and is blocked by it.
Start = dict(zip(Cad.JointNames, (-1.1934131855854702, 25.19959872008269, 84.02332822889508,
                                92.46970575371273, 94.39453125, 0.0)))
Goal = dict(zip(Cad.JointNames, (-111.83002421691454, 5.348727706029081, 91.85792255743246,
                               -11.889487676212426, 94.39453125, 0.0)))
FormerlyAccepted = dict(zip(Cad.JointNames, (65.89120577648667, -29.950785957082086, -17.85434311220202,
                                           -74.60905235804546, 94.39453125, 0.0)))


class AbsentBody:
    def Overlaps(Self, *Args, **Options):
        return False


class BodyTests(Unittest.TestCase):
    def TestSourceIncludesEveryRenderedMastAndHeadPart(Self):
        Scene = Json.loads((Root.parent / 'AI Controlled WebApp/src/lib/XLeRobotScene.json').read_text())
        Expected = {Part['Name']: Part for Part in Scene['Parts'] if Part['Category'] in ('Mast', 'Head')}
        Actual = {Part['Name']: Part for Part in Collision.BodyData['Parts']}
        Self.assertEqual(set(Expected), set(Actual))
        for Name in Expected:
            Np.testing.assert_allclose(Actual[Name]['BoundsCm'], Expected[Name]['BoundsCm'], atol=1e-4)
            Self.assertGreater(Actual[Name]['FaceCount'], 0)
            First, Count = Actual[Name]['FirstFace'], Actual[Name]['FaceCount']
            Surface = Collision.BodySurfaces['Right']
            Vertices = Surface.Triangles[First:First + Count].reshape(-1, 3) + Collision.Model['Mounts']['Right']['PanCm']
            Np.testing.assert_allclose([Vertices.min(axis=0), Vertices.max(axis=0)], Expected[Name]['BoundsCm'], atol=1e-4)
        Self.assertEqual(Hashlib.sha256((Root / 'RobotBodySurface.json').read_bytes()).hexdigest(),
                         Collision.Model['RobotBodySurfaceSha256'])
        Self.assertEqual(Collision.Model['Version'], 'robot-body-cad-exact-v8')
        # Body height cannot silently change the basket rim used for entry.
        Basket = next(Part for Part in Scene['Parts'] if Part['Name'] == 'XLebase_link0')
        Self.assertEqual(Collision.Model['BasketAssemblyBoundsCm'], Basket['BoundsCm'])

    def TestBodyTreeIncludesAllFacesWithinItsBounds(Self):
        Surface = Collision.BodySurfaces['Right']
        Faces = []
        for Index, Node in enumerate(Surface.Nodes):
            if 'Children' in Node:
                for Child in Node['Children']:
                    Self.assertTrue(Np.all(Surface.Bounds[Child, 0] >= Surface.Bounds[Index, 0]))
                    Self.assertTrue(Np.all(Surface.Bounds[Child, 1] <= Surface.Bounds[Index, 1]))
            else:
                Faces.extend(Node['Faces'])
                Self.assertTrue(Np.all(Surface.Triangles[Node['Faces']] >= Surface.Bounds[Index, 0]))
                Self.assertTrue(Np.all(Surface.Triangles[Node['Faces']] <= Surface.Bounds[Index, 1]))
        Self.assertEqual(sorted(Faces), list(range(len(Surface.Triangles))))

    def TestContactOnEveryPartIsBlockedInBothArmFrames(Self):
        Controller, _ = MakeController()
        for Side in ('Right', 'Left'):
            World = Collision.CollisionWorld(Controller.arm_geometry, {'RobotFrame': {'ControlledArm': Side}})
            Self.assertIsNone(World.MountReason)
            for Part in Collision.BodyData['Parts']:
                First, Count = Part['FirstFace'], Part['FaceCount']
                for Triangle in World.BodySurface.Triangles[First:First + Count:max(1, Count // 12)]:
                    Point = dict(zip(('x', 'y', 'z'), Triangle.mean(axis=0)))
                    Self.assertIsNotNone(World.TargetReason(Point), (Side, Part['Name'], Point))
                    if Part['Category'] == 'Head':
                        Self.assertIn('camera head', World.TargetReason(Point))

    def TestHeadInteriorIsBlockedButEmptySpaceBesideMastRemainsFree(Self):
        Controller, _ = MakeController()
        World = Controller.CollisionWorld()
        Offset = Np.array(Collision.Model['Mounts']['Right']['PanCm'])
        Head = next(Part for Part in Collision.BodyData['Parts'] if Part['Name'] == 'XLehead_pan_link1')
        Center = Np.mean(Head['BoundsCm'], axis=0) - Offset
        Self.assertTrue(World.BodySurface.Contains(Center))
        Self.assertFalse(World.Surface.Overlaps(Center, Np.eye(3), Np.zeros(3)))
        Self.assertIn('camera head', World.TargetReason(dict(zip(('x', 'y', 'z'), Center))))
        Void = Np.array([0., -10., 100.]) - Offset
        Self.assertTrue(Np.all(Void >= World.BodySurface.Bounds[0, 0]))
        Self.assertTrue(Np.all(Void <= World.BodySurface.Bounds[0, 1]))
        Self.assertIsNone(World.TargetReason(dict(zip(('x', 'y', 'z'), Void))))
        Self.assertIsNone(World.PoseReason(Controller._current_ik_angles()))

    def TestClearTipCannotHideArmContactWithTheBody(Self):
        Controller, _ = MakeController()
        World = Controller.CollisionWorld()
        Tip = Cad.Positions(Controller.arm_geometry, FormerlyAccepted)[-1]
        Self.assertIsNone(World.TargetReason(Tip))
        Legacy = Copy.copy(World)
        Legacy.BodySurface = AbsentBody()
        Self.assertIsNone(Legacy.PoseReason(FormerlyAccepted))
        Self.assertIn('robot body', World.PoseReason(FormerlyAccepted, Tracking=False))
        Self.assertIn('robot body', World.PoseReason(FormerlyAccepted))
        Preview = Controller.preview_ik({**Tip, 'wrist_pitch': Cad.Pitch(Controller.arm_geometry, FormerlyAccepted)})
        Self.assertFalse(Preview['reachable'])
        Self.assertIn('robot body', Preview['collision_reason'])

    def TestHeadTargetRejectsSendWithoutMotorWrites(Self):
        Controller, Bus = MakeController()
        Offset = Np.array(Collision.Model['Mounts']['Right']['PanCm'])
        Head = next(Part for Part in Collision.BodyData['Parts'] if Part['Name'] == 'XLehead_pan_link1')
        Target = dict(zip(('x', 'y', 'z'), Np.mean(Head['BoundsCm'], axis=0) - Offset))
        Ok, Message = Controller.send_ik(Target={**Target, 'wrist_pitch': 0.})
        Self.assertFalse(Ok)
        Self.assertIn('camera head', Message)
        Self.assertFalse(Bus.writes)

    def TestMastCrossingDetourExecutesWithIndependentMotorSpeeds(Self):
        Controller, Bus = MakeController()
        Place(Controller, Bus, Start)
        Controller.elbow_mode = Cad.Branch(Controller.arm_geometry, Goal)
        World = Controller.CollisionWorld()
        Before = Controller._current_ik_angles()
        StartVector, GoalVector = (Np.array([Angles[Name] for Name in Cad.JointNames]) for Angles in (Before, Goal))
        for Angles in (Before, Goal): Self.assertIsNone(World.PoseReason(Angles))
        Legacy = Copy.copy(World)
        Legacy.BodySurface = AbsentBody()
        Self.assertIsNone(Legacy.EdgeReason(StartVector, GoalVector))
        Self.assertIn('robot body', World.EdgeReason(StartVector, GoalVector))
        Target = Cad.Positions(Controller.arm_geometry, Goal)[-1]
        Pitch = Cad.Pitch(Controller.arm_geometry, Goal)
        Ok, Message = Controller.send_ik(Target={**Target, 'wrist_pitch': Pitch})
        Self.assertTrue(Ok, Message)
        Self.assertGreater(len(Controller.MotionRoute['Path']), 1)
        Check = Controller.SolverSnapshot()
        for Point in Controller.MotionRoute['Path']:
            Check.ValidateMotionPath(Point)
            for Name, Raw in Point.items(): Check.servos[Name].present_raw = Raw
        Controller.last_refresh_s = Bridge.time.monotonic()
        for _ in range(12000):
            for Index, Name in enumerate(Cad.JointNames): Bus.step(Name, steps_per_tick=3 + Index)
            Controller.refresh()
            Self.assertIsNone(Controller.MotionFault)
            if Controller.MotionRoute is None: break
        Self.assertIsNone(Controller.MotionRoute)
        Final = Cad.Positions(Controller.arm_geometry, Controller._current_ik_angles())[-1]
        Self.assertLess(Math.dist(tuple(Final.values()), tuple(Target.values())), .8)
        Self.assertTrue(Bus.torque_enabled)


def load_tests(Loader, Tests, Pattern):
    '''`python -m unittest TestRobotBodyCollision` runs the Test* methods too (unittest names this hook).'''
    Prefixed = Unittest.TestLoader()
    Prefixed.testMethodPrefix = 'Test'
    return Prefixed.loadTestsFromTestCase(BodyTests)


if __name__ == '__main__':
    Loader = Unittest.TestLoader()
    Loader.testMethodPrefix = 'Test'
    Result = Unittest.TextTestRunner(verbosity=2).run(Loader.loadTestsFromTestCase(BodyTests))
    raise SystemExit(not Result.wasSuccessful())
