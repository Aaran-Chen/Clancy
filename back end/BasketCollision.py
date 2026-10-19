"""Conservative CAD collision and rearward checks with bounded joint-space routing, no hardware.

Every moving visual is enclosed by a link-local box. Coarse motion bounds cull
distant obstacles; directional enclosures refine possible contact, including
independent servo progress (contact adds ContactTrackingAllowanceRaw, 0 since
v8: CAD-exact). Every moving envelope, with the tracking tolerance, stays ahead
of the rear workspace limit. Inconclusive checks fail closed.
"""
import itertools as Itertools
import json as Json
import math as Math
import time as Time
from pathlib import Path

import numpy as Np
import CadKinematics as Cad
import BasketSurface
import SceneSafety
from ArmMotionEnvelope import BoxEnvelope

Root = Path(__file__).resolve().parent
Model = Json.loads((Root / "CollisionModel.json").read_text())
Names = Cad.JointNames
Degrees = Math.pi / 180
PlannerVersion = "flyby-cad-exact-v8"
RearwardReason = f"Arm cannot move behind Y = {Model['RearwardLimit']['MinimumDisplayYCm']:g} cm"
# Waypoint spacing along a straight joint-space leg near obstacles. The bridge
# passes intermediate waypoints without stopping, but slows near each one.
GateStepDegrees = 6.0
# Subdivision budget for certifying a longer stretch of a finished leg as one
# edge. A miss only keeps its gates, so it fails at the first check.
CoarseEdgeBudget = 0
# A measured pose may press a link box into the model by up to this much: the
# boxes enclose the parts loosely, and an arm dropped by a coast rests on the
# basket (measured on the hardware: 1.8 cm of model overlap for an arm lying on
# the rim). Leaving such a pose keeps each overlapping box inset by its own
# overlap plus the margin, so it can press at most the margin further in. The
# margin is the resolution of the subdivided check (its finest boxes still pad
# a link about a millimetre); without it no escape certifies. Deeper overlap
# means a wrong model or pose, and is refused.
ContactReliefMaximumCm = 2.5
ContactReliefMarginCm = 0.3
# Real servos do not arrive together: a joint can run ahead past a waypoint the
# arm flies past, or still trail it when the next leg starts. The bridge
# certifies each leg's box this many encoder steps beyond such a waypoint, along
# each moving joint's travel and around every joint that may still be settling
# from one, and its route monitor accepts the same region.
WaypointReachRaw = 32
WaypointReachDegrees = WaypointReachRaw * 360 / 4096
# A gripper-only move (CollisionWorld.ToolReason) is certified in jaw steps of at most this much,
# each a small swept box, so the bound stays tight over a full close (about 70 degrees).
ToolStepDegrees = 2.0


class PlanningCancelled(Exception):
    """Route planning ended early: the world's Cancel said the motion it plans for was stopped meanwhile.

    Not a ValueError, so no search loop takes it for a blocked edge and goes on: it ends the planning
    at the next pose check. The bridge sets Cancel on its planning snapshots (bridge_server.PlanningSnapshot):
    a Stop, Hold, Coast or emergency stop (anything that bumps its MotionEpoch) ends a queued plan within
    one check instead of after its 2.5 + 6 + 9 s budgets (2026-10-07 audit M2).
    """


class RoutePath(list):
    """Route waypoints, plus the indices of tight waypoints: those without the waypoint reach.

    The arm passes a tight waypoint within the plain tracking allowance instead
    of flying past it. An arrive waypoint keeps the reach for its own approach,
    but the stage after it has none at its start, so there too every joint
    arrives within that allowance before the next stage. Reach maps a waypoint
    index to the reach (encoder steps, per joint) its stage was certified with
    beyond it, where that is not WaypointReachRaw for every joint. The route end
    always settles. Waypoints stay plain joint dictionaries.
    """

    def __init__(Self, Points=(), Tight=(), Arrive=(), Reach=None):
        super().__init__(Points)
        Self.Tight = set(Tight)
        Self.Arrive = set(Arrive)
        Self.Reach = dict(Reach or {})


Surfaces = {Side: BasketSurface.Surface(Mount["PanCm"]) for Side, Mount in Model["Mounts"].items()}
BodyData = Json.loads((Root / "RobotBodySurface.json").read_text())
BodySurfaces = {Side: BasketSurface.Surface(Mount["PanCm"], BodyData) for Side, Mount in Model["Mounts"].items()}


def BasketBounds(Frame):
    Side = Frame.get("RobotFrame", {}).get("ControlledArm", "Right")
    return Np.array(Model["BasketAssemblyBoundsCm"]) - Np.array(Model["Mounts"][Side]["PanCm"])


def CollarSweep(Bounds):
    """The model-frame box a shoulder_link box (BoundsCm, link frame) sweeps over its complete pan revolution:
    the mounting collar CollisionWorld certifies against the basket instead of exempting it."""
    Corners = Np.array(list(Itertools.product(*zip(Bounds[0], Bounds[1]))))
    Frame = Cad.ForwardUrdf({})[1]["shoulder_pan"]
    Centers = Frame[:3, 3] + Corners[:, 2, None] * Frame[:3, 2]
    Extents = Np.linalg.norm(Corners[:, :2], axis=1)[:, None] * Np.linalg.norm(Frame[:3, :2], axis=1)
    return Np.array([(Centers - Extents).min(axis=0), (Centers + Extents).max(axis=0)])


def BoxesOverlap(Center, Rotation, Half, Bounds, Padding=0.0):
    """OBB/AABB separating-axis test, with a spherical clearance lower bound."""
    OtherCenter = (Bounds[0] + Bounds[1]) / 2
    OtherHalf = (Bounds[1] - Bounds[0]) / 2 + Padding
    Delta = Center - OtherCenter
    # World axes cheaply reject most parts, then box axes and cross products.
    if Np.any(Np.abs(Delta) > OtherHalf + Np.abs(Rotation) @ Half):
        return False
    if Np.any(Np.abs(Rotation.T @ Delta) > Half + Np.abs(Rotation.T) @ OtherHalf):
        return False
    for Axis in Np.eye(3):
        for Local in Rotation.T:
            Cross = Np.cross(Axis, Local)
            if abs(Delta @ Cross) > OtherHalf @ Np.abs(Cross) + Half @ Np.abs(Rotation.T @ Cross) + 1e-9:
                return False
    return True


class CollisionWorld:
    def __init__(Self, Geometry, Frame):
        Side = Frame.get("RobotFrame", {}).get("ControlledArm", "Right")
        Self.Geometry = Geometry
        # A callable answering whether the planning this world serves was cancelled (PlanningCancelled); None: never.
        Self.Cancel = None
        Self.Bounds = BasketBounds(Frame)
        Self.Surface = Surfaces[Side]
        Self.BodySurface = BodySurfaces[Side]
        Self.SceneObstacles = [(str(Item.get("label", "measured scene obstacle")), SceneSafety.ModelBounds(Item, Frame))
                               for Item in Frame.get("SceneObstacles", [])]
        Carried = Frame.get("SceneCarried")
        Self.SceneFree = ([SceneSafety.ModelBounds(Item, Frame) for Item in Frame["SceneFree"]]
                          if "SceneFree" in Frame else None)
        Self.MinimumY = (Model["RearwardLimit"]["MinimumDisplayYCm"]
                        + Frame.get("RobotFrame", {}).get("OriginAssemblyCm", Model["Mounts"][Side]["PanCm"])[1]
                        - Model["Mounts"][Side]["PanCm"][1])
        # The tracking allowance (degrees): the route monitor's, and the rearward limit's. Basket,
        # cart and body contact adds ContactSlack instead: 0 since v8, CAD-exact (2026-10-04 08:32
        # the arm stood at the grasp point clear of the basket, and the close was refused there
        # because the gripper box's 16-step envelope touched the cart, the box itself 2.4 cm off).
        Self.Slack = Model["JointTrackingAllowanceRaw"] * 360 / 4096
        Self.ContactSlack = Model["ContactTrackingAllowanceRaw"] * 360 / 4096
        Self.Boxes = []
        Self.MountReason = None
        for Box in Model["ArmBoxes"]:
            Bounds = Np.array(Box["BoundsCm"])
            Corners = Np.array(list(Itertools.product(*zip(Bounds[0], Bounds[1]))))
            Radii = Self.MotionRadii(Box["Link"], float(Np.max(Np.linalg.norm(Corners, axis=1))))
            if Box["Link"] == "shoulder_link":
                # This mounting collar is designed to sit immediately above the
                # bin. Certify its COMPLETE pan revolution against the basket
                # surfaces instead of exempting it from collision tests.
                Sweep = CollarSweep(Bounds)
                Reason = Self.ObstacleReason(Sweep.mean(axis=0), Np.eye(3), (Sweep[1] - Sweep[0]) / 2,
                                             Model["MountClearanceCm"])
                if Reason:
                    Self.MountReason = f"Shoulder mounting collar intersects the {Reason}"
                continue
            Self.Boxes.append((Box["Name"], Box["Link"], Bounds.mean(axis=0), (Bounds[1] - Bounds[0]) / 2, Radii))
        # Subdivide the joint whose travel moves the arm furthest first.
        Self.SplitWeights = Np.max([Box[4] for Box in Self.Boxes], axis=0)
        if Carried:
            Bounds = Np.asarray(Carried["bounds"], dtype=float)
            if Bounds.shape != (2, 3) or not Np.isfinite(Bounds).all() or Np.any(Bounds[1] <= Bounds[0]):
                raise ValueError("Held object geometry is invalid")
            Link = "gripper_link"
            Corners = Np.array(list(Itertools.product(*zip(Bounds[0], Bounds[1]))))
            Radii = Self.MotionRadii(Link, float(Np.max(Np.linalg.norm(Corners, axis=1))))
            Self.Boxes.append(("held object", Link, Bounds.mean(axis=0), (Bounds[1] - Bounds[0]) / 2, Radii))
            Self.SplitWeights = Np.maximum(Self.SplitWeights, Radii)

    @staticmethod
    def MotionRadii(Link, Radius):
        Radii = Np.zeros(len(Names))
        while Link != Cad.Model["Root"]:
            Joint = next(J for J in Cad.OrderedJoints if J["Child"] == Link)
            if Joint["Name"] in Names:
                Radii[Names.index(Joint["Name"])] = Radius
            Radius += float(Np.linalg.norm(Cad.Origins[Joint["Name"]][:3, 3]))
            Link = Joint["Parent"]
        return Radii

    def ObstacleReason(Self, Center, Rotation, Half, Padding=0.0):
        if Self.Surface.Overlaps(Center, Rotation, Half, Padding):
            return "basket"
        if Self.BodySurface.Overlaps(Center, Rotation, Half, Padding):
            return "robot body or camera head"
        for Label, Bounds in Self.SceneObstacles:
            if BoxesOverlap(Center, Rotation, Half, Bounds, Padding):
                return Label
        return None

    def TargetReason(Self, Point):
        Value = Np.array([Point[Axis] for Axis in ("x", "y", "z")], dtype=float)
        if not Np.all(Np.isfinite(Value)):
            return "Target coordinates must be finite"
        Reason = Self.ObstacleReason(Value, Np.eye(3), Np.zeros(3), Model["ClearanceCm"])
        if Reason:
            return f"Target touches or enters {Reason} material"
        if Value[1] < Self.MinimumY:
            return RearwardReason
        return None

    def SweptObstacleReason(Self, Center, Half, Radii, Transform, Frames, Motion):
        """Refine coarse contact with the directions joints can actually move.

        Both envelopes contain every pose in the interval. Either can prove
        separation; overlap alone is inconclusive and must pass the refinement.
        Keep the cheap spherical bound as the fast path for distant obstacles.
        """
        WorldCenter = Transform[:3, :3] @ Center + Transform[:3, 3]
        Padding = Model["ClearanceCm"] + float(Radii @ Motion)
        Envelope = None
        for Surface, Reason in ((Self.Surface, "basket"), (Self.BodySurface, "robot body or camera head")):
            if not Surface.Overlaps(WorldCenter, Transform[:3, :3], Half, Padding):
                continue
            if Np.any(Motion):
                if Envelope is None:
                    Envelope = BoxEnvelope(Center, Half, Radii, Transform, Frames, Motion)
                if not Surface.Overlaps(*Envelope, Model["ClearanceCm"]):
                    continue
            return Reason
        return Self.SweptSceneReason(Center, Half, Radii, Transform, Frames, Motion)

    def SweptSceneReason(Self, Center, Half, Radii, Transform, Frames, Motion):
        WorldCenter = Transform[:3, :3] @ Center + Transform[:3, 3]
        Padding = Model["ClearanceCm"] + float(Radii @ Motion)
        Envelope = None
        for Label, Bounds in Self.SceneObstacles:
            if not BoxesOverlap(WorldCenter, Transform[:3, :3], Half, Bounds, Padding):
                continue
            if Np.any(Motion):
                if Envelope is None:
                    Envelope = BoxEnvelope(Center, Half, Radii, Transform, Frames, Motion)
                if not BoxesOverlap(*Envelope, Bounds, Model["ClearanceCm"]):
                    continue
            return Label
        return None

    def SceneCoverageReason(Self, Center, Half, Radii, Transform, Frames, Motion):
        if Self.SceneFree is None:
            return None
        Center_, Rotation, Half_ = BoxEnvelope(Center, Half, Radii, Transform, Frames, Motion)
        Extent = Np.abs(Rotation) @ Half_ + Model["ClearanceCm"]
        if not SceneSafety.Covered(Np.array([Center_ - Extent, Center_ + Extent]), Self.SceneFree):
            return "gripper or held object enters space not observed by the depth camera"
        return None

    def RearwardExcess(Self, Links, Motion=None):
        """Maximum envelope distance behind the display-Y limit; negative ahead.

        The Y support of each oriented box is exact. Motion padding bounds all
        corners throughout an interval, including independently moving joints.
        Contact relief never shrinks the envelopes used for this workspace limit.
        """
        Excess = -Math.inf
        for _, Link, Center, Half, Radii in Self.Boxes:
            Row = Links[Link][1]
            Minimum = float(Row[:3] @ Center + Row[3] - Np.abs(Row[:3]) @ Half)
            Padding = 0.0 if Motion is None else float(Radii @ Motion)
            Excess = max(Excess, Self.MinimumY - Minimum + Padding)
        return Excess

    def IntervalReason(Self, Low, High, Tracking=True, RearwardSlack=0.0, Inset=None, Contact=True, Rearward=True):
        """Why some pose in the joint interval [Low, High] is unsafe, or None.

        Tracking widens the interval by the tracking allowance: ContactSlack for
        basket, cart and body contact (0: CAD-exact), Slack for the rearward limit.
        Contact, Rearward: which of the two to check (EdgeReason certifies each over
        its own region). Inset (cm per index into Self.Boxes) shrinks those link
        boxes; it is only for leaving a measured pose that already overlaps the
        model (StartRelief).
        """
        if Self.MountReason:
            return Self.MountReason
        Cancel = getattr(Self, "Cancel", None)
        if Cancel is not None and Cancel():
            raise PlanningCancelled("Route planning stopped: the arm was stopped (Stop, Hold, Coast or emergency stop) "
                                    "or another motion started while it planned. No motion sent.")
        Mid = (Low + High) / 2
        Spread = (High - Low) / 2
        Angles = dict(zip(Names, Mid))
        Links, Frames = Cad.Forward(Self.Geometry, Angles)
        # Contact first: it is the more useful reason when both apply.
        if Contact:
            Motion = (Spread + (getattr(Self, "ContactSlack", Self.Slack) if Tracking else 0)) * Degrees
            for Index, (Name, Link, Center, Half, Radii) in enumerate(Self.Boxes):
                Transform = Links[Link]
                if Link in ("gripper_link", "moving_jaw_so101_v1_link"):
                    Unknown = Self.SceneCoverageReason(Center, Half, Radii, Transform, Frames, Motion)
                    if Unknown:
                        return Unknown
                # Relief may not shrink an arm/held-object box through a measured shelf.
                if Self.SceneObstacles:
                    Reason = Self.SweptSceneReason(Center, Half, Radii, Transform, Frames, Motion)
                    if Reason:
                        return f"{Name.split('__')[0].replace('_', ' ')} could contact the {Reason}"
                if Inset and Index in Inset:
                    Half = Np.maximum(Half - Inset[Index], 0.0)
                Reason = Self.SweptObstacleReason(Center, Half, Radii, Transform, Frames, Motion)
                if Reason:
                    return f"{Name.split('__')[0].replace('_', ' ')} could contact the {Reason}"
        if Rearward and Self.RearwardExcess(Links, (Spread + (Self.Slack if Tracking else 0)) * Degrees) > RearwardSlack:
            return RearwardReason
        return None

    def ToolReason(Self, Start, Goal):
        """Why opening or closing the gripper from Start to Goal (only the gripper changes) is unsafe, or None.

        The arm holds where it is: only the moving jaw moves, so only its CAD boxes
        are judged, over the opening's whole travel in steps of at most
        ToolStepDegrees (plus ContactSlack). Where the rest of the arm stands is
        where it is, a measured pose a jaw move cannot change: 2026-10-04 08:32
        the bridge refused to close the jaws at a grasp pose the arm had just
        descended to, clear of the basket.
        """
        Jaw = Names.index("gripper")
        Count = max(1, Math.ceil(abs(float(Goal[Jaw] - Start[Jaw])) / ToolStepDegrees))
        for Step in range(Count):
            Low, High = Start.astype(float).copy(), Start.astype(float).copy()
            Low[Jaw], High[Jaw] = sorted((Start[Jaw] + (Goal[Jaw] - Start[Jaw]) * Step / Count,
                                          Start[Jaw] + (Goal[Jaw] - Start[Jaw]) * (Step + 1) / Count))
            Mid, Spread = (Low + High) / 2, (High - Low) / 2
            Links, Frames = Cad.Forward(Self.Geometry, dict(zip(Names, Mid)))
            Spread[Jaw] += getattr(Self, "ContactSlack", Self.Slack)
            for Name, Link, Center, Half, Radii in Self.Boxes:
                if not Radii[Jaw]:
                    continue
                Unknown = Self.SceneCoverageReason(Center, Half, Radii, Links[Link], Frames, Spread * Degrees)
                if Unknown:
                    return Unknown
                Reason = Self.SweptObstacleReason(Center, Half, Radii, Links[Link], Frames, Spread * Degrees)
                if Reason:
                    return f"{Name.split('__')[0].replace('_', ' ')} could contact the {Reason}"
        return None

    def PoseReason(Self, Angles, Tracking=True, RearwardSlack=0.0):
        Values = Np.array([Angles.get(Name, 0.0) for Name in Names])
        if RearwardSlack:
            return Self.IntervalReason(Values, Values, Tracking, RearwardSlack)
        return Self.IntervalReason(Values, Values, Tracking)

    def ContactInset(Self, Start):
        """How far (cm) each link box must shrink for a measured pose to clear the model.

        Only boxes that overlap need an inset. Raises if any would need more than
        ContactReliefMaximumCm: that is not an arm resting against the cart, but a
        wrong model or pose.
        """
        Links, _ = Cad.Forward(Self.Geometry, dict(zip(Names, Start)))
        Inset = {}
        for Index, (Name, Link, Center, Half, _) in enumerate(Self.Boxes):
            Transform = Links[Link]
            Place = (Transform[:3, :3] @ Center + Transform[:3, 3], Transform[:3, :3])
            Overlaps = lambda Shrink: Self.ObstacleReason(*Place, Np.maximum(Half - Shrink, 0.0), Model["ClearanceCm"])
            if not Overlaps(0.0):
                continue
            Reason = Overlaps(ContactReliefMaximumCm)
            if Reason:
                Part = Name.split("__")[0].replace("_", " ")
                raise ValueError(f"Current arm pose: {Part} overlaps the {Reason} by more than {ContactReliefMaximumCm:g} cm. "
                                 "Coast and move the arm clear by hand. No motion sent.")
            Clear, Blocked = ContactReliefMaximumCm, 0.0
            for _ in range(12):
                Middle = (Clear + Blocked) / 2
                Clear, Blocked = (Clear, Middle) if Overlaps(Middle) else (Middle, Blocked)
            Inset[Index] = Clear + ContactReliefMarginCm
        return Inset

    def StartRelief(Self, Start):
        """Measured clearance for leaving a start pose already inside the allowance.

        Returns None for a pose with the full tracking allowance inside the workspace.
        A measured pose is where the arm physically is, so a pose that only lacks the
        allowance may still be left. So may a pose that presses a link box slightly
        into the model, as an arm dropped by a coast rests on the basket: the boxes
        that overlap are inset by their own overlap plus ContactReliefMarginCm, so
        leaving can press at most that margin further in. Deeper overlap is refused.
        """
        if Self.IntervalReason(Start, Start) is None:
            return None
        Inset = Self.ContactInset(Start)
        Contact = Self.IntervalReason(Start, Start, False, Math.inf, Inset)
        if Contact:
            raise ValueError(f"Current arm pose: {Contact}. Coast and move the arm clear by hand. No motion sent.")
        Clear, Blocked = 0.0, Self.Slack
        if Self.IntervalReason(Start - Blocked, Start + Blocked, False, Math.inf, Inset) is None:
            Clear = Blocked
        for _ in range(0 if Clear else 10):
            Middle = (Clear + Blocked) / 2
            if Self.IntervalReason(Start - Middle, Start + Middle, False, Math.inf, Inset) is None:
                Clear = Middle
            else:
                Blocked = Middle
        Links, _ = Cad.Forward(Self.Geometry, dict(zip(Names, Start)))
        Excess = Self.RearwardExcess(Links, Np.full(len(Names), Clear * Degrees))
        return {"SlackDegrees": Clear, "RearwardSlackCm": max(0.0, Excess) + 1e-9,
                "InsetCm": Inset}

    def RegionReason(Self, Low, High, RearwardSlack=0.0, Inset=None, Check=None):
        # Check: "contact" or "rearward" only (EdgeReason's own regions); None both.
        Only = {} if Check is None else {"Contact": Check == "contact", "Rearward": Check == "rearward"}
        if Inset:
            return Self.IntervalReason(Low, High, False, RearwardSlack, Inset, **Only)
        if RearwardSlack:
            return Self.IntervalReason(Low, High, False, RearwardSlack, **Only)
        return Self.IntervalReason(Low, High, False, **Only)

    def EdgeReason(Self, Start, Goal, Budget=256, Relief=None, Reach=(0.0, 0.0)):
        """Certify every pose the route monitor accepts, not just a tip line.

        That region is the independent-progress box widened by the tracking
        allowance. It is subdivided directly, so sub-boxes shrink toward exact
        poses instead of each keeping the whole allowance. Relief is only for an
        edge leaving a measured start pose inside the allowance: the faces at that
        pose widen by its own measured clearance, the far faces by the allowance
        (or, for a return to another measured pose, by that pose's clearance), and
        link boxes overlapping the model at that pose keep its inset throughout.
        Reach (degrees at the start and goal ends) further extends each moving
        joint's face at a waypoint the arm flies past. The start reach may be per
        joint: it also widens a joint that holds still on this edge on both sides,
        so give it only to joints that may still be settling from that waypoint.
        Since v8 (CAD-exact contact) basket, cart and body contact is certified
        over the region widened by ContactSlack (0: the region the route commands,
        the reach included), the rearward limit over the region widened by the
        full allowance, as before.
        """
        Slack = getattr(Self, "Slack", 0.0)
        ContactSlack = getattr(Self, "ContactSlack", Slack)
        RearwardSlack = 0.0 if Relief is None else Relief["RearwardSlackCm"]
        Inset = None if Relief is None else Relief.get("InsetCm")
        if ContactSlack == Slack:
            return Self.RegionCertify(*Self.EdgeRegion(Start, Goal, Slack, Relief, Reach), Budget, RearwardSlack, Inset, None)
        Low, High = Self.EdgeRegion(Start, Goal, Slack, Relief, Reach)
        Reason = (Self.RegionCertify(*Self.EdgeRegion(Start, Goal, ContactSlack, Relief, Reach), Budget, RearwardSlack, Inset, "contact")
                  or Self.RegionCertify(Low, High, Budget, RearwardSlack, Inset, "rearward"))
        # As one region certified both: the reason at its centre pose when there is one.
        Mid = (Low + High) / 2
        return Reason and (Self.RegionReason(Mid, Mid, RearwardSlack, Inset) or Reason)

    @staticmethod
    def EdgeRegion(Start, Goal, Slack, Relief, Reach):
        """EdgeReason's region: the edge's independent-progress box, its faces widened by Slack and the reach."""
        Near = Slack if Relief is None else min(Slack, Relief["SlackDegrees"])
        Far = Slack if Relief is None else min(Slack, Relief.get("FarSlackDegrees", Slack))
        Low = Np.where(Goal > Start, Start - Near - Reach[0], Goal - Far - Reach[1])
        High = Np.where(Goal < Start, Start + Near + Reach[0], Goal + Far + Reach[1])
        # A joint that holds still on this edge may still be settling from a
        # flown-past start, behind it or beyond it (only when given start reach).
        Low = Np.where(Goal == Start, Start - min(Near, Far) - Reach[0], Low)
        High = Np.where(Goal == Start, Start + min(Near, Far) + Reach[0], High)
        return Low, High

    def RegionCertify(Self, Low, High, Budget, RearwardSlack, Inset, Check):
        """EdgeReason's subdivided certificate of one region [Low, High] (Check: which test, None both)."""
        Only = {} if Check is None else {"Check": Check}
        Weights = getattr(Self, "SplitWeights", None)
        Weights = Np.ones(len(Low)) if Weights is None else Weights
        Pending = [(Low, High)]
        while Pending:
            Low, High = Pending.pop()
            Reason = Self.RegionReason(Low, High, RearwardSlack, Inset, **Only)
            if Reason is None:
                continue
            # A real collision (or insufficient tracking clearance) at the
            # interval center cannot be repaired by subdivision.
            Mid = (Low + High) / 2
            MidReason = Self.RegionReason(Mid, Mid, RearwardSlack, Inset, **Only)
            if MidReason:
                return MidReason
            if Budget <= 0:
                return Reason
            Spread = (High - Low) * Weights
            Index = int(Np.argmax(Spread))
            if Spread[Index] < 0.2 * float(Weights.max()):
                return Reason
            Budget -= 1
            LeftHigh, RightLow = High.copy(), Low.copy()
            LeftHigh[Index] = RightLow[Index] = Mid[Index]
            Pending.extend(((Low, LeftHigh), (RightLow, High)))
        return None

    def LegGates(Self, Start, Goal, Until=None, Snap=None):
        """Waypoints certifying a straight joint-space leg, excluding its start; None if blocked.

        A long leg's single independent-progress box spans poses far from the
        line (one joint finished, another not started). Evenly spaced waypoints
        keep every certified box close to the line itself. Every gate needs the
        full allowance, so a leg leaving a start pose without it cannot be gated.

        Snap maps a pose to the nearest one the servos can hold exactly; every
        certified waypoint, both ends included, is snapped first.
        """
        Snap = Snap or (lambda Values: Values)
        Start, Goal = Snap(Start), Snap(Goal)
        Count = max(1, Math.ceil(float(Np.max(Np.abs(Goal - Start))) / GateStepDegrees - 1e-9))
        Points = [Snap(Start + (Goal - Start) * (Index / Count)) for Index in range(1, Count)] + [Goal]
        Previous = Start
        for Point in Points:
            if Until is not None and Time.monotonic() >= Until:
                return None
            if Self.EdgeReason(Previous, Point, Budget=64):
                return None
            Previous = Point
        return Points

    def CoarsenLeg(Self, Start, Gates, Until=None):
        """Drop gates from a certified straight leg wherever a longer stretch of it certifies at once.

        In open space the whole leg's box is clear, so it becomes one edge and the
        arm cruises along it instead of slowing near every gate. Elsewhere the leg
        is halved; a stretch that is not clear at the first check keeps its gates.
        Only finished route legs are coarsened, not the search's trial legs.
        """
        def Span(First, Last):
            if Last - First == 1 or (Until is not None and Time.monotonic() >= Until):
                return Gates[First:Last]
            if Self.EdgeReason(Start if First == 0 else Gates[First - 1], Gates[Last - 1], Budget=CoarseEdgeBudget) is None:
                return [Gates[Last - 1]]
            Middle = (First + Last) // 2
            return Span(First, Middle) + Span(Middle, Last)
        return Span(0, len(Gates))

    @staticmethod
    def RouteDicts(Points):
        """Joint dictionaries for a certified waypoint list. The bridge decides which waypoints get the reach."""
        return RoutePath(dict(zip(Names, Values)) for Values in Points)

    @staticmethod
    def PathCost(Path, EdgeCost):
        return sum(EdgeCost(Start, Goal) for Start, Goal in zip(Path, Path[1:]))

    def ImprovePath(Self, Path, EdgeCost, Deadline, Lower=None, Upper=None, LegClear=None):
        """Remove bends and pull them inward, certifying EVERY replacement leg."""
        LegClear = LegClear or (lambda A, B, Until: Self.EdgeReason(A, B, Budget=64) is None)
        Path = list(Path)
        for _ in range(4):
            # Try the shortcuts with the greatest time saving first. Do not
            # impose the old eight-node cap on a possible shortcut.
            Costs = Np.cumsum([0.0] + [EdgeCost(A, B) for A, B in zip(Path, Path[1:])])
            Shortcuts = sorted((
                (Costs[End] - Costs[Begin] - EdgeCost(Path[Begin], Path[End]), Begin, End)
                for Begin in range(len(Path) - 2) for End in range(Begin + 2, len(Path))
            ), reverse=True)
            Changed = False
            for Saving, Begin, End in Shortcuts:
                if Time.monotonic() >= Deadline:
                    return Path
                if Saving > 1e-8 and LegClear(Path[Begin], Path[End], Deadline):
                    Path = Path[:Begin + 1] + Path[End:]
                    Changed = True
                    break
            if Changed:
                continue
            for Index in range(1, len(Path) - 1):
                Before, Point, After = Path[Index - 1:Index + 2]
                Delta = After - Before
                Fraction = float(Np.clip((Point - Before) @ Delta / max(1e-9, Delta @ Delta), 0, 1))
                Nearest = Before + Fraction * Delta
                OldCost = EdgeCost(Before, Point) + EdgeCost(Point, After)
                for Pull in (0.75, 0.5, 0.25):
                    if Time.monotonic() >= Deadline:
                        return Path
                    Candidate = Point + Pull * (Nearest - Point)
                    if Lower is not None:
                        Candidate = Np.clip(Candidate, Lower, Upper)
                    if EdgeCost(Before, Candidate) + EdgeCost(Candidate, After) >= OldCost - 1e-8:
                        continue
                    if LegClear(Before, Candidate, Deadline) and LegClear(Candidate, After, Deadline):
                        Path[Index] = Candidate
                        Changed = True
                        break
            if not Changed:
                break
        return Path

    def Plan(Self, StartAngles, GoalAngles, Limits, Seconds=5.0, EdgeCost=None, LocalOnly=False, Snap=None):
        """Certified joint-space route from StartAngles to GoalAngles, excluding the start.

        Snap, if given, maps a pose to the nearest one the servos can hold
        exactly. Every returned waypoint is then snapped before it is certified,
        so the caller can execute exactly the certified edges. Start and goal
        must already be exact.
        """
        Snap = Snap or (lambda Values: Values)
        Deadline = Time.monotonic() + Seconds
        Start = Np.array([StartAngles[Name] for Name in Names])
        Goal = Np.array([GoalAngles[Name] for Name in Names])
        Lower = Np.array([Limits[Name][0] for Name in Names], dtype=float)
        Upper = Np.array([Limits[Name][1] for Name in Names], dtype=float)
        if not Np.all(Np.isfinite([Start, Goal, Lower, Upper])) or Np.any(Lower > Upper):
            raise ValueError("Joint poses and limits must be finite and ordered. No motion sent.")
        if Np.any(((Goal < Lower) | (Goal > Upper)) & (Goal != Start)):
            raise ValueError("Target arm pose exceeds joint limits. No motion sent.")
        # Hold the tool axes unless explicitly requested; also preserve any
        # unchanged, hand-posed joint outside its GUI limits.
        for Index in range(len(Names)):
            if Start[Index] == Goal[Index] and (Index >= 4 or not Lower[Index] <= Start[Index] <= Upper[Index]):
                Lower[Index] = Upper[Index] = Start[Index]
        # Only the gripper moves, the arm holding where it is: the moving jaw's CAD boxes over the
        # opening's travel (ToolReason), and the rearward limit at the new opening as before.
        Jaw = Names.index("gripper")
        if Start[Jaw] != Goal[Jaw] and Np.array_equal(Np.delete(Start, Jaw), Np.delete(Goal, Jaw)):
            Reason = Self.ToolReason(Start, Goal) or Self.IntervalReason(Goal, Goal, Contact=False)
            if Reason:
                raise ValueError(f"Target arm pose: {Reason}. No motion sent.")
            return Self.RouteDicts([Goal])
        Relief = Self.StartRelief(Start)
        Reason = Self.IntervalReason(Goal, Goal)
        if Reason:
            raise ValueError(f"Target arm pose: {Reason}. No motion sent.")
        # The bridge supplies a slowdown-aware duration estimate; the default
        # is concurrent angular travel plus a small cost for each stop.
        EdgeCost = EdgeCost or (lambda A, B: float(Np.max(Np.abs(B - A))) + 2.0)
        if Relief is not None:
            return Self.PlanEscape(Start, Goal, Lower, Upper, Relief, Deadline, EdgeCost, LocalOnly, Snap)
        return Self.PlanFrom(Start, Goal, Lower, Upper, Deadline, EdgeCost, LocalOnly, Snap)

    def PlanEscape(Self, Start, Goal, Lower, Upper, Relief, Deadline, EdgeCost, LocalOnly, Snap):
        """Leave a start pose inside the allowance by the shortest checked leg, then plan normally.

        Only that first leg uses the start pose's measured clearance, so the
        relaxed part of the route is as short as possible and every later leg has
        the full allowance and rearward check.
        """
        Toward = Goal - Start
        Singles, Pairs = [], []
        if float(Np.max(Np.abs(Toward))) > 1e-9:
            Singles.append((Toward / float(Np.max(Np.abs(Toward))), float(Np.max(Np.abs(Toward)))))
        for Count, Group in ((1, Singles), (2, Pairs)):
            for Joints in Itertools.combinations(range(4), Count):
                for Signs in Itertools.product((-1.0, 1.0), repeat=Count):
                    Unit = Np.zeros(len(Names))
                    Unit[list(Joints)] = Signs
                    Group.append((Unit, 90.0))
        # Single-joint and toward-goal escapes first. Searching is capped so
        # planning onward from an escape keeps most of the time.
        for Group in (Singles, Pairs):
            SearchUntil = Time.monotonic() + max(0.05, (Deadline - Time.monotonic()) * 0.4)
            Escapes = []
            for Unit, Longest in Group:
                Previous = 0.0
                for Magnitude in (1.0, 2.0, 4.0, 8.0, 16.0, 32.0, 64.0, 90.0):
                    if Time.monotonic() >= SearchUntil:
                        break
                    Magnitude = min(Magnitude, Longest)
                    Point = Np.clip(Start + Unit * Magnitude, Lower, Upper)
                    if Self.IntervalReason(Point, Point) is None:
                        # Shortest clear step, then a little further so encoder
                        # rounding cannot put the waypoint back inside the allowance.
                        Clear, Blocked = Magnitude, Previous
                        for _ in range(4):
                            Middle = Np.clip(Start + Unit * (Clear + Blocked) / 2, Lower, Upper)
                            if Self.IntervalReason(Middle, Middle) is None:
                                Clear = (Clear + Blocked) / 2
                            else:
                                Blocked = (Clear + Blocked) / 2
                        # One edge, not gates: every pose short of this point still
                        # lacks the allowance, so no intermediate gate could be certified.
                        for Step in (min(Longest, Clear + 0.5), Clear):
                            Point = Snap(Np.clip(Start + Unit * Step, Lower, Upper))
                            if (Self.IntervalReason(Point, Point) is None
                                    and Self.EdgeReason(Start, Point, Budget=64, Relief=Relief) is None):
                                Escapes.append((EdgeCost(Start, Point) + EdgeCost(Point, Goal), Point))
                                break
                        break
                    if Magnitude >= Longest:
                        break
                    Previous = Magnitude
            for _, Point in sorted(Escapes, key=lambda Item: Item[0]):
                if Time.monotonic() >= Deadline:
                    break
                if Np.allclose(Point, Goal, rtol=0, atol=1e-9):
                    return RoutePath([dict(zip(Names, Goal))])
                try:
                    Rest = Self.PlanFrom(Point, Goal, Lower, Upper, Deadline, EdgeCost, LocalOnly, Snap)
                except ValueError:
                    continue
                # The escape waypoint is tight: its edge uses the start relief,
                # which has no room for waypoint reach.
                return RoutePath([dict(zip(Names, Point)), *Rest], {0, *(Index + 1 for Index in Rest.Tight)})
        raise ValueError("The arm is inside the safety margin of an obstacle or behind the rearward limit, "
                         "and no checked way out toward this target was found. No motion sent.")

    def PlanFrom(Self, Start, Goal, Lower, Upper, Deadline, EdgeCost, LocalOnly, Snap=None):
        """Plan legs (bends) between two clear poses, then expand each into gates.

        Candidates are ranked by the bridge's time estimate. Every leg is
        certified through its gates, so every returned consecutive pair passes
        the full independent-progress check.
        """
        Seconds = max(0.0, Deadline - Time.monotonic())
        SearchDeadline = Deadline - min(0.35, Seconds * 0.1)
        Legs = {}

        def Leg(A, B, Until):
            Key = A.tobytes() + B.tobytes()
            if Key not in Legs:
                Gates = Self.LegGates(A, B, Until, Snap)
                if Gates is None and Time.monotonic() >= Until:
                    return None  # Interrupted, not blocked: do not remember.
                Legs[Key] = Gates
            return Legs[Key]

        Snap = Snap or (lambda Values: Values)

        def Expand(Path):
            Points = []
            for A, B in zip(Path, Path[1:]):
                Gates = Leg(A, B, Math.inf)
                if Gates is None:
                    raise ValueError("A planned leg failed certification when expanded. No motion sent.")
                Points.extend(Self.CoarsenLeg(Snap(A), Gates, Deadline))
            return Self.RouteDicts(Points)

        Direct = Leg(Start, Goal, Deadline)
        if Direct is not None:
            return Self.RouteDicts(Self.CoarsenLeg(Snap(Start), Direct, Deadline))

        def Finish(Path):
            Refined = Self.ImprovePath(Path, EdgeCost, min(Deadline, Time.monotonic() + 0.35), Lower, Upper,
                                       LegClear=lambda A, B, Until: Leg(A, B, Until) is not None)
            return Expand(Refined)

        def Clear(Path, Until):
            for Point in Path[1:-1]:
                if Time.monotonic() >= Until or Self.IntervalReason(Point, Point):
                    return False
            return all(Leg(Before, After, Until) is not None for Before, After in zip(Path, Path[1:]))

        Candidates, Seen = [], set()

        def Add(Interior):
            Path = [Start]
            for Point in [Np.clip(Value, Lower, Upper) for Value in Interior] + [Goal]:
                if not Np.allclose(Path[-1], Point, rtol=0, atol=1e-7):
                    Path.append(Point)
            if len(Path) < 3:  # The direct leg has already failed certification.
                return
            Key = tuple(Np.array(Path).ravel())
            if Key not in Seen:
                Seen.add(Key)
                Deviation = sum(float(Np.linalg.norm(Point - (Start + Fraction * (Goal - Start))))
                                for Point, Fraction in zip(Path[1:-1], Np.linspace(0, 1, len(Path))[1:-1]))
                Candidates.append((Self.PathCost(Path, EdgeCost), Deviation, Path))

        for Fraction in (0.25, 0.5, 0.75):
            Middle = Start + Fraction * (Goal - Start)
            for Index in range(4):
                for Offset in (-64, -32, -16, -8, -4, 4, 8, 16, 32, 64):
                    Point = Middle.copy()
                    Point[Index] += Offset
                    Add([Np.clip(Point, Lower, Upper)])
                for Value in (Start[Index], Goal[Index]):
                    Point = Middle.copy()
                    Point[Index] = Value
                    Add([Np.clip(Point, Lower, Upper)])
        # A small parallel offset can get around an obstruction that blocks
        # every single-bend path (raise, traverse, return).
        for Index in range(4):
            for Offset in (-64, -32, -16, -8, -4, 4, 8, 16, 32, 64):
                Shift = Np.zeros(len(Names))
                Shift[Index] = Offset
                Add([Np.clip(Start + Shift, Lower, Upper), Np.clip(Goal + Shift, Lower, Upper)])
        Active = [Index for Index in range(len(Names)) if abs(Start[Index] - Goal[Index]) > 1e-6]
        for Choices in Itertools.product((False, True), repeat=len(Active)):
            Point = Start.copy()
            for Index, UseGoal in zip(Active, Choices):
                if UseGoal:
                    Point[Index] = Goal[Index]
            Add([Point])
        # Single-bend repairs certify far more useful routes per second here than
        # the sampling fallback does, so they get the larger half of the clock.
        LocalDeadline = SearchDeadline if LocalOnly else min(SearchDeadline, Time.monotonic() + Seconds * 0.55)
        for _, _, Path in sorted(Candidates, key=lambda Item: Item[:2]):
            if Time.monotonic() >= LocalDeadline:
                break
            if Clear(Path, LocalDeadline):
                return Finish(Path)

        if LocalOnly:
            raise ValueError("No small checked repair of the direct motion was found")

        # Keep ordered moves as a fallback, but compare their cost and optimize
        # a certified result rather than exposing each single-joint leg.
        Orders = []
        for Order in Itertools.permutations(Active):
            Path, Current = [Start], Start.copy()
            for Index in Order:
                Next = Current.copy()
                Next[Index] = Goal[Index]
                Path.append(Next)
                Current = Next
            Orders.append((Self.PathCost(Path, EdgeCost), Path))
        OrderDeadline = min(SearchDeadline, Time.monotonic() + Seconds * 0.15)
        for _, Path in sorted(Orders, key=lambda Item: Item[0]):
            if Time.monotonic() >= OrderDeadline:
                break
            if Clear(Path, OrderDeadline):
                return Finish(Path)
        # Bidirectional RRT. Each short leg is certified through its gates; the
        # result is shortened before it is expanded.
        Random = Np.random.default_rng(101)
        Trees = [([Start], [-1]), ([Goal], [-1])]

        def Extend(Tree, Target):
            Nodes, Parents = Tree
            Parent = int(Np.argmin(Np.linalg.norm(Np.array(Nodes) - Target, axis=1)))
            Delta = Target - Nodes[Parent]
            Step = Nodes[Parent] + Delta * min(1.0, 12 / max(1e-9, float(Np.max(Np.abs(Delta)))))
            Step = Np.clip(Step, Lower, Upper)
            # The independent-progress box is symmetric, so a goal-tree leg
            # certified toward the start also certifies the reverse motion.
            if Leg(Nodes[Parent], Step, SearchDeadline) is None:
                return None
            Nodes.append(Step)
            Parents.append(Parent)
            return len(Nodes) - 1

        def Trace(Tree, Index):
            Path = []
            while Index >= 0:
                Path.append(Tree[0][Index])
                Index = Tree[1][Index]
            return Path[::-1]

        for Iteration in range(1500):
            if Time.monotonic() >= SearchDeadline:
                break
            Side = Iteration % 2
            First, Second = Trees[Side], Trees[1 - Side]
            Sample = Second[0][0] if Iteration % 5 == 0 else Random.uniform(Lower, Upper)
            Index = Extend(First, Sample)
            if Index is None:
                continue
            Target = First[0][Index]
            for _ in range(40):
                if Time.monotonic() >= SearchDeadline:
                    break
                Other = Extend(Second, Target)
                if Other is None:
                    break
                if Np.max(Np.abs(Second[0][Other] - Target)) < 1e-7:
                    Left, Right = Trace(First, Index), Trace(Second, Other)
                    Path = Left + Right[-2::-1] if Side == 0 else Right + Left[-2::-1]
                    return Finish(Path)
        raise ValueError("No clear route around the basket and robot body was found. Choose a different target or wrist pitch.")
