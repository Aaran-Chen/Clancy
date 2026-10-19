"""SO-101 rigid-transform FK and bounded, exact-chain position/pitch IK.

Geometry is shared with the browser in CadModel.json. Joint identifiers retain
the URDF spelling. App angles are calibrated deltas, in degrees; FK uses cm.
"""
import json as Json
import math as Math
from pathlib import Path

import numpy as Np

Root = Path(__file__).resolve().parent
Model = Json.loads((Root / "CadModel.json").read_text())
JointNames = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
SolveNames = JointNames[:4]
SolverVersion = "all-poses-v2"
OrderedJoints = [next(Joint for Joint in Model["Joints"] if Joint["Name"] == Name)
                 for Name in (*JointNames, "gripper_frame_joint")]
Origins = {Joint["Name"]: Np.array(Joint["Origin"]) for Joint in OrderedJoints}
BaseTransform = Np.array(Model["CadToModel"])


def Wrap(Value):
    return (Value + Math.pi) % (2 * Math.pi) - Math.pi


def Rotate(Frame, Angle):
    """Frame @ Rz(Angle): the joint's motion. One small matmul beats np.eye plus slice assignment."""
    Cos, Sin = Math.cos(Angle), Math.sin(Angle)
    return Frame @ Np.array(((Cos, -Sin, 0, 0), (Sin, Cos, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1.0)))


def ForwardUrdf(Radians):
    Links = {Model["Root"]: BaseTransform.copy()}
    Frames = {}
    for Joint in OrderedJoints:
        Frame = Links[Joint["Parent"]] @ Origins[Joint["Name"]]
        Frames[Joint["Name"]] = Frame
        if Joint["Type"] != "fixed":
            Angle = float(Radians.get(Joint["Name"], 0))
            if not Math.isfinite(Angle):
                raise ValueError("CAD joint angles must be finite")
            Links[Joint["Child"]] = Rotate(Frame, Angle)
        else:
            Links[Joint["Child"]] = Frame
    return Links, Frames


# The tool point only depends on the serial chain pan..roll, then the fixed
# gripper frame; the jaw joint hangs off to the side. The IK inner loop walks
# just that chain instead of building every link.
ToolChain = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
assert [Joint["Parent"] for Joint in OrderedJoints if Joint["Name"] in ToolChain[1:]] == \
       [Joint["Child"] for Joint in OrderedJoints if Joint["Name"] in ToolChain[:-1]]
assert next(Joint for Joint in OrderedJoints if Joint["Name"] == "gripper_frame_joint")["Parent"] == \
       next(Joint for Joint in OrderedJoints if Joint["Name"] == "wrist_roll")["Child"]
assert next(Joint for Joint in OrderedJoints if Joint["Name"] == "gripper_frame_joint")["Child"] == Model["ToolLink"]
ChainOrigins = [BaseTransform @ Origins[ToolChain[0]], *(Origins[Name] for Name in ToolChain[1:])]
ToolOrigin = Origins["gripper_frame_joint"]


def ToolFrames(Values, Roll):
    """Joint frames (before their own rotation) for the four solve joints, and the tool transform."""
    Frames = []
    Link = None
    for Index, Origin in enumerate(ChainOrigins):
        Frame = Origin if Link is None else Link @ Origin
        if Index < 4:
            Frames.append(Frame)
        Link = Rotate(Frame, Values[Index] if Index < 4 else Roll)
    return Frames, Link @ ToolOrigin


def ToolPitch(Tool, Pan):
    Direction = Tool[:3, 2]
    return Math.atan2(float(Direction[2]), float(Direction[0] * Math.sin(Pan) + Direction[1] * Math.cos(Pan)))


def ToolJacobian(Frames, Tool, Pan):
    """d(tool xyz, tool pitch)/d(pan, lift, elbow, flex). Every joint turns about its frame's z axis."""
    Position, Direction = Tool[:3, 3], Tool[:3, 2]
    Sin, Cos = Math.sin(Pan), Math.cos(Pan)
    Radial = Direction[0] * Sin + Direction[1] * Cos
    Norm = Radial * Radial + Direction[2] * Direction[2]
    Axes = Np.array([Frame[:3, 2] for Frame in Frames])
    Arms = Position - Np.array([Frame[:3, 3] for Frame in Frames])
    # Row-wise cross products; Np.cross costs more than the rest of the solve on 3-vectors.
    Next, After = [1, 2, 0], [2, 0, 1]
    Jacobian = Np.empty((4, 4))
    Jacobian[:3] = (Axes[:, Next] * Arms[:, After] - Axes[:, After] * Arms[:, Next]).T
    Turn = Axes[:, Next] * Direction[After] - Axes[:, After] * Direction[Next]
    RadialRate = Turn[:, 0] * Sin + Turn[:, 1] * Cos
    RadialRate[0] += Direction[0] * Cos - Direction[1] * Sin
    Jacobian[3] = (Radial * Turn[:, 2] - Direction[2] * RadialRate) / Norm if Norm > 0 else 0.0
    return Jacobian


def ToUrdf(Angles, Alignment):
    return {Name: Math.radians(Alignment["UrdfAtReferenceDegrees"][Name]
                              + Alignment["Direction"][Name] * float(Angles.get(Name, 0))) for Name in JointNames}


def FromUrdf(Radians, Alignment):
    return {Name: (Math.degrees(Radians.get(Name, 0)) - Alignment["UrdfAtReferenceDegrees"][Name])
                  / Alignment["Direction"][Name] for Name in JointNames}


def Forward(Geometry, Angles):
    return ForwardUrdf(ToUrdf(Angles, Geometry["CadAlignment"]))


def Point(Vector):
    return dict(zip(("x", "y", "z"), map(float, Vector)))


def Positions(Geometry, Angles):
    Links, Frames = Forward(Geometry, Angles)
    return [Point(Frames[Name][:3, 3]) for Name in JointNames[:5]] + [Point(Links[Model["ToolLink"]][:3, 3])]


def PitchUrdf(Radians, Links=None):
    Links = Links if Links is not None else ForwardUrdf(Radians)[0]
    Direction = Links[Model["ToolLink"]][:3, 2]
    Pan = Radians.get("shoulder_pan", 0)
    Radial = Direction[0] * Math.sin(Pan) + Direction[1] * Math.cos(Pan)
    return Math.atan2(float(Direction[2]), float(Radial))


def BoundedLeastSquares(Function, Jacobian, Start, Lower, Upper, MaxEvaluations, Tolerance=1e-12):
    """Levenberg-Marquardt inside box bounds, for the 3-4 variable IK fits.

    SciPy's trust-region solver spent ~0.4 ms of bookkeeping per iteration on
    this problem, far more than the kinematics. Variables pinned at a bound whose
    gradient pushes outward are frozen for the step (an active set), the rest take
    a damped Gauss-Newton step that is clipped back inside the box. Like SciPy's
    ftol, a step that no longer shrinks the cost by a relative 1e-11 ends the fit:
    a seed on a branch the limits block otherwise crawls along a bound all budget.
    """
    Values = Np.clip(Start, Lower, Upper)
    Residual = Function(Values)
    Cost = float(Residual @ Residual)
    Damping = 1e-6
    Evaluations = 1
    while Evaluations < MaxEvaluations and Cost > Tolerance ** 2:
        Matrix = Jacobian(Values)
        Gradient = Matrix.T @ Residual
        Normal = Matrix.T @ Matrix
        Span = Upper - Lower
        Pinned = ((Values <= Lower + 1e-12 * Span) & (Gradient > 0)) | ((Values >= Upper - 1e-12 * Span) & (Gradient < 0))
        Free = ~Pinned
        if not Free.any() or float(Np.abs(Gradient[Free]).max()) < Tolerance ** 2:
            break
        Reduced = Normal[Np.ix_(Free, Free)]
        Diagonal = Np.maximum(Np.diag(Reduced), 1e-9)
        Improved = False
        while Evaluations < MaxEvaluations:
            Step = Np.zeros_like(Values)
            try:
                Step[Free] = Np.linalg.solve(Reduced + Damping * Np.diag(Diagonal), -Gradient[Free])
            except Np.linalg.LinAlgError:
                Damping *= 10
                continue
            Candidate = Np.clip(Values + Step, Lower, Upper)
            Moved = float(Np.abs(Candidate - Values).max())
            if Moved <= Tolerance * (1 + float(Np.abs(Values).max())):
                return Values, Residual
            Trial = Function(Candidate)
            Evaluations += 1
            TrialCost = float(Trial @ Trial)
            if TrialCost < Cost:
                if Cost - TrialCost < 1e-11 * Cost:
                    return Candidate, Trial
                Values, Residual, Cost = Candidate, Trial, TrialCost
                Damping = max(Damping / 5, 1e-12)
                Improved = True
                break
            Damping *= 4
            if Damping > 1e12:
                break
        if not Improved:
            break
    return Values, Residual


def Pitch(Geometry, Angles):
    return Math.degrees(PitchUrdf(ToUrdf(Angles, Geometry["CadAlignment"])))


def Branch(Geometry, Angles):
    Points = Positions(Geometry, Angles)
    Pan = ToUrdf(Angles, Geometry["CadAlignment"])["shoulder_pan"]
    def Elevation(Start, End):
        Delta = {Axis: End[Axis] - Start[Axis] for Axis in Start}
        return Math.atan2(Delta["z"], Delta["x"] * Math.sin(Pan) + Delta["y"] * Math.cos(Pan))
    return "up" if Wrap(Elevation(Points[2], Points[3]) - Elevation(Points[1], Points[2])) >= 0 else "down"


def Dimensions(Geometry):
    Points = Positions(Geometry, {})
    Upper = float(Np.linalg.norm(Origins["elbow_flex"][:3, 3]))
    Forearm = float(Np.linalg.norm(Origins["wrist_flex"][:3, 3]))
    Wrist = float(Np.linalg.norm(Origins["wrist_roll"][:3, 3]))
    Tip = float(Np.linalg.norm(Origins["gripper_frame_joint"][:3, 3]))
    def Elevation(Start, End):
        return Math.degrees(Math.atan2(End["z"] - Start["z"], End["y"] - Start["y"]))
    return {"base_radial_cm": float(abs(Origins["shoulder_lift"][0, 3])),
            "base_z_cm": Points[1]["z"], "upper_len_cm": Upper, "forearm_len_cm": Forearm,
            "wrist_len_cm": Wrist, "tip_len_cm": Tip, "lower_len_cm": Forearm + Wrist + Tip,
            "upper_zero_deg": Elevation(Points[1], Points[2]), "forearm_zero_deg": Elevation(Points[2], Points[3]),
            "lower_zero_deg": Elevation(Points[2], Points[3]), "wrist_zero_deg": Pitch(Geometry, {}),
            "max_reach_cm": float(Np.linalg.norm(Origins["shoulder_lift"][:3, 3])) + Upper + Forearm + Wrist + Tip,
            "max_z_cm": Points[1]["z"] + Upper + Forearm + Wrist + Tip}


def AnalyticSeeds(Target, PitchRadians, FixedRadians, Cosines=None):
    """Exact planar reduction supplies seeds; full URDF transforms refine them.

    The upstream Euler angles are rounded, so the pitch axes are only nearly
    parallel. The nonlinear refinement retains those small rotations exactly.
    Cosines, if given, receives each reduction's unclamped elbow cosine: beyond
    +-1 the planar arm cannot reach the target.
    """
    Zero = {**dict.fromkeys(JointNames, 0.0), **FixedRadians}
    Links, Frames = ForwardUrdf(Zero)
    Shoulder = Frames["shoulder_lift"][:3, 3]
    Elbow = Frames["elbow_flex"][:3, 3]
    Wrist = Frames["wrist_flex"][:3, 3]
    Tool = Links[Model["ToolLink"]][:3, 3]
    UpperVector, ForearmVector, Distal = Elbow - Shoulder, Wrist - Elbow, Tool - Wrist
    Upper = float(Np.linalg.norm(UpperVector[1:]))
    Forearm = float(Np.linalg.norm(ForearmVector[1:]))
    UpperZero = Math.atan2(UpperVector[2], UpperVector[1])
    ForearmZero = Math.atan2(ForearmVector[2], ForearmVector[1])
    ToolPitchZero = PitchUrdf(Zero, Links)
    DistalAngle = PitchRadians + Math.atan2(Distal[2], Distal[1]) - ToolPitchZero
    DistalLength = float(Np.linalg.norm(Distal[1:]))
    Radius = Math.hypot(Target[0], Target[1])
    Lateral = float(Tool[0])
    RadialMagnitude = Math.sqrt(max(0.0, Radius * Radius - Lateral * Lateral))
    Seeds = []
    for RadialSign in (1, -1):
        Radial = RadialSign * RadialMagnitude
        Pan = Math.atan2(Target[0], Target[1]) - Math.atan2(Lateral, Radial)
        R = Radial - Shoulder[1] - DistalLength * Math.cos(DistalAngle)
        Z = Target[2] - Shoulder[2] - DistalLength * Math.sin(DistalAngle)
        CosElbow = (R * R + Z * Z - Upper * Upper - Forearm * Forearm) / (2 * Upper * Forearm)
        if Cosines is not None:
            Cosines.append(CosElbow)
        CosElbow = min(1.0, max(-1.0, CosElbow))
        for Sign in (1, -1):
            Bend = Sign * Math.acos(CosElbow)
            Elevation = Math.atan2(Z, R) - Math.atan2(Forearm * Math.sin(Bend), Upper + Forearm * Math.cos(Bend))
            Lift = UpperZero - Elevation
            ElbowAngle = ForearmZero - UpperZero - Bend
            Flex = ToolPitchZero - PitchRadians - Lift - ElbowAngle
            Seeds.append(Np.array([Pan, Lift, ElbowAngle, Flex]))
    return Seeds


def SolveCandidates(Geometry, X, Y, Z, *, elbow="up", joint_limits=None, initial_angles=None,
          preferred_angles=None, wrist_pitch_deg=None, tolerance_cm=0.1,
          max_iterations=70, Quick=False, **Unused):
    """Reachable and unreachable joint solutions for a tool point and pitch, best first.

    Quick is for scanning many targets (calibration sampling): a target the
    planar reduction clearly cannot reach is returned unsolved, and the search
    stops at the first seed that converges on the preferred elbow branch.
    Forward-reaching seeds come first, so that is normally the same solution.
    """
    Alignment = Geometry["CadAlignment"]
    Initial = ToUrdf(initial_angles or {}, Alignment)
    Preferred = ToUrdf(preferred_angles or initial_angles or {}, Alignment)
    Fixed = {Name: Initial[Name] for Name in JointNames[4:]}
    Target = Np.array([X, Y, Z], dtype=float)
    PitchTarget = Math.radians(float(wrist_pitch_deg)) if wrist_pitch_deg is not None else PitchUrdf(Initial)
    if not Np.all(Np.isfinite(Target)) or not Math.isfinite(PitchTarget):
        raise ValueError("IK target and pitch must be finite")
    Lower, Upper = [], []
    for Name in SolveNames:
        Bounds = (joint_limits or {}).get(Name, (-180.0, 180.0))
        Values = [Math.radians(Alignment["UrdfAtReferenceDegrees"][Name] + Alignment["Direction"][Name] * Value) for Value in Bounds]
        Lower.append(min(Values))
        Upper.append(max(Values))
    Lower, Upper = Np.array(Lower), Np.array(Upper)
    if not Np.all(Np.isfinite([Lower, Upper])):
        raise ValueError("CAD IK joint ranges must be finite")
    Active = Np.flatnonzero(Upper - Lower > 1e-12)

    Roll = Fixed["wrist_roll"]
    if not Math.isfinite(Roll):
        raise ValueError("CAD joint angles must be finite")

    def Evaluate(Values):
        _, Tool = ToolFrames(Values, Roll)
        return Np.append(Tool[:3, 3] - Target, Wrap(ToolPitch(Tool, Values[0]) - PitchTarget) * 10)

    def EvaluateJacobian(Values):
        Frames, Tool = ToolFrames(Values, Roll)
        Jacobian = ToolJacobian(Frames, Tool, Values[0])
        Jacobian[3] *= 10
        return Jacobian

    def Reach(Values):
        """Tip offset alone, for the nearest-tip pass on an unreachable target."""
        return ToolFrames(Values, Roll)[1][:3, 3] - Target

    def ReachJacobian(Values):
        Frames, Tool = ToolFrames(Values, Roll)
        return ToolJacobian(Frames, Tool, Values[0])[:3]

    def Expand(Values, Variables):
        Result = Values.copy()
        Result[Active] = Variables
        return Result

    def Describe(Values, Residual):
        Radians = {**Fixed, **dict(zip(SolveNames, Values))}
        Angles = FromUrdf(Radians, Alignment)
        Error = float(Np.linalg.norm(Residual[:3]))
        PitchError = abs(Math.degrees(Residual[3] / 10))
        Reachable = Error <= min(tolerance_cm, 0.1) and PitchError <= 0.25
        Converged = Error <= 1e-5 and PitchError <= 1e-5
        ActualBranch = Branch(Geometry, Angles)
        Travel = sum((Radians[Name] - Preferred[Name]) ** 2 for Name in SolveNames)
        Score = (0 if Converged else 1 if Reachable else 2, 0 if Reachable and ActualBranch == elbow else 1,
                 Travel if Reachable else Error + PitchError * 0.1)
        return Score, {"reachable": Reachable, "joint_angles_degrees": Angles, "error_cm": Error,
                       "wrist_pitch_error_deg": PitchError, "wrist_pitch_degrees": Pitch(Geometry, Angles),
                       "elbow": ActualBranch, "tip_cm": Positions(Geometry, Angles)[-1]}

    Cosines = []
    Seeds = AnalyticSeeds(Target, PitchTarget, Fixed, Cosines)
    Seeds.append(Np.array([Initial[Name] for Name in SolveNames]))
    Candidates = []
    Used = set()
    if Quick and min(abs(Value) for Value in Cosines) > 1.05:
        Angles = FromUrdf({**Fixed, **Initial}, Alignment)
        return [{"reachable": False, "joint_angles_degrees": Angles, "error_cm": Math.inf, "wrist_pitch_error_deg": Math.inf,
                 "wrist_pitch_degrees": Pitch(Geometry, Angles), "elbow": Branch(Geometry, Angles),
                 "tip_cm": Positions(Geometry, Angles)[-1]}]
    for Seed in Seeds:
        for Index, Name in enumerate(SolveNames):
            Equivalents = [Seed[Index] + Turn * 2 * Math.pi for Turn in range(-3, 4)]
            Inside = [Value for Value in Equivalents if Lower[Index] <= Value <= Upper[Index]]
            Seed[Index] = min(Inside or Equivalents, key=lambda Value: abs(Value - Initial[Name]))
        Inset = Np.minimum(1e-10, (Upper - Lower) * .1)
        Seed = Np.clip(Seed, Lower + Inset, Upper - Inset)
        Key = tuple(Np.round(Seed, 6))
        if Key in Used:
            continue
        Used.add(Key)
        Residual = Evaluate(Seed)
        if Np.linalg.norm(Residual) > 1e-6 and len(Active):
            Fit, Residual = BoundedLeastSquares(lambda Variables: Evaluate(Expand(Seed, Variables)),
                                                lambda Variables: EvaluateJacobian(Expand(Seed, Variables))[:, Active],
                                                Seed[Active], Lower[Active], Upper[Active],
                                                max(20, min(100, max_iterations * 2)))
            Values = Expand(Seed, Fit)
        else:
            Values = Seed
        Score, Record = Describe(Values, Residual)
        Candidates.append((Score, Record))
        if Quick and Score[0] == 0 and Record["elbow"] == elbow:
            break
    Candidates.sort(key=lambda Item: Item[0])
    Results = [Result for _, Result in Candidates]
    # Out of reach, answer with the pose whose TIP is closest to the request.
    # The solve above is free to give up position to hold the wrist pitch, so a
    # target dragged toward the arm could make the shown pose retreat from it:
    # to the operator the preview looked like it was refusing to follow. The
    # pitch has already been chosen (the closest reachable one) before this call.
    if Results and not Results[0]["reachable"] and not Quick and len(Active):
        Best = None
        for Result in Results[:3]:
            Start = Np.array([ToUrdf(Result["joint_angles_degrees"], Alignment)[Name] for Name in SolveNames])
            Fit, _ = BoundedLeastSquares(lambda Variables: Reach(Expand(Start, Variables)),
                                         lambda Variables: ReachJacobian(Expand(Start, Variables))[:, Active],
                                         Start[Active], Lower[Active], Upper[Active], 40, 1e-10)
            Values = Expand(Start, Fit)
            Score, Record = Describe(Values, Evaluate(Values))
            if Best is None or Record["error_cm"] < Best[1]["error_cm"] - 1e-9:
                Best = (Score, Record)
        if Best and Best[1]["error_cm"] < Results[0]["error_cm"] - 1e-9:
            Results.insert(0, Best[1])
    return Results


def Solve(Geometry, X, Y, Z, *, PoseAllowed=None, **Options):
    Candidates = SolveCandidates(Geometry, X, Y, Z, **Options)
    # Branch and travel are preferences. A colliding preferred solution must
    # not hide another reachable configuration at exactly the same XYZ/pitch.
    if PoseAllowed is not None:
        for Candidate in Candidates:
            if Candidate["reachable"] and PoseAllowed(Candidate["joint_angles_degrees"]):
                return Candidate
    return Candidates[0]
