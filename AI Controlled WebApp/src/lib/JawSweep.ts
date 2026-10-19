import Parts from "./GripperParts.json";
import { CadModel } from "./CadKinematics";

// THE JAW-PLANE SWEEP (grasp milestone M3). The moving jaw swings about its
// hinge in the gripper's x-z plane (gripper_link: x from the fixed pad into
// the gap, z along the fingers, the tips at -z). An object's slice in that
// plane is slid against the fixed finger (the moving jaw pushes it there),
// then the jaw closes on it from its open angle until it first touches. Two
// measures of that contact decide the grasp (CloseOn):
//  - PushDeg: how far the moving jaw's push leans from head-on (-x) toward
//    the finger tips (the design's "lean"). With only that contact's friction
//    holding it (mu >= tan 20 = 0.36) it may be at most MaxLeanDeg. The exact
//    CAD outlines give a 6 cm can 16.4 / 15.4 / 28.1 / 36.9 deg at wrist
//    0 / -15 / -30 / -45 (the design's CAD sweep: 16.3 / 15.3 / 28 / 37).
//  - GripDeg: the friction angle the two-pad grip needs, both pads gripping:
//    the line between the two contacts against each push (an antipodal grasp;
//    a round slice: half the angle between the pushes). Within MaxLeanDeg it
//    holds even where the push alone would not.
// A box's top edge under the jaw gives a steep push, a can's side near level
// a shallow one. Replaces the JawWindow table for objects the table refuses.
// Push (the planner's mode): the object does not slide straight to the fixed
// finger. The moving jaw meets it where it stands and pushes it there, and a
// push that leans toward the tips carries it along the fingers on the way
// (the 6.6 cm can about 0.7 cm, a 5 cm one about 1 cm, from a flat wrist):
// CloseOn moves it the way the jaw drives it (PushAlong) and judges the grip
// where it ends. The planner starts the object that much nearer the palm.
// 2D in the jaws' plane; GripIn3D adds what the plane leaves out (the jaws
// tilted against a standing object's side).

// The closing simulation's work so far: vertex-edge pairs its exact polygon tests went through (Overlaps, ClosestPoints,
// SlideToward, FirstTouchDeg), counted per call. GraspPlan's best-effort pass budgets its search by work, never by the clock.
export const SweepWork = { Pairs: 0 };

export const SweepRules = {
  // The most friction angle a grip may need at a pad (mu >= tan 20 = 0.36). The design's
  // rule held the moving jaw's push alone to it; the planner holds the two-pad grip.
  MaxLeanDeg: 20,
  // The object must stop the jaw at least this far above where the pads meet.
  StopAboveMeetDeg: 2,
  // CAD angle where the pads meet (GraspRules.PadsMeetDeg).
  PadsMeetDeg: -11.41,
  // Gripper fractions the open jaw may stand at, smallest first; the open jaw clears the slice by this much.
  OpenFractions: [0.5, 0.6, 0.8],
  OpenClearanceCm: 0.5,
  // The real jaw sits about this much more closed than the CAD's (report C 2f): the open test checks both.
  JawUncertaintyDeg: 11,
  // Push: how far the object moves per step while the jaw drives it to the fixed finger, cm
  // (0.3 moves the 16:49 can 0.04 cm farther than 0.05 does: a little conservative).
  PushStepCm: 0.25,
  // Half the fingers' thickness across the jaws' plane (the fixed pad: gripper y -0.53..0.49).
  FingerHalfThickCm: 0.5,
};

export type P2 = { x: number; z: number };

// The gripper joint: the moving jaw's link frame in gripper_link is R0 Rz(theta) + T0,
// the hinge at gripper (x, z) (2.02, -2.34), its axis -y.
const Joint = CadModel.Joints.find(Item => Item.Name === "gripper")!;
const R0 = Joint.Origin.slice(0, 3).map(Row => Row.slice(0, 3));
const T0 = Joint.Origin.slice(0, 3).map(Row => Row[3]);
// The jaw swings in the gripper's x-z plane only if its axis is the gripper's y (the hinge).
if (Math.abs(R0[0][2]) > 1e-3 || Math.abs(R0[2][2]) > 1e-3) throw new Error("JawSweep: the gripper hinge is no longer along gripper_link y");

const Cross = (O: P2, A: P2, B: P2) => (A.x - O.x) * (B.z - O.z) - (A.z - O.z) * (B.x - O.x);
/** Convex hull (Andrew's monotone chain), counter-clockwise in (x, z). */
export function ConvexHull2(Points: P2[]): P2[] {
  const Sorted = Points.slice().sort((A, B) => A.x - B.x || A.z - B.z);
  if (Sorted.length < 3) return Sorted;
  const Lower: P2[] = [], Upper: P2[] = [];
  for (const P of Sorted) {
    while (Lower.length >= 2 && Cross(Lower[Lower.length - 2], Lower[Lower.length - 1], P) <= 1e-12) Lower.pop();
    Lower.push(P);
  }
  for (let Index = Sorted.length - 1; Index >= 0; Index--) {
    const P = Sorted[Index];
    while (Upper.length >= 2 && Cross(Upper[Upper.length - 2], Upper[Upper.length - 1], P) <= 1e-12) Upper.pop();
    Upper.push(P);
  }
  return [...Lower.slice(0, -1), ...Upper.slice(0, -1)];
}

// The moving jaw's outline in its own link frame (x, y), and the fixed
// finger's in gripper (x, z): their exact outlines from the GLB
// (BuildGripperParts.py Silhouettes), not convex: the moving jaw's inner face
// steps back 0.2 and 0.4 cm above its pad and a can touches those steps (the
// design's 16.3 deg push for a 6 cm can at a level wrist is that contact).
// The palm: the hull of its samples (it only stops an object). The fixed finger's
// outline holds the pad the tool point sits on, x -0.79 at z -10.35..-9.45
// (BuildGripperParts.py checks it).
const JawLocal = Parts.Silhouettes.MovingJaw.map(([X, Y]) => ({ x: X, z: Y }));
// How far the jaw reaches from its hinge: bounds a step of the closing search.
const JawRadius = Math.max(...JawLocal.map(P => Math.hypot(P.x, P.z))) + 1e-9;
export const FixedFingerPolygon = Parts.Silhouettes.FixedFinger.map(([X, Z]) => ({ x: X, z: Z }));
export const PalmPolygon = ConvexHull2(Parts.Parts.PalmAndServo.Samples.map(([X, , Z]) => ({ x: X, z: Z })));

/** The moving jaw's silhouette in gripper (x, z) at a CAD gripper angle (deg). */
export function JawPolygon(Deg: number): P2[] {
  return JawLocal.map(P => JawToGripper(P, Deg));
}
// A point of the jaw (its link frame x, y) in gripper (x, z) at an angle, and back.
function JawToGripper(P: P2, Deg: number): P2 {
  const C = Math.cos(Deg * Math.PI / 180), S = Math.sin(Deg * Math.PI / 180);
  const X = C * P.x - S * P.z, Y = S * P.x + C * P.z;
  return { x: R0[0][0] * X + R0[0][1] * Y + T0[0], z: R0[2][0] * X + R0[2][1] * Y + T0[2] };
}
function GripperToJaw(P: P2, Deg: number): P2 {
  // R0's x-z block is orthonormal (the hinge is along y): its transpose inverts it.
  const Dx = P.x - T0[0], Dz = P.z - T0[2];
  const X = R0[0][0] * Dx + R0[2][0] * Dz, Y = R0[0][1] * Dx + R0[2][1] * Dz;
  const C = Math.cos(Deg * Math.PI / 180), S = Math.sin(Deg * Math.PI / 180);
  return { x: C * X + S * Y, z: -S * X + C * Y };
}
// Which way a point of the jaw moves as it closes (the angle falls), unit.
function JawVelocity(Point: P2, Deg: number): P2 {
  const Local = GripperToJaw(Point, Deg), Next = JawToGripper(Local, Deg - 0.01);
  const Dx = Next.x - Point.x, Dz = Next.z - Point.z, L = Math.hypot(Dx, Dz) || 1;
  return { x: Dx / L, z: Dz / L };
}

// --- Polygon geometry (counter-clockwise: hulls, and the gripper's outlines) ----------------

/**
 * Whether two polygons overlap (separating axes; touching within 1e-9 counts as
 * apart): exact for convex ones, and for the gripper's outlines it tests their
 * hulls (an object in a jaw's step counts as overlapping: conservative).
 */
export function Overlaps(A: P2[], B: P2[]): boolean {
  SweepWork.Pairs += (A.length + B.length) * (A.length + B.length);
  for (const Polygon of [A, B]) {
    for (let Index = 0; Index < Polygon.length; Index++) {
      const P = Polygon[Index], Q = Polygon[(Index + 1) % Polygon.length];
      const Nx = -(Q.z - P.z), Nz = Q.x - P.x;
      let MinA = Infinity, MaxA = -Infinity, MinB = Infinity, MaxB = -Infinity;
      for (const V of A) { const D = V.x * Nx + V.z * Nz; MinA = Math.min(MinA, D); MaxA = Math.max(MaxA, D); }
      for (const V of B) { const D = V.x * Nx + V.z * Nz; MinB = Math.min(MinB, D); MaxB = Math.max(MaxB, D); }
      const Scale = Math.hypot(Nx, Nz) || 1;
      if (MaxA < MinB + 1e-9 * Scale || MaxB < MinA + 1e-9 * Scale) return false;
    }
  }
  return true;
}

type Closest = { Distance: number; OnA: P2; OnB: P2; AEdge: boolean; BEdge: boolean };
// The closest points of two disjoint polygons (a corner of one on a side of the other: exact for any
// simple polygons), and whether each lies inside an edge (not on a vertex).
function ClosestPoints(A: P2[], B: P2[]): Closest {
  SweepWork.Pairs += 2 * A.length * B.length;
  let Best: Closest = { Distance: Infinity, OnA: A[0], OnB: B[0], AEdge: false, BEdge: false };
  const Box = (Polygon: P2[]) => {
    let LoX = Infinity, HiX = -Infinity, LoZ = Infinity, HiZ = -Infinity;
    for (const P of Polygon) { LoX = Math.min(LoX, P.x); HiX = Math.max(HiX, P.x); LoZ = Math.min(LoZ, P.z); HiZ = Math.max(HiZ, P.z); }
    return { LoX, HiX, LoZ, HiZ };
  };
  const Boxes = [Box(A), Box(B)];
  const Try = (From: P2[], To: P2[], Swap: boolean) => {
    const Bounds = Boxes[Swap ? 0 : 1];
    for (const V of From) {
      // A vertex farther from the other polygon's box than the best so far cannot do better.
      const Bx = Math.max(Bounds.LoX - V.x, 0, V.x - Bounds.HiX), Bz = Math.max(Bounds.LoZ - V.z, 0, V.z - Bounds.HiZ);
      if (Bx * Bx + Bz * Bz >= Best.Distance * Best.Distance) continue;
      for (let Index = 0; Index < To.length; Index++) {
        const P = To[Index], Q = To[(Index + 1) % To.length];
        const Ex = Q.x - P.x, Ez = Q.z - P.z, L2 = Ex * Ex + Ez * Ez;
        const T = L2 > 0 ? Math.max(0, Math.min(1, ((V.x - P.x) * Ex + (V.z - P.z) * Ez) / L2)) : 0;
        const On = { x: P.x + T * Ex, z: P.z + T * Ez };
        const Distance = Math.hypot(V.x - On.x, V.z - On.z);
        if (Distance < Best.Distance - 1e-12) {
          const Interior = T > 1e-6 && T < 1 - 1e-6;
          Best = Swap ? { Distance, OnA: On, OnB: V, AEdge: Interior, BEdge: false } : { Distance, OnA: V, OnB: On, AEdge: false, BEdge: Interior };
        }
      }
    }
  };
  Try(A, B, false);
  Try(B, A, true);
  return Best;
}
/** The gap between two polygons (0 when they, or for an outline its hull, overlap). */
export function Gap(A: P2[], B: P2[]): number {
  return Overlaps(A, B) ? 0 : ClosestPoints(A, B).Distance;
}

// How far B, moved along -x, travels before it touches A (Infinity: it never does).
function SlideToward(A: P2[], B: P2[]): number {
  SweepWork.Pairs += 2 * A.length * B.length;
  let Best = Infinity;
  // Rays from From's vertices along Direction (+1: +x, -1: -x) onto To's edges.
  const Cast = (From: P2[], To: P2[], Direction: number) => {
    for (const V of From) {
      for (let Index = 0; Index < To.length; Index++) {
        const P = To[Index], Q = To[(Index + 1) % To.length];
        if ((V.z < Math.min(P.z, Q.z) - 1e-12) || (V.z > Math.max(P.z, Q.z) + 1e-12)) continue;
        const Xs = Math.abs(Q.z - P.z) < 1e-12 ? [P.x, Q.x] : [P.x + (V.z - P.z) / (Q.z - P.z) * (Q.x - P.x)];
        for (const X of Xs) { const T = (X - V.x) * Direction; if (T >= -1e-9) Best = Math.min(Best, Math.max(T, 0)); }
      }
    }
  };
  Cast(B, A, -1);
  Cast(A, B, 1);
  return Best;
}
// How far B, moved along the unit direction D, travels before it touches A: the
// plane turned so D is -x (a rotation, so distances keep).
function SlideAlong(A: P2[], B: P2[], D: P2): number {
  const Turn_ = (P: P2) => ({ x: -D.x * P.x - D.z * P.z, z: D.z * P.x - D.x * P.z });
  return SlideToward(A.map(Turn_), B.map(Turn_));
}
const Moved = (Polygon: P2[], D: P2, Cm: number) => Polygon.map(P => ({ x: P.x + Cm * D.x, z: P.z + Cm * D.z }));
// The hinge in gripper (x, z), and which way the jaw turns there as it closes (+1: counter-clockwise in x-z).
const Hinge: P2 = { x: T0[0], z: T0[2] };
const CloseSense = (() => {
  const P = JawToGripper({ x: 0, z: -5 }, 30), Q = JawToGripper({ x: 0, z: -5 }, 29.99);
  return Math.sign((P.x - Hinge.x) * (Q.z - Hinge.z) - (P.z - Hinge.z) * (Q.x - Hinge.x));
})();
// The fixed finger's tip along the fingers (gripper z): the pad ends at -10.35, the finger at -10.43.
export const FixedFingertipZ = Math.min(...FixedFingerPolygon.map(P => P.z));
// The moving jaw's hinge (gripper x, z) and its reach from it: its tip, 8.25 cm out.
export const MovingJawHinge: P2 = Hinge;
export const MovingJawReachCm = JawRadius;
// The palm's face between the fingers (gripper z): an object deeper than this is in the palm.
export const PalmFaceZ = Math.min(...PalmPolygon.map(P => P.z));
/**
 * THE STRADDLE: how far each jaw reaches past an object's centre in the jaws'
 * plane (cm), the centre the middle of its slice's extent. The fixed finger:
 * its tip (FixedFingertipZ) beyond the centre along the fingers. The moving
 * jaw: its reach from the hinge beyond the centre, so as it swings closed its
 * face crosses the centre that far short of its tip. Both positive: the object
 * stands between the jaws with each reaching past its middle, so the jaws
 * surround it before they close. 2026-10-03 14:50 "Grab the can": the plan put
 * the can's centre at the fixed pad's middle (0.6 cm inside the finger's tip,
 * 1.4 with its head start) and the pad 3.1 cm from it: 1.35 and 0.7 cm, and the
 * moving jaw met the can near its own tip and pushed it out past the fingertips.
 */
export function StraddleOf(Slice: P2[]): { FixedCm: number; MovingCm: number; Centre: P2 } {
  const Xs = Slice.map(P => P.x), Zs = Slice.map(P => P.z);
  const Centre = { x: (Math.min(...Xs) + Math.max(...Xs)) / 2, z: (Math.min(...Zs) + Math.max(...Zs)) / 2 };
  return { FixedCm: Centre.z - FixedFingertipZ, MovingCm: JawRadius - Math.hypot(Centre.x - Hinge.x, Centre.z - Hinge.z), Centre };
}
// The turn (rad, 0..2 pi) from direction From to To about the hinge, the way the jaw closes.
function TurnTo(From: P2, To: P2): number {
  const Angle = CloseSense * Math.atan2(From.x * To.z - From.z * To.x, From.x * To.x + From.z * To.z);
  return Angle < -1e-12 ? Angle + 2 * Math.PI : Math.max(Angle, 0);
}
// Where a circle about the hinge of radius R crosses the segment A-B (0, 1 or 2 points).
function CircleCrossings(R: number, A: P2, B: P2): P2[] {
  const Dx = B.x - A.x, Dz = B.z - A.z, Fx = A.x - Hinge.x, Fz = A.z - Hinge.z;
  const Qa = Dx * Dx + Dz * Dz, Qb = 2 * (Fx * Dx + Fz * Dz), Qc = Fx * Fx + Fz * Fz - R * R;
  const Disc_ = Qb * Qb - 4 * Qa * Qc;
  if (Qa < 1e-18 || Disc_ < 0) return [];
  const Root = Math.sqrt(Disc_), Out: P2[] = [];
  for (const T of [(-Qb - Root) / (2 * Qa), (-Qb + Root) / (2 * Qa)]) if (T >= -1e-9 && T <= 1 + 1e-9) Out.push({ x: A.x + T * Dx, z: A.z + T * Dz });
  return Out;
}
/**
 * The angle (CAD deg) at which the jaw, closing from Deg, first touches the
 * object: exact for polygons, where a jaw corner's arc about the hinge meets
 * an object side or an object corner's arc (the jaw turning past it) meets a
 * jaw side. Null: not before the pads meet.
 */
function FirstTouchDeg(Object_: P2[], Deg: number, Rules: typeof SweepRules): number | null {
  const Jaw = JawPolygon(Deg);
  SweepWork.Pairs += 2 * Jaw.length * Object_.length;
  const Limit = (Deg - Rules.PadsMeetDeg) * Math.PI / 180;
  let Best = Limit + 1e-9;
  const Reach = (Points: P2[], Sides: P2[], Moving: boolean) => {
    for (const V of Points) {
      const Vx = V.x - Hinge.x, Vz = V.z - Hinge.z, R = Math.hypot(Vx, Vz);
      for (let Index = 0; Index < Sides.length; Index++) {
        const A = Sides[Index], B = Sides[(Index + 1) % Sides.length];
        // Quick reject: the circle misses the segment's span of distances from the hinge.
        const Ra = Math.hypot(A.x - Hinge.x, A.z - Hinge.z), Rb = Math.hypot(B.x - Hinge.x, B.z - Hinge.z);
        if (R > Math.max(Ra, Rb) + 1e-9) continue;
        for (const X of CircleCrossings(R, A, B)) {
          const Xr = { x: X.x - Hinge.x, z: X.z - Hinge.z };
          // A jaw corner turns from V to X; an object corner meets a jaw side that turned from X to V.
          const Turn_ = Moving ? TurnTo({ x: Vx, z: Vz }, Xr) : TurnTo(Xr, { x: Vx, z: Vz });
          if (Turn_ < Best) Best = Turn_;
        }
      }
    }
  };
  Reach(Jaw, Object_, true);
  Reach(Object_, Jaw, false);
  return Best <= Limit ? Deg - Best * 180 / Math.PI : null;
}
// Close the jaw from Deg until it touches the object. Null: the pads meet first.
function CloseTo(Object_: P2[], Deg: number, Rules: typeof SweepRules): { Deg: number; Near: Closest } | null {
  const Touch = FirstTouchDeg(Object_, Deg, Rules);
  if (Touch == null) return null;
  // Measured a hair before the touch, so the closest points are the contact's.
  const At = Math.min(Deg, Touch + 1e-5);
  return { Deg: At, Near: ClosestPoints(JawPolygon(At), Object_) };
}
/**
 * Where the object ends when the closing jaw drives it to the fixed finger (or
 * the palm), from where it stands. Quasi-static on the desk: at each step it
 * moves the way the jaw's contact drives it, its push turned toward the jaw
 * point's motion by at most the friction angle (MaxLeanDeg: the jaw grips it
 * that well), then the jaw catches up. A push that leans toward the tips
 * carries it along the fingers (DriftCm, + toward the tips). FirstPushDeg:
 * the lean where the jaw first touches it. Null: the jaw closes past it.
 */
function PushAlong(Slice: P2[], OpenDeg: number, Rules: typeof SweepRules) {
  const Stops = [FixedFingerPolygon, PalmPolygon];
  let Object_ = Slice, Contact = CloseTo(Object_, OpenDeg, Rules);
  if (!Contact) return null;
  const FirstDeg = Contact.Deg, FirstPush = TouchOf(JawPolygon(Contact.Deg), Object_, Contact.Near).Push;
  const Friction = Rules.MaxLeanDeg * Math.PI / 180;
  let Travel = 0;
  for (let Step = 0; Step < 200; Step++) {
    const Jaw = JawPolygon(Contact.Deg);
    const N = TouchOf(Jaw, Object_, Contact.Near).Push, V = JawVelocity(Contact.Near.OnA, Contact.Deg);
    // The object goes the way the jaw point goes if friction holds it there, else along the cone's edge.
    const Off = Math.atan2(N.x * V.z - N.z * V.x, N.x * V.x + N.z * V.z), Turn_ = Math.sign(Off) * Math.min(Math.abs(Off), Friction);
    const D = { x: N.x * Math.cos(Turn_) - N.z * Math.sin(Turn_), z: N.x * Math.sin(Turn_) + N.z * Math.cos(Turn_) };
    const ToStop = Math.min(...Stops.map(Stop => SlideAlong(Stop, Object_, D)));
    // A ledge of the jaw itself in the way (measured from just off the contact).
    const ToJaw = SlideAlong(Jaw, Moved(Object_, D, 1e-3), D) + 1e-3;
    const Cm = Math.min(ToStop, ToJaw, Rules.PushStepCm);
    Object_ = Moved(Object_, D, Math.max(Cm - (Cm === ToStop ? 0 : 1e-4), 0));
    Travel += Cm;
    if (Cm >= ToStop || ToJaw <= Rules.PushStepCm || Travel > 6) break;
    const Next = CloseTo(Object_, Contact.Deg, Rules);
    if (!Next) return null;
    Contact = Next;
  }
  const Against = Object_;
  return { Against, Travel, DriftCm: Slice[0].z - Against[0].z, FirstDeg, FirstPushDeg: Math.atan2(-FirstPush.z, -FirstPush.x) * 180 / Math.PI };
}

// --- The closing test ------------------------------------------------------------------------

export type CloseCode = "held" | "slips" | "finger" | "jaw" | "free";
export type CloseResult = { Ok: boolean; Code: CloseCode;
  // Where the jaw first touches the object (CAD deg); its push's lean toward
  // the tips (PushDeg) and the two-pad grip's friction angle (GripDeg), both
  // signed: negative pushes the object toward the palm.
  ContactDeg: number | null; PushDeg: number | null; GripDeg: number | null;
  // How far the object slides from its planned place to the fixed finger before the jaw reaches it.
  SlideCm: number; Why: string;
  // Push mode: how far the jaw carried it toward the finger tips on the way (+), and how its first touch leaned.
  DriftCm?: number; FirstPushDeg?: number;
  // Where the two contacts are (gripper x, z: points along each) and their pushes into the object.
  Contacts?: { Jaw: { Patch: P2[]; Push: P2 }; Fixed: { Patch: P2[]; Push: P2 } } };
export type CloseMode = "slide" | "first" | "push";

/**
 * Close the jaw from OpenDeg on an object slice (gripper x, z) and judge the
 * grip where it ends. Mode "slide": slid straight along -x against the fixed
 * finger (or the palm) first, the design's model. "push": the jaw meets it
 * where it stands and drives it there (PushAlong), so a push that leans toward
 * the tips carries it along the fingers: the planner's. "first": the jaw's
 * first touch where it stands (the design's "first contact" column), no grip
 * (GripDeg null). Conservative advancement: the jaw's farthest point moves at
 * most JawRadius x the angle step, so a step of Gap / JawRadius never skips a
 * contact. Ok: it stops the jaw StopAboveMeetDeg above the pads' meeting angle
 * (the caller judges PushDeg and GripDeg against MaxLeanDeg). A boolean Mode
 * is the older call: true "slide", false "first".
 */
export function CloseOn(Slice: P2[], OpenDeg: number, Rules = SweepRules, Mode: CloseMode | boolean = "slide", Checked = true): CloseResult {
  const Kind: CloseMode = Mode === true ? "slide" : Mode === false ? "first" : Mode;
  const Fail = (Code: CloseCode, Why: string, Extra: Partial<CloseResult> = {}): CloseResult =>
    ({ Ok: false, Code, ContactDeg: null, PushDeg: null, GripDeg: null, SlideCm: 0, Why, ...Extra });
  if (Slice.length < 3) return Fail("free", "it does not cross the jaws' plane");
  // Unchecked (dense sampling of a slice already checked nearby): the open parts not tested against it.
  if (Checked && (Overlaps(FixedFingerPolygon, Slice) || Overlaps(PalmPolygon, Slice))) return Fail("finger", "the fixed finger or the palm would already be in it");
  if (Checked && Overlaps(JawPolygon(OpenDeg), Slice)) return Fail("jaw", "the open moving jaw would already be in it");
  const Closes = "the jaws would close past it (the moving jaw does not meet it before the pads meet)";
  let Against: P2[], Short: P2[], Slide: number, Extra: Partial<CloseResult> = {};
  if (Kind === "push") {
    const Pushed = PushAlong(Slice, OpenDeg, Rules);
    if (!Pushed) return Fail("slips", Closes);
    Against = Pushed.Against;
    Slide = Pushed.Travel;
    Extra = { DriftCm: Pushed.DriftCm, FirstPushDeg: Pushed.FirstPushDeg };
    // The stop's touch is measured 1e-3 cm short of it, back the way the object came.
    const Dx = Slice[0].x - Against[0].x, Dz = Slice[0].z - Against[0].z, L = Math.hypot(Dx, Dz) || 1;
    Short = Moved(Against, { x: Dx / L, z: Dz / L }, 1e-3);
  } else {
    Slide = Kind === "slide" ? Math.min(SlideToward(FixedFingerPolygon, Slice), SlideToward(PalmPolygon, Slice)) : 0;
    if (!Number.isFinite(Slide)) return Fail("slips", "it would slide past the fixed finger's tip instead of against it");
    Against = Slice.map(P => ({ x: P.x - Slide, z: P.z }));
    Short = Slice.map(P => ({ x: P.x - Slide + 1e-3, z: P.z }));
    if (Checked && Overlaps(JawPolygon(OpenDeg), Against)) return Fail("jaw", "the open moving jaw would already be in it", { SlideCm: Slide });
  }
  // Each step turns the jaw less than the gap, so it never passes into the object: no overlap test needed.
  const Touch = CloseTo(Against, OpenDeg, Rules);
  if (!Touch || Touch.Deg < Rules.PadsMeetDeg + Rules.StopAboveMeetDeg) return Fail("slips", Closes, { SlideCm: Slide, ...Extra });
  const Deg = Touch.Deg;
  // The two contacts: the moving jaw's, and the fixed finger's (or the palm's)
  // where the object came against it (measured 1e-3 cm short of touching).
  const JawTouch = TouchOf(JawPolygon(Deg), Against, Touch.Near);
  if (Kind === "first") return { Ok: true, Code: "held", ContactDeg: Deg, SlideCm: 0, Why: "", PushDeg: Math.atan2(-JawTouch.Push.z, -JawTouch.Push.x) * 180 / Math.PI, GripDeg: null };
  const Stop = [FixedFingerPolygon, PalmPolygon].map(Polygon => ({ Polygon, Near: ClosestPoints(Polygon, Short) }))
    .reduce((A, B) => (B.Near.Distance < A.Near.Distance ? B : A));
  const FixedTouch = TouchOf(Stop.Polygon, Short, Stop.Near);
  const Jaw = JawTouch.Push, Fixed = FixedTouch.Push;
  // The two pushes squeeze the object toward the tips (or the palm) unless
  // friction holds it. Both pads gripping (mu at each), the line between the
  // two contacts must lie inside both friction cones (an antipodal grasp):
  // GripDeg is the larger of its angles to the two pushes, the best over where
  // along a flat contact the push acts. A round slice gives half the angle
  // between the pushes; a box's corner under the jaw, more.
  return { Ok: true, Code: "held", ContactDeg: Deg, SlideCm: Slide, Why: "", ...Extra,
    PushDeg: Math.atan2(-Jaw.z, -Jaw.x) * 180 / Math.PI,
    GripDeg: (Jaw.z + Fixed.z < 0 ? 1 : -1) * ChordAngle(FixedTouch.Patch, Fixed, JawTouch.Patch, Jaw),
    Contacts: { Jaw: JawTouch, Fixed: FixedTouch } };
}

export type Vec3 = { x: number; y: number; z: number };
/**
 * The two-pad grip's friction angle in 3D (deg) for a standing object (its
 * sides vertical), from the 2D contacts: what the jaws' plane leaves out. A
 * plane tilted out of level by the wrist's roll meets a vertical side at a
 * slant, so each finger touches it first on one edge of its thickness (rigid:
 * the 16:49 can's fixed pad on its top edge, the moving jaw on its bottom
 * one), and pushes along the side's own (level) normal there. The line
 * between those contacts, against those normals; the best pair along the
 * patches, as ChordAngle. U, H, A: the jaws' plane (gripper x, the hinge, the
 * approach; display frame, unit); gripper z is -A.
 */
export function GripIn3D(Contacts: NonNullable<CloseResult["Contacts"]>, U: Vec3, H: Vec3, A: Vec3, Rules = SweepRules): number {
  const In3 = (P: P2): Vec3 => ({ x: P.x * U.x - P.z * A.x, y: P.x * U.y - P.z * A.y, z: P.x * U.z - P.z * A.z });
  const Norm = (V: Vec3) => { const L = Math.hypot(V.x, V.y, V.z) || 1; return { x: V.x / L, y: V.y / L, z: V.z / L }; };
  const Side = (Push: P2) => {
    const N = In3(Push);
    // Level: the side's normal is N turned about the plane's line by the part along H that cancels its rise.
    const Lambda = Math.abs(H.z) > 1e-6 ? -N.z / H.z : 0;
    // The finger's edge that touches first: the one that would go into the side ((h H).N3 > 0).
    const Edge = Lambda > 1e-9 ? Rules.FingerHalfThickCm : Lambda < -1e-9 ? -Rules.FingerHalfThickCm : 0;
    return { Normal: Norm({ x: N.x + Lambda * H.x, y: N.y + Lambda * H.y, z: N.z + Lambda * H.z }), Edge };
  };
  const F = Side(Contacts.Fixed.Push), J = Side(Contacts.Jaw.Push);
  const Angle = (C: Vec3, N: Vec3) => {
    const L = Math.hypot(C.x, C.y, C.z) || 1;
    return Math.acos(Math.max(-1, Math.min(1, (C.x * N.x + C.y * N.y + C.z * N.z) / L))) * 180 / Math.PI;
  };
  let Best = Infinity;
  for (const Pf of Contacts.Fixed.Patch) for (const Pj of Contacts.Jaw.Patch) {
    const A3 = In3(Pf), B3 = In3(Pj);
    const C = { x: B3.x - A3.x + (J.Edge - F.Edge) * H.x, y: B3.y - A3.y + (J.Edge - F.Edge) * H.y, z: B3.z - A3.z + (J.Edge - F.Edge) * H.z };
    Best = Math.min(Best, Math.max(Angle(C, F.Normal), Angle({ x: -C.x, y: -C.y, z: -C.z }, J.Normal)));
  }
  return Best;
}

// Outward normals of a counter-clockwise hull's sides; the signed turn from one direction to another.
const Outward = (A: P2, B: P2) => { const L = Math.hypot(B.x - A.x, B.z - A.z) || 1; return { x: (B.z - A.z) / L, z: -(B.x - A.x) / L }; };
const Turn = (A: P2, B: P2) => Math.atan2(A.x * B.z - A.z * B.x, A.x * B.x + A.z * B.z);
// The object's corner nearest a point, and whether the sides on both sides of it are short:
// a polygon standing in for a curve there (a round slice), not a real corner.
function CurveCorner(Object_: P2[], Point: P2, Known = -1) {
  let Index = Known, Best = Infinity;
  if (Index < 0) {
    for (let I = 0; I < Object_.length; I++) {
      const D = (Object_[I].x - Point.x) ** 2 + (Object_[I].z - Point.z) ** 2;
      if (D < Best) { Best = D; Index = I; }
    }
  }
  const Prev = Object_[(Index + Object_.length - 1) % Object_.length], V = Object_[Index], Next = Object_[(Index + 1) % Object_.length];
  const Curve = Math.hypot(V.x - Prev.x, V.z - Prev.z) < FlatEdgeCm && Math.hypot(Next.x - V.x, Next.z - V.z) < FlatEdgeCm;
  return { Curve, Prev, V, Next, N1: Outward(Prev, V), N2: Outward(V, Next),
    M1: { x: (Prev.x + V.x) / 2, z: (Prev.z + V.z) / 2 }, M2: { x: (V.x + Next.x) / 2, z: (V.z + Next.z) / 2 } };
}
// On the curve a polygon of short sides stands for, the point whose outward normal is Facing:
// between the two sides' midpoints (which lie on the curve), by how far the normal has turned.
function CurvePoint(Corner: ReturnType<typeof CurveCorner>, Facing: P2): P2 {
  const Whole = Turn(Corner.N1, Corner.N2), T = Math.abs(Whole) > 1e-9 ? Math.max(0, Math.min(1, Turn(Corner.N1, Facing) / Whole)) : 0.5;
  return { x: Corner.M1.x + T * (Corner.M2.x - Corner.M1.x), z: Corner.M1.z + T * (Corner.M2.z - Corner.M1.z) };
}
/**
 * How a gripper part touches the object at a near contact: the push it gives
 * (unit, into the object) and where (points along the contact). On a curve
 * (a round slice's polygon) a face of the part pushes along its own normal at
 * the curve's point facing it, a corner of the part along the curve's normal
 * where it sits; elsewhere the polygon's own contact (PushOn, ContactPatch).
 */
function TouchOf(Part: P2[], Object_: P2[], Near: Closest): { Push: P2; Patch: P2[] } {
  if (!CurveCorner(Object_, Near.OnB).Curve) {
    const Push = PushOn(Near, Object_);
    return { Push, Patch: ContactPatch(Part, Object_, Near.Distance + 2e-3) };
  }
  let Best: { Gap: number; Push: P2; Point: P2 } | null = null;
  for (let Index = 0; Index < Part.length; Index++) {
    const A = Part[Index], B = Part[(Index + 1) % Part.length], L = Math.hypot(B.x - A.x, B.z - A.z);
    if (L < 1e-9) continue;
    const M = Outward(A, B);
    let Low = 0;
    Object_.forEach((P, I) => { if ((P.x - A.x) * M.x + (P.z - A.z) * M.z < (Object_[Low].x - A.x) * M.x + (Object_[Low].z - A.z) * M.z) Low = I; });
    const Corner = CurveCorner(Object_, Object_[Low], Low);
    const Point = Corner.Curve ? CurvePoint(Corner, { x: -M.x, z: -M.z }) : Object_[Low];
    const Gap = (Point.x - A.x) * M.x + (Point.z - A.z) * M.z, Along = ((Point.x - A.x) * (B.x - A.x) + (Point.z - A.z) * (B.z - A.z)) / (L * L);
    if (Gap < -1e-3 || Along < -1e-6 || Along > 1 + 1e-6) continue;
    if (!Best || Gap < Best.Gap) Best = { Gap, Push: M, Point };
  }
  if (Best && Best.Gap <= Near.Distance + 2e-3) return { Push: Best.Push, Patch: [Best.Point] };
  // A corner of the part on the curve: the curve's inward normal where the corner sits.
  const Corner = CurveCorner(Object_, Near.OnB);
  const Span = Math.hypot(Corner.M2.x - Corner.M1.x, Corner.M2.z - Corner.M1.z) || 1;
  const T = Math.max(0, Math.min(1, ((Near.OnB.x - Corner.M1.x) * (Corner.M2.x - Corner.M1.x) + (Near.OnB.z - Corner.M1.z) * (Corner.M2.z - Corner.M1.z)) / (Span * Span)));
  const Angle = Math.atan2(Corner.N1.z, Corner.N1.x) + T * Turn(Corner.N1, Corner.N2);
  return { Push: { x: -Math.cos(Angle), z: -Math.sin(Angle) }, Patch: [Near.OnA] };
}

const PointToPolygon = (P: P2, Polygon: P2[]) => {
  let Best = Infinity;
  for (let Index = 0; Index < Polygon.length; Index++) {
    const A = Polygon[Index], B = Polygon[(Index + 1) % Polygon.length];
    const Ex = B.x - A.x, Ez = B.z - A.z, L2 = Ex * Ex + Ez * Ez;
    const T = L2 > 0 ? Math.max(0, Math.min(1, ((P.x - A.x) * Ex + (P.z - A.z) * Ez) / L2)) : 0;
    Best = Math.min(Best, Math.hypot(P.x - A.x - T * Ex, P.z - A.z - T * Ez));
  }
  return Best;
};
// Faces at least this long and within this of parallel press flat on each other (a box side on
// a pad), closing the last FlatGapCm by the plastic's give; a round slice's short sides never do.
const FlatEdgeCm = 0.3, FlatDeg = 3, FlatGapCm = 0.1;
// Where a gripper part touches a polygonal object, spread as 9 points from one
// end of that patch to the other: the object's corners within Within of the
// part, the part's corners within Within of the object, and where a long
// object side lies flat on a long side of the part, their overlap (one point:
// a corner contact).
function ContactPatch(Part: P2[], Object_: P2[], Within = 2e-3): P2[] {
  const Points: P2[] = [...Part.filter(P => PointToPolygon(P, Object_) <= Within), ...Object_.filter(P => PointToPolygon(P, Part) <= Within)];
  for (let I = 0; I < Part.length; I++) {
    const A = Part[I], B = Part[(I + 1) % Part.length], Lp = Math.hypot(B.x - A.x, B.z - A.z);
    if (Lp < FlatEdgeCm) continue;
    const Ux = (B.x - A.x) / Lp, Uz = (B.z - A.z) / Lp;
    for (let J = 0; J < Object_.length; J++) {
      const P = Object_[J], Q = Object_[(J + 1) % Object_.length], Lo = Math.hypot(Q.x - P.x, Q.z - P.z);
      if (Lo < FlatEdgeCm || Math.abs(Ux * (Q.z - P.z) - Uz * (Q.x - P.x)) / Lo > Math.sin(FlatDeg * Math.PI / 180)) continue;
      // The object side's span along the part's side, clipped to it; both ends must lie on it (within the give).
      const Tp = (P.x - A.x) * Ux + (P.z - A.z) * Uz, Tq = (Q.x - A.x) * Ux + (Q.z - A.z) * Uz;
      const From = Math.max(0, Math.min(Tp, Tq)), To = Math.min(Lp, Math.max(Tp, Tq));
      if (To - From < 1e-6) continue;
      const On = (T: number) => { const S = (T - Tp) / (Tq - Tp || 1); return { x: P.x + S * (Q.x - P.x), z: P.z + S * (Q.z - P.z) }; };
      const Ends = [On(From), On(To)];
      if (Ends.every(E => Math.abs(Ux * (E.z - A.z) - Uz * (E.x - A.x)) <= Within + FlatGapCm)) Points.push(...Ends);
    }
  }
  if (Points.length < 2) return Points;
  let Ends: [P2, P2] = [Points[0], Points[0]], Far = -1;
  for (const A of Points) for (const B of Points) { const D = Math.hypot(A.x - B.x, A.z - B.z); if (D > Far) { Far = D; Ends = [A, B]; } }
  return Array.from({ length: 9 }, (_, Index) => ({ x: Ends[0].x + (Ends[1].x - Ends[0].x) * Index / 8, z: Ends[0].z + (Ends[1].z - Ends[0].z) * Index / 8 }));
}
// The least, over the two contact patches, of the larger angle the line between
// the contacts makes with each push (deg): within atan(mu) of both, it holds.
function ChordAngle(First: P2[], FirstPush: P2, Second: P2[], SecondPush: P2): number {
  const Angle = (Dx: number, Dz: number, Push: P2) => {
    const Length = Math.hypot(Dx, Dz) || 1;
    return Math.acos(Math.max(-1, Math.min(1, (Dx * Push.x + Dz * Push.z) / Length))) * 180 / Math.PI;
  };
  let Best = Infinity;
  for (const A of First) for (const B of Second) {
    Best = Math.min(Best, Math.max(Angle(B.x - A.x, B.z - A.z, FirstPush), Angle(A.x - B.x, A.z - B.z, SecondPush)));
  }
  return Best;
}

// The push a contact gives the object (B), as a unit vector: across the gap
// from A's contact point into B. An A edge on a B corner pushes along the
// edge's normal; a B edge under an A corner, along its own; corner on corner,
// along the mean of B's two edge normals there.
function PushOn(Near: Closest, Object_: P2[]): P2 {
  let Fx = Near.OnB.x - Near.OnA.x, Fz = Near.OnB.z - Near.OnA.z;
  if (Near.Distance < 1e-9 || (!Near.AEdge && !Near.BEdge)) {
    const Index = Object_.findIndex(P => Math.hypot(P.x - Near.OnB.x, P.z - Near.OnB.z) < 1e-9);
    if (Index >= 0) {
      const Prev = Object_[(Index + Object_.length - 1) % Object_.length], Next = Object_[(Index + 1) % Object_.length], P = Object_[Index];
      // Counter-clockwise hulls have their left normals inward.
      const Inward = (A: P2, B: P2) => { const L = Math.hypot(B.x - A.x, B.z - A.z) || 1; return { x: -(B.z - A.z) / L, z: (B.x - A.x) / L }; };
      const N1 = Inward(Prev, P), N2 = Inward(P, Next);
      const Gx = Fx, Gz = Fz;
      Fx = N1.x + N2.x; Fz = N1.z + N2.z;
      if (Near.Distance > 1e-9 && Fx * Gx + Fz * Gz < 0) { Fx = -Fx; Fz = -Fz; }
    }
  }
  const Length = Math.hypot(Fx, Fz) || 1;
  return { x: Fx / Length, z: Fz / Length };
}

/** The open moving jaw's clearance from a slice at an angle and at the real jaw's ~11 deg more closed. */
export function OpenClearance(Slice: P2[], OpenDeg: number, Rules = SweepRules): number {
  return Math.min(Gap(JawPolygon(OpenDeg), Slice), Gap(JawPolygon(Math.max(OpenDeg - Rules.JawUncertaintyDeg, Rules.PadsMeetDeg)), Slice));
}

