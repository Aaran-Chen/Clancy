// THE GAP BETWEEN TWO CONVEX SHAPES (GJK). The deep grasp (GraspPlan.ts PalmGap) measures how near
// an object's prism stands to the palm and servo on their CAD hull: each shape is given by its
// support map (its farthest point in a direction), and GJK walks the Minkowski difference A - B to
// the point nearest the origin. Exact up to Tolerance, from below: the returned gap is never more
// than the true one, and 0 where the shapes meet.

export type V3 = { x: number; y: number; z: number };
export type Support = (Direction: V3) => V3;

const Dot = (A: V3, B: V3) => A.x * B.x + A.y * B.y + A.z * B.z;
const Sub = (A: V3, B: V3) => ({ x: A.x - B.x, y: A.y - B.y, z: A.z - B.z });
const Neg = (A: V3) => ({ x: -A.x, y: -A.y, z: -A.z });

/**
 * The point of a simplex's hull (1 to 4 points) nearest the origin, and the face it lies on: every
 * subset's affine projection of the origin, kept where it lies inside that subset (barycentric weights
 * all >= 0), the nearest of those (the nearest point of the hull is one of them).
 */
function Nearest(Points: V3[]): { Point: V3; Face: V3[] } {
  let Best: { Point: V3; Face: V3[]; Squared: number } | null = null;
  for (let Mask = 1; Mask < 1 << Points.length; Mask++) {
    const Face = Points.filter((_, Index) => Mask & (1 << Index));
    const P0 = Face[0], Edges = Face.slice(1).map(P => Sub(P, P0));
    // The normal equations of min |P0 + sum Mu_i E_i|: G Mu = -E^T P0.
    const K = Edges.length;
    const G = Edges.map(A => Edges.map(B => Dot(A, B))), R = Edges.map(E => -Dot(E, P0));
    let Mu: number[] | null = [];
    if (K === 1) Mu = G[0][0] > 1e-18 ? [R[0] / G[0][0]] : null;
    else if (K === 2) {
      const Det = G[0][0] * G[1][1] - G[0][1] * G[1][0];
      Mu = Math.abs(Det) > 1e-18 * (G[0][0] * G[1][1] + 1e-30) ? [(R[0] * G[1][1] - G[0][1] * R[1]) / Det, (G[0][0] * R[1] - R[0] * G[1][0]) / Det] : null;
    } else if (K === 3) {
      const D3 = (M: number[][]) => M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
        + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
      const Det = D3(G);
      const Scale = G[0][0] * G[1][1] * G[2][2] + 1e-30;
      Mu = Math.abs(Det) > 1e-18 * Scale ? [0, 1, 2].map(Column => D3(G.map((Row, I) => Row.map((Value, J) => (J === Column ? R[I] : Value)))) / Det) : null;
    }
    if (!Mu) continue;
    const Weights = [1 - Mu.reduce((Sum, Value) => Sum + Value, 0), ...Mu];
    if (Weights.some(Weight => Weight < -1e-12)) continue;
    const Point = Edges.reduce((Sum, E, Index) => ({ x: Sum.x + Mu![Index] * E.x, y: Sum.y + Mu![Index] * E.y, z: Sum.z + Mu![Index] * E.z }), P0);
    const Squared = Dot(Point, Point);
    if (!Best || Squared < Best.Squared) Best = { Point, Face, Squared };
  }
  return Best ?? { Point: Points[0], Face: [Points[0]] };
}

/**
 * The gap (cm) between convex shapes A and B (support maps), from below within Tolerance: the
 * largest separating-plane distance GJK found. 0 where they meet. Start: any direction (the
 * nearer it points from B toward A, the fewer steps).
 */
export function ConvexGap(SupportA: Support, SupportB: Support, Start: V3, Tolerance = 1e-4): number {
  const Minkowski = (Direction: V3) => Sub(SupportA(Direction), SupportB(Neg(Direction)));
  let Face: V3[] = [Minkowski(Neg(Start))];
  let V = Face[0], Lower = 0;
  for (let Step = 0; Step < 64; Step++) {
    const Length = Math.sqrt(Dot(V, V));
    if (Length < 1e-9) return 0;
    const W = Minkowski(Neg(V));
    // Every point X of A - B has X . V >= W . V: the plane through W normal to V separates the origin by that much.
    Lower = Math.max(Lower, Dot(V, W) / Length);
    if (Length - Lower <= Tolerance) return Math.max(0, Lower);
    const Next = Nearest([...Face, W]);
    if (Next.Face.length === 4) return 0;
    Face = Next.Face;
    V = Next.Point;
  }
  return Math.max(0, Lower);
}
