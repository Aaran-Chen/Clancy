"""CAD triangle/box contact and solid containment, accelerated by a static BVH."""
import json as Json
import struct
from pathlib import Path
import numpy as Np

Data = Json.loads((Path(__file__).resolve().parent / "BasketSurface.json").read_text())
Epsilon = 1e-7  # Numerical contact tolerance in cm, not a clearance zone.
RayDirection = Np.array([1., 0.3713906763541037, 0.5291502622129182])
# Answers Surface.Overlaps keeps, each about 0.25 kB (a planning process holds two surfaces); past this many it
# starts again.
MemoLimit = 30000
_Shapes = ((3,), (3, 3), (3,))


def _Key(Center, Rotation, Half, Padding):
    """Surface.Overlaps' arguments as one key, or None (not kept) for any but float64 arrays of the planner's shapes and
    a float padding: equal keys only for the very same numbers."""
    if not isinstance(Padding, float):  # NumPy's float64 is a float too.
        return None
    for Value, Shape in zip((Center, Rotation, Half), _Shapes):
        if not isinstance(Value, Np.ndarray) or Value.dtype != Np.float64 or Value.shape != Shape:
            return None
    return Center.tobytes() + Rotation.tobytes() + Half.tobytes() + struct.pack("<d", Padding)


class Surface:
    def __init__(Self, Offset, Source=None):
        Source = Data if Source is None else Source
        Self.Triangles = Np.array(Source["Vertices"])[Np.array(Source["Faces"])] - Np.array(Offset)
        Self.Nodes = Source["Nodes"]
        Self.Bounds = Np.array([Node["Bounds"] for Node in Self.Nodes]) - Np.array(Offset)
        # Scalar bounds avoid allocating tiny NumPy arrays at every tree node.
        # This is the same inclusive overlap test, with the same contact epsilon.
        Self.SearchBounds = Np.concatenate((Self.Bounds[:, 0] - Epsilon, Self.Bounds[:, 1] + Epsilon), axis=1).tolist()
        Self.RayBounds = Self.Bounds.reshape(len(Self.Nodes), 6).tolist()
        # Node children and faces as plain lists: the traversal runs per box
        # check, and a dict lookup per node is a measurable share of planning.
        Self.Children = [Node.get("Children") for Node in Self.Nodes]
        Self.Faces = [Node.get("Faces") for Node in Self.Nodes]
        Self.RootBounds = tuple(Self.Bounds[0].ravel().tolist())
        Self.Components = Np.array(Source["Components"])
        Self.Solids = Np.array(Source["Solids"])
        # The containment ray has a fixed direction, so every quantity that
        # depends only on it and the (static) triangles is computed once here.
        Self.Corner = Self.Triangles[:, 0]
        Self.First = Self.Triangles[:, 1] - Self.Corner
        Self.Second = Self.Triangles[:, 2] - Self.Corner
        Self.RayCross = Np.cross(RayDirection, Self.Second)
        Self.Determinant = Np.einsum("ij,ij->i", Self.First, Self.RayCross)
        Self.RayValid = Np.abs(Self.Determinant) > 1e-12
        Self.RayInverse = Np.divide(1., Self.Determinant, out=Np.zeros_like(Self.Determinant), where=Self.RayValid)
        Self.Memo = {}

    def Candidates(Self, Low, High):
        Pending, Result = [0], []
        Bounds, Children, Faces = Self.SearchBounds, Self.Children, Self.Faces
        LowX, LowY, LowZ = map(float, Low)
        HighX, HighY, HighZ = map(float, High)
        while Pending:
            Index = Pending.pop()
            MinX, MinY, MinZ, MaxX, MaxY, MaxZ = Bounds[Index]
            if HighX < MinX or HighY < MinY or HighZ < MinZ or LowX > MaxX or LowY > MaxY or LowZ > MaxZ:
                continue
            Child = Children[Index]
            if Child is not None:
                Pending.extend(Child)
            else:
                Result.extend(Faces[Index])
        return Result

    def RayCandidates(Self, Point):
        """Faces whose node the fixed-direction ray from Point can cross.

        Scalar slab test: every RayDirection component is positive, so the near
        face of each slab is always the low bound. Same test as dividing the
        bounds by the direction, without a NumPy array per node.
        """
        Pending, Result = [0], []
        Bounds, Children, Faces = Self.RayBounds, Self.Children, Self.Faces
        PointX, PointY, PointZ = map(float, Point)
        InverseX, InverseY, InverseZ = (1. / RayDirection).tolist()
        while Pending:
            Index = Pending.pop()
            MinX, MinY, MinZ, MaxX, MaxY, MaxZ = Bounds[Index]
            Near = (MinX - PointX) * InverseX
            Value = (MinY - PointY) * InverseY
            if Value > Near:
                Near = Value
            Value = (MinZ - PointZ) * InverseZ
            if Value > Near:
                Near = Value
            if Near < 0.:
                Near = 0.
            Far = (MaxX - PointX) * InverseX
            Value = (MaxY - PointY) * InverseY
            if Value < Far:
                Far = Value
            Value = (MaxZ - PointZ) * InverseZ
            if Value < Far:
                Far = Value
            if Near > Far + Epsilon:
                continue
            Child = Children[Index]
            if Child is not None:
                Pending.extend(Child)
            else:
                Result.extend(Faces[Index])
        return Result

    @staticmethod
    def TriangleContact(Triangles, Half):
        """Separating axes for an origin-centered box against a triangle batch."""
        if len(Triangles) == 0:
            return False
        Inside = Np.all(Triangles.min(axis=1) <= Half, axis=1) & Np.all(Triangles.max(axis=1) >= -Half, axis=1)
        Triangles = Triangles[Inside]
        if len(Triangles) == 0:
            return False
        # A corner inside the box is already contact, and needs three array
        # operations instead of the full axis test.
        if Np.any(Np.all(Np.abs(Triangles) <= Half, axis=2)):
            return True
        Edges = Np.roll(Triangles, -1, axis=1) - Triangles
        # The nine edge-by-box-axis crosses, written out (cross(E, X) is
        # (0, Ez, -Ey) and so on), then the triangle normal. A general cross
        # product over a broadcast (n, 3, 3, 3) array costs several times more.
        Zero = Np.zeros(Edges.shape[:2])
        Axes = Np.concatenate([
            Np.stack((Zero, Edges[:, :, 2], -Edges[:, :, 1]), axis=2),
            Np.stack((-Edges[:, :, 2], Zero, Edges[:, :, 0]), axis=2),
            Np.stack((Edges[:, :, 1], -Edges[:, :, 0], Zero), axis=2),
            Np.cross(Edges[:, 0], Edges[:, 1])[:, None, :]], axis=1)
        Projection = Np.einsum("nvc,nac->nav", Triangles, Axes)
        Radius = Np.abs(Axes) @ Half
        Separated = (Projection.min(axis=2) > Radius) | (Projection.max(axis=2) < -Radius)
        return bool(Np.any(~Np.any(Separated, axis=1)))

    def Contains(Self, Point):
        """Union of watertight CAD solids; the open tray cavity stays outside."""
        MinX, MinY, MinZ, MaxX, MaxY, MaxZ = Self.RootBounds
        PointX, PointY, PointZ = map(float, Point)
        if PointX < MinX or PointY < MinY or PointZ < MinZ or PointX > MaxX or PointY > MaxY or PointZ > MaxZ:
            return False
        Indices = Self.RayCandidates(Point)
        if not Indices:
            return False
        Corner = Self.Corner[Indices]
        Cross = Self.RayCross[Indices]
        First = Self.First[Indices]
        Second = Self.Second[Indices]
        Inverse = Self.RayInverse[Indices]
        Valid = Self.RayValid[Indices].copy()
        Delta = Point - Corner
        U = Np.einsum("ij,ij->i", Delta, Cross) * Inverse
        Q = Np.cross(Delta, First)
        V = Q @ RayDirection * Inverse
        Distance = Np.einsum("ij,ij->i", Second, Q) * Inverse
        Valid &= (U >= -1e-10) & (V >= -1e-10) & (U + V <= 1 + 1e-10) & (Distance > Epsilon)
        if not Valid.any():
            return False
        Components = Self.Components[Indices]
        for Component in Np.unique(Components[Valid]):
            if not Self.Solids[Component]:
                continue
            Hits = Np.sort(Distance[Valid & (Components == Component)])
            Count = 1 + int(Np.count_nonzero(Np.diff(Hits) > Epsilon))
            if Count % 2:
                return True
        return False

    def Overlaps(Self, Center, Rotation, Half, Padding=0.0):
        """Whether the box (world Center, Rotation, Half sizes, grown by Padding) touches or is inside the surface.

        The surface never changes, so an answer is kept for the exact same arguments (_Key: the same numbers): a
        route plan asks the same box at the same pose again and again (28% of the calls of the 2026-10-26 02:05
        Home), and the answer is the one the check (Check) gave the first time."""
        Key = _Key(Center, Rotation, Half, Padding)
        if Key is None:
            return Self.Check(Center, Rotation, Half, Padding)
        Memo = Self.Memo
        Answer = Memo.get(Key)
        if Answer is None:
            Answer = Memo[Key] = Self.Check(Center, Rotation, Half, Padding)
            if len(Memo) > MemoLimit:
                Self.Memo = {}
        return Answer

    def Check(Self, Center, Rotation, Half, Padding=0.0):
        # A local cube inflated by r contains the full radius-r motion envelope.
        Expanded = Half + Padding + Epsilon
        Extent = Np.abs(Rotation) @ Expanded
        Indices = Self.Candidates(Center - Extent, Center + Extent)
        if not Indices:
            return Self.Contains(Center)
        Local = (Self.Triangles[Indices] - Center) @ Rotation
        return Self.TriangleContact(Local, Expanded) or Self.Contains(Center)
