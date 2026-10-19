import { Box3, Triangle, Vector3 } from "three";
import Data from "../../../back end/BasketSurface.json";

const Epsilon = 1e-7;
const RayDirection = [1, 0.3713906763541037, 0.5291502622129182];
type Node = { Bounds: number[][]; Children?: number[]; Faces?: number[] };
type SurfaceData = { Nodes: Node[]; Vertices: number[][]; Faces: number[][]; Components: number[]; Solids: boolean[] };
const Cross = (A: number[], B: number[]) => [A[1] * B[2] - A[2] * B[1], A[2] * B[0] - A[0] * B[2], A[0] * B[1] - A[1] * B[0]];
const Dot = (A: number[], B: number[]) => A[0] * B[0] + A[1] * B[1] + A[2] * B[2];
const Subtract = (A: number[], B: number[]) => [A[0] - B[0], A[1] - B[1], A[2] - B[2]];

export class BasketSurface {
  private readonly Nodes: Node[];
  private readonly Vertices: number[][];
  private readonly Box = new Box3();
  private readonly Triangle = new Triangle();

  constructor(Offset: number[], private readonly Source: SurfaceData = Data) {
    this.Nodes = Source.Nodes.map(Node => ({ ...Node, Bounds: Node.Bounds.map(Point => Subtract(Point, Offset)) }));
    this.Vertices = Source.Vertices.map(Point => Subtract(Point, Offset));
  }

  private Candidates(Low: number[], High: number[]): number[] {
    const Pending = [0], Result: number[] = [];
    while (Pending.length) {
      const Node = this.Nodes[Pending.pop()!];
      if (Low.some((Value, Axis) => Value > Node.Bounds[1][Axis] + Epsilon || High[Axis] < Node.Bounds[0][Axis] - Epsilon)) continue;
      if (Node.Children) Pending.push(...Node.Children);
      else Result.push(...Node.Faces!);
    }
    return Result;
  }

  private Contains(Point: number[]): boolean {
    const Bounds = this.Nodes[0].Bounds;
    if (Point.some((Value, Axis) => Value < Bounds[0][Axis] || Value > Bounds[1][Axis])) return false;
    const Pending = [0], Indices: number[] = [];
    while (Pending.length) {
      const Node = this.Nodes[Pending.pop()!];
      let Near = 0, Far = Infinity;
      for (let Axis = 0; Axis < 3; Axis++) {
        Near = Math.max(Near, (Node.Bounds[0][Axis] - Point[Axis]) / RayDirection[Axis]);
        Far = Math.min(Far, (Node.Bounds[1][Axis] - Point[Axis]) / RayDirection[Axis]);
      }
      if (Near > Far + Epsilon) continue;
      if (Node.Children) Pending.push(...Node.Children);
      else Indices.push(...Node.Faces!);
    }
    const Hits = new Map<number, number[]>();
    for (const Index of Indices) {
      const Component = this.Source.Components[Index];
      if (!this.Source.Solids[Component]) continue;
      const [A, B, C] = this.Source.Faces[Index].map(Vertex => this.Vertices[Vertex]);
      const First = Subtract(B, A), Second = Subtract(C, A), P = Cross(RayDirection, Second);
      const Determinant = Dot(First, P);
      if (Math.abs(Determinant) <= 1e-12) continue;
      const Delta = Subtract(Point, A), Q = Cross(Delta, First);
      const U = Dot(Delta, P) / Determinant, V = Dot(Q, RayDirection) / Determinant;
      const Distance = Dot(Second, Q) / Determinant;
      if (U < -1e-10 || V < -1e-10 || U + V > 1 + 1e-10 || Distance <= Epsilon) continue;
      const Values = Hits.get(Component) ?? [];
      Values.push(Distance); Hits.set(Component, Values);
    }
    for (const Values of Hits.values()) {
      Values.sort((A, B) => A - B);
      const Count = 1 + Values.slice(1).filter((Value, Index) => Value - Values[Index] > Epsilon).length;
      if (Count % 2) return true;
    }
    return false;
  }

  Overlaps(Center: Vector3, Axes: Vector3[], Half: Vector3, Padding = 0): boolean {
    const Expanded = Half.clone().addScalar(Padding + Epsilon);
    const Origin = Center.toArray();
    const Extents = [0, 1, 2].map(Axis => Axes.reduce((Sum, Local, Index) => Sum + Math.abs(Local.getComponent(Axis)) * Expanded.getComponent(Index), 0));
    const Indices = this.Candidates(Origin.map((Value, Axis) => Value - Extents[Axis]), Origin.map((Value, Axis) => Value + Extents[Axis]));
    this.Box.min.copy(Expanded).negate(); this.Box.max.copy(Expanded);
    for (const Index of Indices) {
      const Points = [this.Triangle.a, this.Triangle.b, this.Triangle.c];
      this.Source.Faces[Index].forEach((Vertex, Corner) => {
        const Delta = Subtract(this.Vertices[Vertex], Origin);
        Points[Corner].set(...Axes.map(Axis => Axis.x * Delta[0] + Axis.y * Delta[1] + Axis.z * Delta[2]) as [number, number, number]);
      });
      if (this.Box.intersectsTriangle(this.Triangle)) return true;
    }
    return this.Contains(Origin);
  }
}
