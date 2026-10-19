import { build as Build } from "esbuild";
import { readFileSync as ReadFile } from "node:fs";
import { createRequire as CreateRequire } from "node:module";
import Assert from "node:assert/strict";
import Path from "node:path";
import { fileURLToPath as FilePath } from "node:url";
import { Box3, PerspectiveCamera, Vector3 } from "three";

const Root = Path.dirname(FilePath(import.meta.url));
const Bundle = await Build({ stdin: { contents: 'export * from "./src/lib/XLeRobotAssembly"; export * from "./src/lib/CadViewControls";', resolveDir: Root, loader: "ts" }, bundle: true,
  platform: "node", format: "cjs", write: false });
const Module = { exports: {} };
new Function("require", "module", "exports", Bundle.outputFiles[0].text)(CreateRequire(import.meta.url), Module, Module.exports);
const Assembly = Module.exports;
const Source = Assembly.XLeRobotScene;
for (const [View, Normal] of Object.entries({ front: [0, 0, 1], back: [0, 0, -1], right: [-1, 0, 0], left: [1, 0, 0], top: [0, -1, 0], bottom: [0, 1, 0] })) {
  const Camera = new PerspectiveCamera(35, 1, .1, 3000); Camera.up.set(0, 0, 1);
  Assembly.FitAssemblyCamera(Camera, new Box3(new Vector3(-10, -10, -10), new Vector3(10, 10, 10)), View);
  const Rotation = Assembly.ViewCubeRotation(Camera);
  Assert.ok(Math.abs(Rotation.determinant() - 1) < 1e-10, "Cube labels must not mirror");
  Assert.ok(new Vector3(...Normal).transformDirection(Rotation).z > .999, `${View} cube face must face the viewer`);
}
const Frame = JSON.parse(ReadFile(Path.join(Root, "../back end/WorkspaceFrame.json"), "utf8")).RobotFrame;
const ActualPlacement = Assembly.AssemblyPlacement(Frame.ArmBasePositionCm, Frame.ControlledArm);
Assert.ok(ActualPlacement.Origin.length() < 1e-10);
Assert.ok(Math.abs(ActualPlacement.OriginLine[0].z) < 1e-10 && Math.abs(ActualPlacement.OriginLine[1].z) < 1e-10);
Assert.ok(Math.abs(ActualPlacement.OriginLine[0].x + ActualPlacement.OriginLine[1].x) < 1e-10);
Assert.ok(Math.abs(ActualPlacement.BodyOffset.z + Assembly.BinTopZCm) < 1e-10);
Assert.equal(Source.Parts.length, 32);
Assert.equal(new Set(Source.Parts.map(Part => Part.Name)).size, Source.Parts.length);
Assert.deepEqual(new Set(Source.Parts.map(Part => Part.Category)), new Set(["Cart", "Mast", "Head", "Drive", "SchematicEquipment"]));
const Bytes = ReadFile(Path.join(Root, "public/Cad/XLeRobotBody.glb"));
Assert.equal(Bytes.readUInt32LE(0), 0x46546c67);
const Gltf = JSON.parse(Bytes.subarray(20, 20 + Bytes.readUInt32LE(12)).toString("utf8"));
for (const Part of Source.Parts) Assert.ok(Gltf.nodes.some(Node => Node.name === Part.Name), Part.Name);
let ProjectionChecks = 0;
for (const Side of ["Left", "Right"]) {
  for (const Base of [{ x: 0, y: 0, z: 0 }, { x: 13, y: -24, z: 8 }]) {
    const Placement = Assembly.AssemblyPlacement(Base, Side);
    const Pan = new Vector3(...Source.Mounts[Side].PanCm).add(Placement.BodyOffset);
    Assert.ok(Pan.distanceTo(new Vector3(Base.x, Base.y, Base.z)) < 1e-10);
    Assert.ok(Math.abs(Placement.OtherPan.distanceTo(Pan) - 26.6) < 1e-10);
    Assert.ok(Placement.FloorZ < Base.z - 75);
    const Bounds = Assembly.BodyBounds(Placement.BodyOffset);
    // Include a representative extended live arm and target preview.
    Bounds.union(new Box3(new Vector3(Base.x - 20, Base.y - 6, Base.z - 12), new Vector3(Base.x + 15, Base.y + 45, Base.z + 40)));
    for (const Aspect of [0.35, 0.5, 0.78, 1, 1.78, 2.4]) {
      for (const View of ["iso", "top", "bottom", "front", "back", "right", "left"]) {
        const Camera = new PerspectiveCamera(35, Aspect, 0.1, 3000); Camera.up.set(0, 0, 1);
        Assembly.FitAssemblyCamera(Camera, Bounds, View);
        for (const Corner of Assembly.BoxCorners(Bounds)) {
          const Projected = new Vector3(Corner.x, Corner.y, Corner.z).project(Camera);
          Assert.ok(Math.abs(Projected.x) <= 0.820001, JSON.stringify({ Side, Aspect, View, Projected }));
          Assert.ok(Math.abs(Projected.y) <= 0.690001, JSON.stringify({ Side, Aspect, View, Projected }));
          Assert.ok(Projected.z > -1 && Projected.z < 1);
          ProjectionChecks += 1;
        }
      }
    }
  }
}
console.log({ BodyParts: Source.Parts.length, ProjectionChecks, ArmMounts: "Left/right preserve the controlled arm frame" });
