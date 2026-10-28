import { forwardRef as ForwardRef, memo as Memo, useEffect as UseEffect, useImperativeHandle as UseImperativeHandle, useRef as UseRef, useState as UseState } from "react";
import * as Three from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";
import { RoomEnvironment } from "three/examples/jsm/environments/RoomEnvironment.js";
import { LineSegments2 } from "three/examples/jsm/lines/LineSegments2.js";
import { LineSegmentsGeometry } from "three/examples/jsm/lines/LineSegmentsGeometry.js";
import { LineMaterial } from "three/examples/jsm/lines/LineMaterial.js";
import { toCreasedNormals } from "three/examples/jsm/utils/BufferGeometryUtils.js";
import { Box, Expand, Maximize2, Ruler, RotateCcw, Target as TargetIcon } from "lucide-react";
import { CadForward, CadForwardUrdf, CadModel, CadJointNames, GetCadAlignment, type CadAngles } from "../lib/CadKinematics";
import { RobotFrame, formatName, trackAngleFromRaw, type ServoName, type ServoState, type ArmAngles, type Point3, type TargetPose } from "../lib/robotModel";
import { DialTurn, JointArc, JointAxisAngle, JointDialRadiusCm, TurnJoint, type JointDial } from "../lib/JointInteraction";
import { AssemblyPlacement, BodyBounds, FitAssemblyCamera, XLeRobotScene, type ArmSide } from "../lib/XLeRobotAssembly";
import { TargetApproachDirection, ViewCubeRotation, WristWheelDelta, WristWheelStepDegrees } from "../lib/CadViewControls";
import { PitchGuideRadiusCm, type PitchGuideSegment } from "../lib/PitchGuide";
import DraggableViewCube from "./DraggableViewCube";
import type { ViewSettings } from "../lib/ViewSettings";

type ArmCanvasProps = {
  Settings: ViewSettings;
  Mode?: "Manual" | "Ai";
  SelectedJoint: ServoName | null; SelectedAxis: "x" | "y" | "z" | null;
  Servos: Record<ServoName, ServoState>; OnSelectJoint: (Name: ServoName | null) => void;
  /** Commands a joint's target raw, as its side-panel slider does. Without it the dial cannot be dragged. */
  OnJointChange?: (Name: ServoName, Raw: number) => void;
  joints: Point3[]; previewJoints: Point3[]; previewReachable: boolean;
  angles: ArmAngles; previewAngles: ArmAngles; proposalAngles?: ArmAngles;
  proposalJoints?: Point3[]; proposalReachable?: boolean; proposalTarget?: TargetPose | null;
  proposalEffectiveWristPitch?: number; targetWristPitchReachable: boolean; effectiveWristPitch: number;
  pitchGuide: PitchGuideSegment[] | null; target: TargetPose; targetEstimate?: Point3 | null;
  /** The task object as measured: drawn at its real footprint and height instead of the estimate's cube. */
  objectBox?: ObjectBox | null;
  alignmentRecorded: boolean; canCaptureAlignment: boolean; onCaptureAlignment: () => Promise<boolean>;
  TargetVisible?: boolean; TargetReadOnly?: boolean;
  targetEditing: boolean; targetSnapping: boolean;
  onTargetDragStart: (Kind?: "Position" | "Pitch") => void; onTargetDragEnd: (Cancelled?: boolean) => void;
  onTargetChange: (Target: TargetPose | ((Previous: TargetPose) => TargetPose)) => void;
};
export type ArmCanvasHandle = { getImageDataUrl: () => string | null };
/** An axis-aligned box in the visual frame (cm): centre, then X, Y and Z extents. */
export type ObjectBox = { Center: Point3; SizeCm: [number, number, number]; Label: string };
export const WRIST_WHEEL_DEG = WristWheelStepDegrees;
type CadInstance = { Root: Three.Group; Parts: { Mesh: Three.Mesh; Link: string }[];
  PoseKey: string; TransformKey: string; State: ReturnType<typeof CadForward> | null };
type SceneRuntime = { Renderer: Three.WebGLRenderer; Fit: (View?: string) => void; Draw: () => void; Invalidate: () => void };
/* Synthwave stage (SynthwaveTheme.css). Manual: a navy cabin over an electric-blue grid, the live arm cyan.
   AI: a deep indigo cabin over a purple grid, the live arm in the go green. The target is in the mode's colour.
   The scene starts at neither cabin (#141a20), so the first frame always applies its mode's tones. */
const SceneTones = {
  Manual: { Cabin: 0x0a1024, Ground: 0x070c1c, Grid: 0x4f8dff, Live: 0x5fe0ff, Target: 0xa6c4ff, Ghost: 0xa6c4ff, Proposal: 0xcfdcf5 },
  Ai: { Cabin: 0x120d25, Ground: 0x0d0a1b, Grid: 0xa077e0, Live: 0x8fe3be, Target: 0xcab4ff, Ghost: 0xbfa8f0, Proposal: 0xf0b8e4 },
} as const;
/* Test hooks on the canvas (TestCadTypingUi): data-scene-build counts scene builds, data-draws counts frames, and
   data-camera is the camera pose each frame was drawn from. */
let SceneBuilds = 0;

const CadCanvas = ForwardRef<ArmCanvasHandle, ArmCanvasProps>(function CadCanvas(Props, Ref) {
  const Host = UseRef<HTMLDivElement>(null);
  const Runtime = UseRef<SceneRuntime | null>(null);
  const Latest = UseRef(Props); Latest.current = Props;
  const [Loaded, SetLoaded] = UseState(false);
  const [BodyLoaded, SetBodyLoaded] = UseState(false);
  const [Failure, SetFailure] = UseState("");
  const [Reference, SetReference] = UseState(false);
  const [Measure, SetMeasure] = UseState(false);
  const [Recording, SetRecording] = UseState(false);
  const [JointDragging, SetJointDragging] = UseState(false);
  const [LiveSide] = UseState<ArmSide>(() => {
    if (RobotFrame) return RobotFrame.ControlledArm as ArmSide;
    try { return localStorage.getItem("XLeRobotLiveSide") === "Left" ? "Left" : XLeRobotScene.DefaultLiveArm as ArmSide; }
    catch { return XLeRobotScene.DefaultLiveArm as ArmSide; }
  });
  const Options = UseRef({ Reference, Measure, LiveSide }); Options.current = { Reference, Measure, LiveSide };
  const MeasureLabels = UseRef<HTMLDivElement>(null);
  const ViewCubeBody = UseRef<HTMLDivElement>(null);
  UseImperativeHandle(Ref, () => ({ getImageDataUrl: () => {
    try { Runtime.current?.Draw(); return Runtime.current?.Renderer.domElement.toDataURL("image/png") ?? null; }
    catch { return null; }
  } }), []);
  UseEffect(() => {
    const Element = Host.current;
    if (!Element) return;
    let Renderer: Three.WebGLRenderer;
    try { Renderer = new Three.WebGLRenderer({ antialias: true, alpha: true, powerPreference: "high-performance" }); }
    catch { SetFailure("The CAD view needs WebGL. Enable browser hardware acceleration and reload."); return; }
    let Disposed = false, Animation = 0, NeedsDraw = true, ShadowKey = "", MeasurementKey = "", ApproachKey = "", LastTargetFlash = -1;
    const LocalBase = CadForwardUrdf({}).Joints[0];
    let Current: CadInstance | null = null, Preview: CadInstance | null = null, Proposal: CadInstance | null = null, Parked: CadInstance | null = null;
    const Body = new Three.Group(); Body.name = "XLeRobotBody";
    const Canvas = Renderer.domElement;
    Canvas.className = "arm-canvas cad-canvas";
    Canvas.setAttribute("aria-label", "Interactive XLeRobot and SO-101 CAD assembly");
    Canvas.dataset.cadReady = "false";
    Canvas.dataset.sceneBuild = String(++SceneBuilds);
    let Draws = 0;
    Element.appendChild(Canvas);
    Renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    Renderer.shadowMap.enabled = true; Renderer.shadowMap.type = Three.PCFSoftShadowMap;
    Renderer.shadowMap.autoUpdate = false;
    Renderer.shadowMap.needsUpdate = true;
    Renderer.toneMapping = Three.ACESFilmicToneMapping; Renderer.toneMappingExposure = 0.9;
    Renderer.outputColorSpace = Three.SRGBColorSpace;
    const Scene = new Three.Scene(); Scene.background = new Three.Color("#141a20"); Scene.fog = new Three.Fog("#141a20", 400, 950); Scene.add(Body);
    const Environment = new RoomEnvironment();
    const Generator = new Three.PMREMGenerator(Renderer);
    const EnvironmentMap = Generator.fromScene(Environment, 0.04);
    Scene.environment = EnvironmentMap.texture; Scene.environmentIntensity = 0.7; Environment.dispose(); Generator.dispose();
    const Camera = new Three.PerspectiveCamera(35, 1, 0.1, 3000); Camera.up.set(0, 0, 1);
    const Controls = new OrbitControls(Camera, Canvas);
    Controls.enableDamping = true; Controls.dampingFactor = 0.09;
    Controls.minDistance = 16; Controls.maxDistance = 2000; Controls.maxPolarAngle = Math.PI;
    Controls.mouseButtons.LEFT = Three.MOUSE.PAN;
    Controls.mouseButtons.MIDDLE = Three.MOUSE.PAN; Controls.mouseButtons.RIGHT = Three.MOUSE.ROTATE;
    Scene.add(new Three.HemisphereLight(0xeaf6ff, 0x37434a, 1.2));
    const Key = new Three.DirectionalLight(0xf5ffff, 2.8); Key.position.set(-25, -20, 70); Key.castShadow = true;
    Key.shadow.mapSize.set(4096, 4096);
    Key.shadow.camera.left = -95; Key.shadow.camera.right = 95; Key.shadow.camera.top = 95; Key.shadow.camera.bottom = -95;
    Key.shadow.camera.near = 1; Key.shadow.camera.far = 500; Key.shadow.normalBias = 0.05; Key.shadow.bias = -0.0001;
    Scene.add(Key, Key.target);
    const Rim = new Three.DirectionalLight(0x90bedb, 1.6); Rim.position.set(35, 40, 30); Scene.add(Rim);
    const Ground = new Three.Mesh(new Three.PlaneGeometry(2000, 2000), new Three.MeshStandardMaterial({
      color: 0x0b1117, roughness: 1, envMapIntensity: 0.15 }));
    Ground.receiveShadow = true; Scene.add(Ground);
    const Grid = new Three.GridHelper(400, 40, 0xb8bccc, 0x8a8ea0); Grid.name = "WorkspaceGrid"; Grid.rotation.x = Math.PI / 2;
    (Grid.material as Three.Material).transparent = true; (Grid.material as Three.Material).opacity = 0.45; Scene.add(Grid);
    const TargetGroup = new Three.Group(); Scene.add(TargetGroup);
    const AxisGuideMaterial = new LineMaterial({ color: 0xf18b87, linewidth: 2, transparent: true, opacity: .65, depthTest: false, depthWrite: false, toneMapped: false });
    const AxisGuide = new LineSegments2(new LineSegmentsGeometry().setPositions([-60, 0, 0, 60, 0, 0]), AxisGuideMaterial);
    AxisGuide.name = "ActiveTargetAxis"; AxisGuide.visible = false; AxisGuide.renderOrder = 3; TargetGroup.add(AxisGuide);
    const JointGuide = new Three.Group(); JointGuide.name = "SelectedJointGuide"; JointGuide.visible = false; Scene.add(JointGuide);
    const JointRingMaterial = new LineMaterial({ color: 0x59e695, linewidth: 3, depthTest: false, depthWrite: false, toneMapped: false });
    const JointRing = new LineSegments2(new LineSegmentsGeometry(), JointRingMaterial);
    JointRing.name = "SelectedJointArc"; JointRing.renderOrder = 10; JointGuide.add(JointRing);
    const JointNeedleMaterial = new LineMaterial({ color: 0xffd287, linewidth: 3, depthTest: false, depthWrite: false, toneMapped: false });
    const JointNeedle = new LineSegments2(new LineSegmentsGeometry(), JointNeedleMaterial);
    JointNeedle.name = "SelectedJointNeedle"; JointNeedle.renderOrder = 11; JointGuide.add(JointNeedle);
    let JointGuideKey = "", JointSelection = "", SelectionAt = 0;
    // The selected joint's knob sits on its arc at the TARGET angle (the needle
    // shows where the joint is now). Dragging the knob, the arc or the joint's
    // own handle turns the joint through the same command path as its slider.
    const JointKnob = new Three.Mesh(new Three.SphereGeometry(1, 20, 14),
      new Three.MeshBasicMaterial({ depthTest: false, depthWrite: false, toneMapped: false, transparent: true }));
    JointKnob.name = "SelectedJointKnob"; JointKnob.renderOrder = 14; JointKnob.visible = false; Scene.add(JointKnob);
    const JointKnobOffset = new Three.Vector3(), SceneOffset = new Three.Vector3();
    let JointRingLocal: number[] = [];
    type JointDragState = { Joint: ServoName; Pointer: number; Frame: Three.Matrix4; Radius: number; StartRaw: number; Raw: number; Turn: number;
      StartX: number; StartY: number; LastX: number; LastY: number; Active: boolean; ClearOnClick: boolean };
    let JointDrag: JointDragState | null = null, HoverDial = false;
    // Clickable joint handles for both arms in the scene: the live pose and the
    // target pose preview. Drawn without depth so a handle is never buried in a
    // link, and picked in screen space so a small handle is still easy to hit.
    type JointMarker = { Group: Three.Group; Core: Three.Mesh; Halo: Three.Mesh; Joint: ServoName; Kind: "Live" | "Target" };
    const MarkerCore = new Three.SphereGeometry(0.42, 18, 12);
    const MarkerHalo = new Three.RingGeometry(0.66, 0.86, 30);
    const JointMarkers: JointMarker[] = [];
    const MarkerRoot = new Three.Group(); MarkerRoot.name = "JointMarkers"; Scene.add(MarkerRoot);
    for (const Kind of ["Live", "Target"] as const) {
      for (const Joint of CadJointNames) {
        const Group = new Three.Group();
        Group.name = `${Kind}JointMarker_${Joint}`;
        Group.userData.Joint = Joint; Group.userData.Kind = Kind;
        const Core = new Three.Mesh(MarkerCore, new Three.MeshBasicMaterial({ depthTest: false, depthWrite: false, toneMapped: false, transparent: true }));
        const Halo = new Three.Mesh(MarkerHalo, new Three.MeshBasicMaterial({ depthTest: false, depthWrite: false, toneMapped: false, transparent: true, side: Three.DoubleSide }));
        Core.renderOrder = 13; Halo.renderOrder = 12;
        Group.add(Core, Halo); MarkerRoot.add(Group);
        JointMarkers.push({ Group, Core, Halo, Joint: Joint as ServoName, Kind });
      }
    }
    let HoverMarker: JointMarker | null = null;
    const ReducedMotion = window.matchMedia("(prefers-reduced-motion: reduce)").matches;
    const TargetMaterial = new Three.MeshBasicMaterial({ color: 0xeec172, depthTest: false });
    const TargetBall = new Three.Mesh(new Three.SphereGeometry(0.43, 24, 16), TargetMaterial);
    TargetBall.renderOrder = 5; TargetGroup.add(TargetBall);
    const Halo = new Three.Mesh(new Three.TorusGeometry(1.15, 0.065, 8, 64), TargetMaterial); Halo.renderOrder = 5; TargetGroup.add(Halo);
    const ArrowForwardCm = 4.4, ArrowBehindCm = 8;
    const Arrow = new Three.ArrowHelper(new Three.Vector3(0, 1, 0), new Three.Vector3(),
      ArrowForwardCm + ArrowBehindCm, 0xeec172, 0.8, 0.45);
    Arrow.name = "TargetApproachRay"; TargetGroup.add(Arrow);
    for (const Part of [Arrow.line, Arrow.cone]) {
      Part.renderOrder = 4;
      for (const Material of Array.isArray(Part.material) ? Part.material : [Part.material]) {
        Material.depthTest = false; Material.depthWrite = false;
      }
    }
    const PitchMaterial = new LineMaterial({ color: 0x59e695, depthTest: false, depthWrite: false, toneMapped: false });
    PitchMaterial.linewidth = 3;
    const PitchArc = new LineSegments2(new LineSegmentsGeometry(), PitchMaterial);
    PitchArc.name = "AcceptableWristAngles"; PitchArc.renderOrder = 3; PitchArc.visible = false;
    TargetGroup.add(PitchArc);
    let AppliedPitchGuide: PitchGuideSegment[] | null = null;
    const Estimate = new Three.Mesh(new Three.BoxGeometry(2.2, 2.2, 2.2), new Three.MeshBasicMaterial({ color: 0x73b5ff, wireframe: true })); Scene.add(Estimate);
    // A unit box scaled to the measured object: a light fill with its outline.
    const ObjectBoxMesh = new Three.Mesh(new Three.BoxGeometry(1, 1, 1),
      new Three.MeshBasicMaterial({ color: 0x73b5ff, transparent: true, opacity: 0.16, depthWrite: false }));
    ObjectBoxMesh.name = "MeasuredObject"; ObjectBoxMesh.visible = false;
    ObjectBoxMesh.add(new Three.LineSegments(new Three.EdgesGeometry(ObjectBoxMesh.geometry), new Three.LineBasicMaterial({ color: 0x73b5ff })));
    Scene.add(ObjectBoxMesh);
    const DimensionLines = new Three.LineSegments(new Three.BufferGeometry(), new Three.LineBasicMaterial({ color: 0xc7e9e2, depthTest: false, transparent: true, opacity: 0.65 }));
    DimensionLines.renderOrder = 6; Scene.add(DimensionLines);
    const OriginGroup = new Three.Group(); OriginGroup.name = "WorkspaceOrigin"; Scene.add(OriginGroup);
    const HalfSpan = (XLeRobotScene.Mounts.Right.PanCm[0] - XLeRobotScene.Mounts.Left.PanCm[0]) / 2;
    const AxisMaterials: LineMaterial[] = [];
    const OriginDot = new Three.Mesh(new Three.SphereGeometry(0.22, 16, 12), new Three.MeshBasicMaterial({ color: 0xffffff, depthTest: false, depthWrite: false }));
    OriginDot.name = "OriginPoint";
    OriginDot.renderOrder = 8; OriginGroup.add(OriginDot);
    const LabelTextures: Three.CanvasTexture[] = [];
    const OriginLabels: Three.Sprite[] = [];
    function OriginLabel(Text: string, Color: string, Position: Three.Vector3) {
      const LabelCanvas = document.createElement("canvas"); LabelCanvas.width = 80; LabelCanvas.height = 48;
      const Context = LabelCanvas.getContext("2d")!;
      Context.fillStyle = Color; Context.font = "600 28px sans-serif"; Context.textAlign = "center"; Context.textBaseline = "middle";
      Context.fillText(Text, LabelCanvas.width / 2, 24);
      const Texture = new Three.CanvasTexture(LabelCanvas); Texture.colorSpace = Three.SRGBColorSpace; LabelTextures.push(Texture);
      const Label = new Three.Sprite(new Three.SpriteMaterial({ map: Texture, depthTest: false, depthWrite: false, sizeAttenuation: false, toneMapped: false }));
      Label.name = `CoordinateLabel${Text}`; Label.userData.Axis = Text;
      Label.position.copy(Position); Label.userData.WidthPixels = LabelCanvas.width / 48 * 14;
      Label.renderOrder = 9; OriginLabels.push(Label); OriginGroup.add(Label);
    }
    for (const [Direction, Color, Label, Span] of [
      [new Three.Vector3(1, 0, 0), 0xf18b87, "+X", HalfSpan],
      [new Three.Vector3(0, 1, 0), 0x85d999, "+Y", 7],
      [new Three.Vector3(0, 0, 1), 0x8db8ff, "+Z", 7],
    ] as const) {
      // Standard WebGL lines stay one pixel wide on Windows. Expanded line
      // geometry gives the graph a consistent, adjustable screen thickness.
      const Material = new LineMaterial({ color: Color, linewidth: 3, depthTest: false, depthWrite: false, toneMapped: false });
      const Shaft = new LineSegments2(new LineSegmentsGeometry().setPositions([
        ...Direction.clone().multiplyScalar(-Span).toArray(), ...Direction.clone().multiplyScalar(Span).toArray(),
      ]), Material);
      Shaft.name = `CoordinateAxis${Label}`; Shaft.renderOrder = 7; AxisMaterials.push(Material); OriginGroup.add(Shaft);
      const Head = new Three.Mesh(new Three.ConeGeometry(.2, .7, 12), new Three.MeshBasicMaterial({ color: Color, depthTest: false, depthWrite: false, toneMapped: false }));
      Head.name = `PositiveAxis${Label}`;
      Head.position.copy(Direction).multiplyScalar(Span - .35);
      Head.quaternion.setFromUnitVectors(new Three.Vector3(0, 1, 0), Direction);
      Head.renderOrder = 8; OriginGroup.add(Head);
      OriginLabel(Label, `#${Color.toString(16)}`, Direction.clone().multiplyScalar(Span + 1.8));
    }
    const GhostMaterial = new Three.MeshStandardMaterial({ color: 0xeabb6f, transparent: true, opacity: 0.24, metalness: 0, roughness: 0.5, depthWrite: false });
    const ProposalMaterial = GhostMaterial.clone(); ProposalMaterial.color.setHex(0x77afff);
    const ShellMaterial = new Three.MeshPhysicalMaterial({ color: 0x568f7f, metalness: 0.08, roughness: 0.42, clearcoat: 0.22, clearcoatRoughness: 0.45 });
    const ServoMaterial = new Three.MeshStandardMaterial({ color: 0x172129, metalness: 0.38, roughness: 0.38 });
    const SelectedShell = ShellMaterial.clone(); SelectedShell.emissive.setHex(0x36dfaa); SelectedShell.emissiveIntensity = .3;
    const SelectedServo = ServoMaterial.clone(); SelectedServo.emissive.setHex(0x36dfaa); SelectedServo.emissiveIntensity = .35;
    const ParkedMaterial = new Three.MeshStandardMaterial({ color: 0x626262, metalness: 0, roughness: 0.9, envMapIntensity: 0.2 });
    const ParkedServoMaterial = new Three.MeshStandardMaterial({ color: 0x444444, metalness: 0, roughness: 0.9, envMapIntensity: 0.2 });
    const BodyMaterials: Record<string, Three.Material> = {
      Cart: new Three.MeshStandardMaterial({ color: 0x597781, metalness: 0.18, roughness: 0.47 }),
      Metal: new Three.MeshStandardMaterial({ color: 0x849094, metalness: 0.75, roughness: 0.3 }),
      Dark: ServoMaterial, Shell: new Three.MeshPhysicalMaterial({ color: 0x98a5a5, metalness: 0.08, roughness: 0.5 }),
      Rubber: new Three.MeshStandardMaterial({ color: 0x151a1c, roughness: 0.95 }),
      Power: new Three.MeshStandardMaterial({ color: 0x344b59, roughness: 0.55, metalness: 0.1 }),
    };
    new GLTFLoader().load("/Cad/XLeRobotBody.glb", Gltf => {
      if (Disposed) { Gltf.scene.traverse(Object => { if (Object instanceof Three.Mesh) Object.geometry.dispose(); }); return; }
      try {
        for (const Part of XLeRobotScene.Parts) {
          const Source = Gltf.scene.getObjectByName(Part.Name) as Three.Mesh | undefined;
          if (!Source?.geometry) throw new Error(`Missing XLeRobot part ${Part.Name}`);
          // Smooth tessellation seams on curved CAD surfaces while retaining
          // deliberate edges. Vertex positions and collision geometry are unchanged.
          const SmoothGeometry = Part.Category === "Cart" ? toCreasedNormals(Source.geometry, Math.PI / 3) : Source.geometry;
          if (SmoothGeometry !== Source.geometry) Source.geometry.dispose();
          const Mesh = new Three.Mesh(SmoothGeometry, BodyMaterials[Part.Material]);
          Mesh.name = Part.Name; Mesh.matrixAutoUpdate = false; Mesh.castShadow = true; Mesh.receiveShadow = true; Body.add(Mesh);
        }
        Gltf.scene.traverse(Object => { if (Object instanceof Three.Mesh) for (const Material of Array.isArray(Object.material) ? Object.material : [Object.material]) Material.dispose(); });
        Canvas.dataset.assemblyReady = "true"; Canvas.dataset.bodyParts = String(Body.children.length);
        Renderer.shadowMap.needsUpdate = true; SetBodyLoaded(true); Fit();
      } catch (Error) { SetFailure(String(Error)); }
    }, undefined, () => { if (!Disposed) SetFailure("The full robot model could not load. Reload to try again."); });
    const Scale = new Three.Matrix4().makeScale(100, 100, 100);
    new GLTFLoader().load("/Cad/SO101Parts.glb", Gltf => {
      if (Disposed) { Gltf.scene.traverse(Object => { if (Object instanceof Three.Mesh) Object.geometry.dispose(); }); return; }
      const BuildInstance = (Ghost: Three.Material | null, IsParked = false): CadInstance => {
        const Root = new Three.Group(); Root.matrixAutoUpdate = false; Scene.add(Root);
        Root.name = IsParked ? "InactiveArm" : Ghost === GhostMaterial ? "TargetPosePreview" : Ghost ? "AiPosePreview" : "ControlledArm";
        const Parts: CadInstance["Parts"] = [];
        for (const Visual of CadModel.Visuals) {
          const Source = Gltf.scene.getObjectByName(Visual.Name) as Three.Mesh | undefined;
          if (!Source?.geometry) throw new Error(`Missing CAD part ${Visual.Name}`);
          const Mesh = new Three.Mesh(Source.geometry, Ghost ?? (IsParked
            ? Visual.Servo ? ParkedServoMaterial : ParkedMaterial
            : Visual.Servo ? ServoMaterial : ShellMaterial));
          Mesh.name = Visual.Name; Mesh.matrixAutoUpdate = false; Mesh.castShadow = !Ghost; Mesh.receiveShadow = !Ghost; Mesh.renderOrder = Ghost ? 2 : 0;
          Mesh.userData.Joint = CadModel.Joints.find(Joint => Joint.Child === Visual.Link)?.Name ?? "shoulder_pan";
          Mesh.userData.Servo = Visual.Servo;
          Root.add(Mesh); Parts.push({ Mesh, Link: Visual.Link });
        }
        return { Root, Parts, PoseKey: "", TransformKey: "", State: null };
      };
      try {
        Current = BuildInstance(null); Preview = BuildInstance(GhostMaterial); Proposal = BuildInstance(ProposalMaterial); Parked = BuildInstance(null, true);
        Gltf.scene.traverse(Object => { if (Object instanceof Three.Mesh) for (const Material of Array.isArray(Object.material) ? Object.material : [Object.material]) Material.dispose(); });
        Canvas.dataset.cadReady = "true"; Canvas.dataset.cadParts = String(Current.Parts.length);
        Renderer.shadowMap.needsUpdate = true; SetLoaded(true); Fit();
      } catch (Error) { SetFailure(String(Error)); }
    }, undefined, () => { if (!Disposed) SetFailure("CAD parts could not load. Reload to try again."); });
    function SetPose(Instance: CadInstance | null, Angles: CadAngles, Offset: Three.Vector3, UrdfPose = false) {
      if (!Instance) return;
      const PoseKey = `${UrdfPose ? "urdf" : GetCadAlignment().Version}:${CadJointNames.map(Name => Angles[Name] ?? 0).join(",")}`;
      const TransformKey = `${PoseKey}:${Offset.x},${Offset.y},${Offset.z}`;
      if (Instance.TransformKey === TransformKey) return;
      if (Instance.PoseKey !== PoseKey) {
        Instance.State = UrdfPose ? CadForwardUrdf(Angles) : CadForward(Angles); Instance.PoseKey = PoseKey;
      }
      const State = Instance.State!;
      const Translation = new Three.Matrix4().makeTranslation(Offset.x, Offset.y, Offset.z);
      for (const Part of Instance.Parts) {
        Part.Mesh.matrix.copy(Translation).multiply(State.Links[Part.Link]).multiply(Scale); Part.Mesh.matrixWorldNeedsUpdate = true;
      }
      Instance.TransformKey = TransformKey;
    }
    function Fit(View = "iso") {
      FreezeCamera();
      Draw();
      const Bounds = Current ? new Three.Box3().setFromObject(Current.Root) : new Three.Box3().setFromPoints(Latest.current.joints.map(Point => new Three.Vector3(Point.x, Point.y, Point.z)));
      if (!Options.current.Reference) {
        if (Body.visible) Bounds.union(BodyBounds(Body.position));
        if (Parked?.Root.visible) Bounds.union(new Three.Box3().setFromObject(Parked.Root));
        if (Preview?.Root.visible) Bounds.union(new Three.Box3().setFromObject(Preview.Root));
        if (Proposal?.Root.visible) Bounds.union(new Three.Box3().setFromObject(Proposal.Root));
      }
      Controls.target.copy(FitAssemblyCamera(Camera, Bounds, View)); Controls.update();
      Canvas.dataset.viewScope = Options.current.Reference ? "Reference" : "Robot";
      Invalidate();
    }
    function Draw() {
      if (ViewCubeBody.current) ViewCubeBody.current.style.transform = `matrix3d(${ViewCubeRotation(Camera).elements.join(",")})`;
      const Values = Latest.current;
      const View = Values.Settings;
      const PixelRatio = View.RenderQuality === "Ultra" ? Math.min(3, Math.max(2, window.devicePixelRatio || 1)) : Math.min(window.devicePixelRatio || 1, 2);
      if (Renderer.getPixelRatio() !== PixelRatio) Renderer.setPixelRatio(PixelRatio);
      Canvas.dataset.renderQuality = View.RenderQuality;
      Controls.zoomSpeed = View.ZoomSpeed; Controls.rotateSpeed = View.OrbitSpeed; Controls.panSpeed = View.PanSpeed;
      Renderer.shadowMap.enabled = View.Shadows;
      const SceneBase = Values.joints[0] ?? LocalBase;
      const Offset = new Three.Vector3(SceneBase.x - LocalBase.x, SceneBase.y - LocalBase.y, SceneBase.z - LocalBase.z); SceneOffset.copy(Offset);
      const Placement = AssemblyPlacement(SceneBase, Options.current.LiveSide);
      OriginGroup.position.copy(Placement.Origin); OriginGroup.visible = !Options.current.Reference && View.ShowGraph;
      OriginDot.visible = View.ShowOrigin;
      AxisMaterials.forEach(Material => { Material.linewidth = View.AxisWidth; });
      const LabelScale = 2 * Math.tan(Camera.fov * Math.PI / 360) / Math.max(Element!.clientHeight, 1);
      OriginLabels.forEach(Label => {
        Label.scale.set(Label.userData.WidthPixels * LabelScale, 14 * LabelScale, 1);
        Label.visible = View.CoordinateLabels === "All" || View.CoordinateLabels === "PositiveY" && Label.userData.Axis === "+Y";
      });
      Canvas.dataset.workspaceOrigin = JSON.stringify(Placement.Origin.toArray());
      Body.position.copy(Placement.BodyOffset); Body.visible = !Options.current.Reference && View.ShowBody;
      SetPose(Current, Options.current.Reference ? {} : Values.angles, Offset, Options.current.Reference);
      if (Current) for (const Part of Current.Parts) {
        const Selected = Values.SelectedJoint === Part.Mesh.userData.Joint && !Options.current.Reference;
        Part.Mesh.material = Selected ? Part.Mesh.userData.Servo ? SelectedServo : SelectedShell : Part.Mesh.userData.Servo ? ServoMaterial : ShellMaterial;
      }
      JointGuide.visible = !!Values.SelectedJoint && !Options.current.Reference;
      if ((Values.SelectedJoint ?? "") !== JointSelection) { JointSelection = Values.SelectedJoint ?? ""; SelectionAt = performance.now(); }
      if (Values.SelectedJoint) {
        const Servo = Values.Servos[Values.SelectedJoint];
        // While the joint is being dragged the knob shows the dragged target, not an older one echoed back.
        const TargetRaw = JointDrag?.Active && JointDrag.Joint === Values.SelectedJoint ? JointDrag.Raw : Servo.targetRaw;
        const NextKey = JSON.stringify([Values.SelectedJoint, Values.angles, Servo.min, Servo.max, Servo.arcInverted, Servo.presentRaw, TargetRaw]);
        if (NextKey !== JointGuideKey) {
          const Arc = JointArc(Values.angles, { ...Servo, targetRaw: TargetRaw });
          const Center = Arc.Center.clone().add(Offset);
          JointGuide.position.copy(Center);
          JointRingLocal = Arc.Segments.map((Value, Index) => Value - Arc.Center.getComponent(Index % 3));
          JointRing.geometry.dispose(); JointRing.geometry = new LineSegmentsGeometry().setPositions(JointRingLocal);
          JointNeedle.geometry.dispose(); JointNeedle.geometry = new LineSegmentsGeometry().setPositions([0, 0, 0, ...Arc.Current.sub(Arc.Center).toArray()]);
          JointKnobOffset.copy(Arc.Target).sub(Arc.Center);
          JointGuideKey = NextKey;
        }
        const Progress = ReducedMotion ? 1 : Math.min(1, (performance.now() - SelectionAt) / 220);
        // At least 44 screen pixels of radius, so the dial stays easy to grab
        // zoomed out; its angles, and so what a drag commands, do not change.
        const DialPixels = JointDialRadiusCm / Math.max(1e-6, Camera.position.distanceTo(JointGuide.position) * 2 * Math.tan(Camera.fov * Math.PI / 360) / Math.max(Element!.clientHeight, 1));
        JointGuide.scale.setScalar(Math.max(1, 44 / DialPixels) * (.4 + .6 * (1 - (1 - Progress) ** 3)));
        JointKnob.position.copy(JointKnobOffset).multiplyScalar(JointGuide.scale.x).add(JointGuide.position);
      }
      JointKnob.visible = JointGuide.visible && !!Values.OnJointChange;
      JointRingMaterial.linewidth = JointDrag?.Active || HoverDial ? 4.5 : 3;
      const Tone = SceneTones[Values.Mode === "Ai" ? "Ai" : "Manual"];
      if ((Scene.background as Three.Color).getHex() !== Tone.Cabin) {
        (Scene.background as Three.Color).setHex(Tone.Cabin);
        (Scene.fog as Three.Fog).color.setHex(Tone.Cabin);
        (Ground.material as Three.MeshStandardMaterial).color.setHex(Tone.Ground);
        (Grid.material as Three.LineBasicMaterial).color.setHex(Tone.Grid);
        // The arm keeps its own colours; only the selection light follows the mode.
        SelectedShell.emissive.setHex(Tone.Live);
        SelectedServo.emissive.setHex(Tone.Live);
        JointRingMaterial.color.setHex(Tone.Live);
        JointNeedleMaterial.color.setHex(Tone.Target);
        (JointKnob.material as Three.MeshBasicMaterial).color.setHex(Tone.Target);
      }
      if (!Options.current.Reference) SetPose(Parked, XLeRobotScene.ParkedUrdfRadians, Placement.OtherPan, true);
      if (Parked) Parked.Root.visible = !Options.current.Reference && View.ShowParkedArm;
      Canvas.dataset.liveSide = Options.current.LiveSide;
      if (Preview) Preview.Root.visible = !Options.current.Reference && View.ShowPosePreview && Object.keys(Values.previewAngles).some(Name => Math.abs((Values.previewAngles[Name as keyof ArmAngles] ?? 0) - (Values.angles[Name as keyof ArmAngles] ?? 0)) > 0.8);
      if (Preview?.Root.visible) SetPose(Preview, Values.previewAngles, Offset);
      GhostMaterial.color.setHex(Values.previewReachable ? Tone.Ghost : 0xe47676);
      ProposalMaterial.color.setHex(Tone.Proposal);
      if (Proposal) { Proposal.Root.visible = !Options.current.Reference && View.ShowPosePreview && !!Values.proposalAngles; if (Values.proposalAngles) SetPose(Proposal, Values.proposalAngles, Offset); }
      // Joint handles follow whichever arm they belong to, and keep a steady
      // screen size so a distant joint is still a comfortable target.
      const MarkersOn = View.ShowJointMarkers && !Options.current.Reference;
      const PixelScale = 2 * Math.tan(Camera.fov * Math.PI / 360) / Math.max(Element!.clientHeight, 1);
      for (const Marker of JointMarkers) {
        const Source = Marker.Kind === "Live" ? Current : Preview;
        const Live = Marker.Kind === "Live";
        const Visible = MarkersOn && !!Source?.State && (Live || Preview!.Root.visible);
        Marker.Group.visible = Visible;
        if (!Visible) continue;
        const Frame = Source!.State!.Frames[Marker.Joint];
        Marker.Group.position.setFromMatrixPosition(Frame).add(Offset);
        Marker.Halo.quaternion.copy(Camera.quaternion);
        const Selected = Values.SelectedJoint === Marker.Joint;
        const Hovered = HoverMarker?.Joint === Marker.Joint && HoverMarker.Kind === Marker.Kind;
        const Colour = Live ? Tone.Live : Tone.Target;
        (Marker.Core.material as Three.MeshBasicMaterial).color.setHex(Colour);
        (Marker.Halo.material as Three.MeshBasicMaterial).color.setHex(Colour);
        // The live arm reads as a filled handle, the target pose as a ring, so
        // the two are still told apart where the arms overlap.
        Marker.Core.scale.setScalar(Live ? 1 : 0.42);
        (Marker.Core.material as Three.MeshBasicMaterial).opacity = Selected || Hovered ? 1 : Live ? 0.85 : 0.75;
        (Marker.Halo.material as Three.MeshBasicMaterial).opacity = Selected ? 0.95 : Hovered ? 0.8 : Live ? 0.3 : 0.62;
        // 13 screen pixels of radius at rest, more once a handle is live.
        const Size = Camera.position.distanceTo(Marker.Group.position) * PixelScale * 16;
        Marker.Group.scale.setScalar(Math.max(0.5, Size) * (Selected ? 1.35 : Hovered ? 1.18 : 1));
      }
      Canvas.dataset.jointMarkers = String(MarkersOn ? JointMarkers.filter(Marker => Marker.Group.visible).length : 0);
      Canvas.dataset.hoveredJoint = HoverMarker ? `${HoverMarker.Kind}:${HoverMarker.Joint}` : HoverDial && Values.SelectedJoint ? `Dial:${Values.SelectedJoint}` : "";
      if (JointKnob.visible) {
        // Nine screen pixels of radius at rest, larger under the cursor and while turning.
        const Size = Camera.position.distanceTo(JointKnob.position) * PixelScale * (JointDrag?.Active ? 12 : HoverDial ? 11 : 9);
        JointKnob.scale.setScalar(Math.max(0.3, Size));
        (JointKnob.material as Three.MeshBasicMaterial).opacity = JointDrag?.Active || HoverDial ? 1 : 0.9;
      }
      Canvas.dataset.jointDrag = JointDrag?.Active ? JointDrag.Joint : "";
      Canvas.dataset.jointTurnable = JointKnob.visible ? Values.SelectedJoint ?? "" : "";
      // Full view ground follows the cart wheels; reference view keeps the arm mounting plane.
      const BasePlane = Options.current.Reference ? SceneBase.z + CadModel.CadToModel[2][3] : Placement.FloorZ;
      Ground.position.set(Body.position.x, Body.position.y, BasePlane - 0.06); Grid.position.set(Body.position.x, Body.position.y, BasePlane - 0.04);
      Grid.visible = View.ShowGraph && View.ShowGrid;
      (Grid.material as Three.Material).opacity = View.GridOpacity;
      Key.target.position.set(Body.position.x, Body.position.y, BasePlane + 55);
      Key.position.set(Body.position.x - 110, Body.position.y + 140, BasePlane + 240);
      Rim.position.set(Body.position.x + 80, Body.position.y - 70, BasePlane + 150);
      const NextShadowKey = `${Current?.TransformKey}|${Parked?.TransformKey}|${Options.current.Reference}|${View.ShowBody}|${View.ShowParkedArm}|${View.Shadows}`;
      if (ShadowKey !== NextShadowKey) { Renderer.shadowMap.needsUpdate = true; ShadowKey = NextShadowKey; }
      const ViewDistance = Camera.position.distanceTo(Controls.target);
      (Scene.fog as Three.Fog).near = ViewDistance + 200; (Scene.fog as Three.Fog).far = ViewDistance + 750;
      TargetGroup.visible = !Options.current.Reference && Values.TargetVisible !== false; TargetGroup.position.set(Values.target.x, Values.target.y, Values.target.z);
      AxisGuide.visible = !!Values.SelectedAxis;
      AxisGuide.rotation.set(0, 0, 0);
      if (Values.SelectedAxis === "y") AxisGuide.rotation.z = Math.PI / 2;
      if (Values.SelectedAxis === "z") AxisGuide.rotation.y = -Math.PI / 2;
      AxisGuideMaterial.color.setHex(Values.SelectedAxis === "y" ? 0x85d999 : Values.SelectedAxis === "z" ? 0x8db8ff : 0xf18b87);
      Halo.scale.setScalar(Values.SelectedAxis ? 1.55 : 1); TargetBall.scale.setScalar(Values.SelectedAxis ? 1.35 : 1);
      Canvas.dataset.selectedJoint = Values.SelectedJoint ?? ""; Canvas.dataset.selectedAxis = Values.SelectedAxis ?? "";
      Arrow.visible = View.ShowApproachRay;
      const Flash = !Values.TargetReadOnly && Values.targetEditing && !Values.previewReachable;
      const TargetColor = Values.previewReachable ? Tone.Target : Flash && Math.floor(performance.now() / 140) % 2 ? 0x8c2020 : 0xff4545;
      TargetMaterial.color.setHex(TargetColor); Arrow.setColor(TargetColor);
      Canvas.dataset.targetState = Values.targetSnapping ? "snapping" : Values.targetEditing ? Values.previewReachable ? "drag-valid" : "drag-invalid" : Values.previewReachable ? "valid" : "invalid";
      Canvas.dataset.targetColor = TargetMaterial.color.getHexString();
      const NextApproachKey = `${GetCadAlignment().Version}:${CadJointNames.map(Name => Values.previewAngles[Name]).join(",")}:${Values.target.wristPitch}`;
      if (ApproachKey !== NextApproachKey) {
        const Direction = TargetApproachDirection(Values.previewAngles, Values.target.wristPitch);
        Arrow.position.copy(Direction).multiplyScalar(-ArrowBehindCm); Arrow.setDirection(Direction);
        ApproachKey = NextApproachKey;
      }
      Halo.quaternion.copy(Camera.quaternion);
      if (Values.pitchGuide !== AppliedPitchGuide) {
        AppliedPitchGuide = Values.pitchGuide;
        if (AppliedPitchGuide?.length) {
          const Positions = AppliedPitchGuide.flatMap(Segment => [Segment.Start, Segment.End]
            .flatMap(Point => Point.Direction.map(Value => -Value * PitchGuideRadiusCm)));
          PitchArc.geometry.dispose(); PitchArc.geometry = new LineSegmentsGeometry().setPositions(Positions);
        }
      }
      PitchArc.visible = View.ShowAngleGuide && !!Values.pitchGuide?.length;
      Canvas.dataset.pitchGuideState = Values.pitchGuide === null ? "pending" : Values.pitchGuide.length ? "ready" : "empty";
      Canvas.dataset.pitchGuideSegments = String(Values.pitchGuide?.length ?? 0);
      ObjectBoxMesh.visible = !!Values.objectBox && !Options.current.Reference;
      if (Values.objectBox) {
        ObjectBoxMesh.position.set(Values.objectBox.Center.x, Values.objectBox.Center.y, Values.objectBox.Center.z);
        ObjectBoxMesh.scale.set(...Values.objectBox.SizeCm);
      }
      Estimate.visible = !!Values.targetEstimate && !ObjectBoxMesh.visible && !Options.current.Reference;
      if (Values.targetEstimate) Estimate.position.set(Values.targetEstimate.x, Values.targetEstimate.y, Values.targetEstimate.z);
      DimensionLines.visible = Options.current.Measure;
      if (Options.current.Measure && Current && MeasureLabels.current) {
        const State = Current.State!;
        const Points = State.Joints.map(Point => new Three.Vector3(Point.x, Point.y, Point.z).add(Offset));
        if (MeasurementKey !== Current.TransformKey) {
          DimensionLines.geometry.setFromPoints([Points[1], Points[2], Points[2], Points[3]]); MeasurementKey = Current.TransformKey;
        }
        [...MeasureLabels.current.children].forEach((Label, Index) => {
          const Position = Points[Index + 1].clone().lerp(Points[Index + 2], 0.5).project(Camera);
          (Label as HTMLElement).style.transform = `translate(${(Position.x * 0.5 + 0.5) * Element!.clientWidth + (Index ? 50 : -50)}px, ${(-Position.y * 0.5 + 0.5) * Element!.clientHeight + (Index ? -30 : 25)}px) translate(-50%, -50%)`;
        });
      }
      Renderer.render(Scene, Camera);
      Canvas.dataset.draws = String(++Draws);
      Canvas.dataset.camera = [...Camera.position.toArray(), ...Controls.target.toArray(), Camera.zoom].map(Value => Value.toFixed(4)).join(",");
    }
    let FirstResize = true;
    const Resize = new ResizeObserver(() => {
      const Width = Math.max(Element.clientWidth, 1), Height = Math.max(Element.clientHeight, 1);
      Renderer.setSize(Width, Height); Camera.aspect = Width / Height; Camera.updateProjectionMatrix();
      // Tiling changes the viewport, not the user's chosen camera pose.
      if (FirstResize) { FirstResize = false; Fit(); } else Invalidate();
    }); Resize.observe(Element);
    const Raycaster = new Three.Raycaster();
    let DragPlane: Three.Plane | null = null, DragStart: Three.Vector3 | null = null, TargetStart: TargetPose | null = null;
    let DragPointer: number | null = null;
    let WheelActive = false, WheelPitch = 0, WheelAppliedPitch = 0, WheelTimer = 0, WheelFrame = 0;
    function PointerRay(Event: PointerEvent) {
      const Bounds = Canvas.getBoundingClientRect(); Raycaster.setFromCamera(new Three.Vector2((Event.clientX - Bounds.left) / Bounds.width * 2 - 1, -(Event.clientY - Bounds.top) / Bounds.height * 2 + 1), Camera);
    }
    /* Joint handles are picked by screen distance, not by their geometry: a
       handle drawn at 13 pixels is still comfortable to hit at 22. */
    function MarkerAt(ClientX: number, ClientY: number) {
      const Bounds = Canvas.getBoundingClientRect();
      let Closest: JointMarker | null = null, Nearest = 22;
      for (const Marker of JointMarkers) {
        if (!Marker.Group.visible) continue;
        const Screen = Marker.Group.position.clone().project(Camera);
        if (Screen.z < -1 || Screen.z > 1) continue;
        const Distance = Math.hypot(ClientX - Bounds.left - (Screen.x * 0.5 + 0.5) * Bounds.width,
          ClientY - Bounds.top - (-Screen.y * 0.5 + 0.5) * Bounds.height);
        if (Distance < Nearest) { Nearest = Distance; Closest = Marker; }
      }
      return Closest;
    }
    /* The selected joint's dial, where that joint can be commanded: its knob
       within 16 pixels, or anywhere on its arc within 10. */
    function DialAt(ClientX: number, ClientY: number): "Knob" | "Ring" | null {
      if (!JointKnob.visible || !Latest.current.SelectedJoint || Options.current.Reference) return null;
      const Bounds = Canvas.getBoundingClientRect(), X = ClientX - Bounds.left, Y = ClientY - Bounds.top;
      const Screen = (Point: Three.Vector3) => { const Projected = Point.project(Camera);
        return Projected.z < -1 || Projected.z > 1 ? null : [(Projected.x * .5 + .5) * Bounds.width, (-Projected.y * .5 + .5) * Bounds.height]; };
      const Knob = Screen(JointKnob.position.clone());
      if (Knob && Math.hypot(X - Knob[0], Y - Knob[1]) <= 16) return "Knob";
      const Scale = JointGuide.scale.x, Point = (Index: number) => Screen(new Three.Vector3(JointRingLocal[Index], JointRingLocal[Index + 1], JointRingLocal[Index + 2]).multiplyScalar(Scale).add(JointGuide.position));
      for (let Index = 0; Index + 5 < JointRingLocal.length; Index += 6) {
        const Start = Point(Index), End = Point(Index + 3);
        if (!Start || !End) continue;
        const Dx = End[0] - Start[0], Dy = End[1] - Start[1], Length = Dx * Dx + Dy * Dy;
        const Along = Length ? Math.max(0, Math.min(1, ((X - Start[0]) * Dx + (Y - Start[1]) * Dy) / Length)) : 0;
        if (Math.hypot(X - Start[0] - Along * Dx, Y - Start[1] - Along * Dy) <= 10) return "Ring";
      }
      return null;
    }
    function UpdateCursor() {
      const Values = Latest.current;
      const Turnable = HoverDial || HoverMarker?.Kind === "Live" && HoverMarker.Joint === Values.SelectedJoint && !!Values.OnJointChange;
      Canvas.style.cursor = JointDrag?.Active ? "grabbing" : Turnable ? "grab" : HoverMarker ? "pointer" : "";
    }
    function SetHover(Marker: JointMarker | null, Dial = false) {
      if (HoverMarker === Marker && HoverDial === Dial) return;
      HoverMarker = Marker; HoverDial = Dial;
      UpdateCursor();
      Invalidate();
    }
    function FreezeCamera() {
      // Consume residual orbit/pan damping while preserving the camera's current
      // pose. Disabling controls alone does not stop update() applying inertia.
      const Position = Camera.position.clone(), Center = Controls.target.clone(), Zoom = Camera.zoom;
      const Damping = Controls.enableDamping;
      Controls.enableDamping = false; Controls.update();
      Camera.position.copy(Position); Controls.target.copy(Center); Camera.zoom = Zoom; Camera.updateProjectionMatrix();
      Controls.update(); Controls.enableDamping = Damping;
    }
    /* The target ball keeps the top of the pick order: it is dragged constantly
       and a joint handle often sits right behind it. */
    function TookHandle(Event: PointerEvent) {
      const Handle = MarkerAt(Event.clientX, Event.clientY);
      if (!Handle) return false;
      Latest.current.OnSelectJoint(Latest.current.SelectedJoint === Handle.Joint ? null : Handle.Joint);
      Event.preventDefault(); Event.stopImmediatePropagation();
      return true;
    }
    /* Pick order below the target ball: the selected joint's knob, the handles
       (the selected joint's own live handle turns it when dragged, and still
       clears the selection when only clicked), then the selected joint's arc. */
    function TookJoint(Event: PointerEvent) {
      const Values = Latest.current, Dial = DialAt(Event.clientX, Event.clientY), Handle = MarkerAt(Event.clientX, Event.clientY);
      const OwnHandle = !!Handle && Handle.Kind === "Live" && Handle.Joint === Values.SelectedJoint && !!Values.OnJointChange;
      // Zoomed far out the knob nears its own hub: the nearer of the two wins.
      const Away = (Point: Three.Vector3) => { const Bounds = Canvas.getBoundingClientRect(), Projected = Point.clone().project(Camera);
        return Math.hypot(Event.clientX - Bounds.left - (Projected.x * .5 + .5) * Bounds.width, Event.clientY - Bounds.top - (-Projected.y * .5 + .5) * Bounds.height); };
      if (Dial === "Knob" && !(OwnHandle && Away(Handle!.Group.position) < Away(JointKnob.position))) return BeginJointDrag(Event, Values.SelectedJoint!, false);
      if (OwnHandle) return BeginJointDrag(Event, Handle!.Joint, true);
      if (TookHandle(Event)) return true;
      if (Dial === "Ring") return BeginJointDrag(Event, Values.SelectedJoint!, false);
      return false;
    }
    /* A joint drag turns the dial by the pointer's movement from where it was
       grabbed, never to where it was grabbed, so pressing alone never moves the
       arm. Nothing is commanded until the pointer has moved 4 pixels. */
    function BeginJointDrag(Event: PointerEvent, Joint: ServoName, ClearOnClick: boolean) {
      const Bounds = Canvas.getBoundingClientRect(), X = Event.clientX - Bounds.left, Y = Event.clientY - Bounds.top;
      const Frame = new Three.Matrix4().makeTranslation(SceneOffset.x, SceneOffset.y, SceneOffset.z).multiply(CadForward(Latest.current.angles).Frames[Joint]);
      const StartRaw = Latest.current.Servos[Joint].targetRaw;
      // The dial as drawn: the camera holds still for the whole drag, so its on-screen size does too.
      const Radius = JointDialRadiusCm * (Latest.current.SelectedJoint === Joint && JointGuide.visible ? JointGuide.scale.x : 1);
      JointDrag = { Joint, Pointer: Event.pointerId, Frame, Radius, StartRaw, Raw: StartRaw, Turn: 0, StartX: X, StartY: Y, LastX: X, LastY: Y, Active: false, ClearOnClick };
      FreezeCamera(); Controls.enabled = false;
      Event.preventDefault(); Event.stopImmediatePropagation();
      Canvas.setPointerCapture(Event.pointerId);
      return true;
    }
    function MoveJointDrag(Event: PointerEvent) {
      const Drag = JointDrag;
      if (!Drag || Event.pointerId !== Drag.Pointer) return;
      const Bounds = Canvas.getBoundingClientRect(), X = Event.clientX - Bounds.left, Y = Event.clientY - Bounds.top;
      if (!Drag.Active) {
        if (Math.hypot(X - Drag.StartX, Y - Drag.StartY) < 4) return;
        Drag.Active = true; SetJointDragging(true); UpdateCursor();
      }
      const Servo = Latest.current.Servos[Drag.Joint];
      const Dial: JointDial = { Frame: Drag.Frame, Radius: Drag.Radius, Angle: JointAxisAngle(Servo, Drag.StartRaw) + Drag.Turn };
      const Turned = TurnJoint(Servo, Drag.StartRaw, Drag.Turn + DialTurn(Dial, Camera, Bounds.width, Bounds.height, [Drag.LastX, Drag.LastY], [X, Y]));
      Drag.LastX = X; Drag.LastY = Y; Drag.Turn = Turned.Turn;
      if (Turned.Raw !== Drag.Raw) { Drag.Raw = Turned.Raw; Latest.current.OnJointChange?.(Drag.Joint, Turned.Raw); }
      Invalidate();
    }
    function EndJointDrag(Cancelled: boolean) {
      const Drag = JointDrag;
      if (!Drag) return;
      JointDrag = null; Controls.enabled = true;
      if (Canvas.hasPointerCapture(Drag.Pointer)) Canvas.releasePointerCapture(Drag.Pointer);
      if (Drag.Active) SetJointDragging(false);
      else if (Drag.ClearOnClick && !Cancelled) Latest.current.OnSelectJoint(null);
      UpdateCursor(); Invalidate();
    }
    function PointerDown(Event: PointerEvent) {
      if (Event.button !== 0 || DragPlane || JointDrag || Options.current.Reference) return;
      PointerRay(Event); const ScreenTarget = TargetGroup.position.clone().project(Camera), Bounds = Canvas.getBoundingClientRect();
      if (Math.hypot(Event.clientX - Bounds.left - (ScreenTarget.x * 0.5 + 0.5) * Bounds.width, Event.clientY - Bounds.top - (-ScreenTarget.y * 0.5 + 0.5) * Bounds.height) > 22) {
        if (TookJoint(Event)) return;
        const Pickable = [...(Current?.Parts.map(Part => Part.Mesh) ?? []), ...(Body.visible ? Body.children : []),
          ...(Parked?.Root.visible ? Parked.Parts.map(Part => Part.Mesh) : [])];
        const Hit = Raycaster.intersectObjects(Pickable, false)[0];
        const Name = Hit && Current?.Parts.some(Part => Part.Mesh === Hit.object) ? Hit.object.userData.Joint : null;
        if (Name && CadJointNames.includes(Name)) { Latest.current.OnSelectJoint(Name); Event.preventDefault(); Event.stopImmediatePropagation(); }
        return;
      }
      if (Latest.current.TargetReadOnly || !TargetGroup.visible) { TookJoint(Event); return; }
      Latest.current.OnSelectJoint(null);
      FreezeCamera();
      // Freeze the screen plane through the target at grab time. Keeping the
      // initial ray hit preserves the cursor's pickup offset without a jump.
      const Plane = new Three.Plane().setFromNormalAndCoplanarPoint(Camera.getWorldDirection(new Three.Vector3()), TargetGroup.position);
      PointerRay(Event);
      const Hit = Raycaster.ray.intersectPlane(Plane, new Three.Vector3());
      if (!Hit) return;
      Event.preventDefault(); Event.stopImmediatePropagation(); Controls.enabled = false;
      DragPlane = Plane; DragStart = Hit; TargetStart = { ...Latest.current.target }; DragPointer = Event.pointerId;
      Latest.current.onTargetDragStart(); Canvas.setPointerCapture(Event.pointerId);
    }
    function PointerMove(Event: PointerEvent) {
      if (JointDrag) { MoveJointDrag(Event); return; }
      if (!DragPlane && !WheelActive) {
        const Screen = TargetGroup.position.clone().project(Camera), Area = Canvas.getBoundingClientRect();
        const OverTarget = TargetGroup.visible && Math.hypot(Event.clientX - Area.left - (Screen.x * 0.5 + 0.5) * Area.width,
          Event.clientY - Area.top - (-Screen.y * 0.5 + 0.5) * Area.height) <= 22;
        const Dial = Options.current.Reference || OverTarget ? null : DialAt(Event.clientX, Event.clientY);
        const Marker = Options.current.Reference || OverTarget || Dial === "Knob" ? null : MarkerAt(Event.clientX, Event.clientY);
        SetHover(Marker, Dial === "Knob" || !Marker && Dial === "Ring");
      }
      if (!DragPlane || !DragStart || !TargetStart || Event.pointerId !== DragPointer) return;
      PointerRay(Event); const Hit = Raycaster.ray.intersectPlane(DragPlane, new Three.Vector3()); if (!Hit) return;
      const Delta = Hit.sub(DragStart); Latest.current.onTargetChange({ ...TargetStart, x: TargetStart.x + Delta.x, y: TargetStart.y + Delta.y, z: TargetStart.z + Delta.z });
    }
    function EndDrag(Cancelled: boolean) {
      if (!DragPlane) return;
      const Pointer = DragPointer;
      DragPlane = null; DragStart = null; TargetStart = null; DragPointer = null; Controls.enabled = true;
      if (Pointer !== null && Canvas.hasPointerCapture(Pointer)) Canvas.releasePointerCapture(Pointer);
      Latest.current.onTargetDragEnd(Cancelled); Invalidate();
    }
    function PointerUp(Event: PointerEvent) {
      if (JointDrag && Event.pointerId === JointDrag.Pointer) {
        if (Event.type === "pointerup" && Event.button !== 0) return;
        if (Event.type === "pointerup") MoveJointDrag(Event);
        EndJointDrag(Event.type !== "pointerup"); return;
      }
      if (Event.pointerId !== DragPointer || (Event.type === "pointerup" && Event.button !== 0)) return;
      if (Event.type === "pointerup") PointerMove(Event);
      EndDrag(Event.type !== "pointerup");
    }
    function KeyDown(Event: KeyboardEvent) {
      // Escape stops a joint drag where it is: what was already commanded stands, nothing more is sent.
      if (Event.key === "Escape" && (DragPlane || WheelActive || JointDrag)) { Event.preventDefault(); EndDrag(true); FinishWheel(true); EndJointDrag(true); }
    }
    function PointerLeave() { SetHover(null); }
    function WindowBlur() { EndDrag(true); FinishWheel(true); EndJointDrag(true); SetHover(null); }
    function ContextMenu(Event: MouseEvent) { Event.preventDefault(); }
    function ApplyWheelPitch() {
      WheelFrame = 0;
      if (!WheelActive || WheelPitch === WheelAppliedPitch) return;
      WheelAppliedPitch = WheelPitch;
      Latest.current.onTargetChange(Previous => ({ ...Previous, wristPitch: WheelPitch }));
    }
    function FinishWheel(Cancelled = false) {
      if (!WheelActive) return;
      window.clearTimeout(WheelTimer); cancelAnimationFrame(WheelFrame); WheelFrame = 0;
      if (!Cancelled) ApplyWheelPitch();
      WheelActive = false; Controls.enabled = true;
      Latest.current.onTargetDragEnd(Cancelled); Invalidate();
    }
    function FinishWheelOnPointer() { FinishWheel(); }
    function Wheel(Event: WheelEvent) {
      const Bounds = Canvas.getBoundingClientRect(), Screen = TargetGroup.position.clone().project(Camera);
      const OverTarget = Screen.z >= -1 && Screen.z <= 1 && Math.hypot(
        Event.clientX - Bounds.left - (Screen.x * .5 + .5) * Bounds.width,
        Event.clientY - Bounds.top - (-Screen.y * .5 + .5) * Bounds.height) <= 26;
      // Normal wheel over the target adjusts pitch. Empty space still zooms;
      // Shift also adjusts pitch anywhere, and Ctrl/pinch always zooms.
      if ((!Event.shiftKey && !OverTarget) || Event.ctrlKey || Event.metaKey || Options.current.Reference || Latest.current.TargetReadOnly || !TargetGroup.visible) { FinishWheel(); return; }
      Event.preventDefault(); Event.stopImmediatePropagation();
      if (DragPlane || JointDrag) return;
      const Delta = WristWheelDelta(Event, Canvas.clientHeight);
      if (!Delta) return;
      if (!WheelActive) {
        WheelActive = true; WheelPitch = WheelAppliedPitch = Latest.current.target.wristPitch;
        FreezeCamera(); Controls.enabled = false; Latest.current.onTargetDragStart("Pitch");
      }
      WheelPitch += Delta;
      if (!WheelFrame) WheelFrame = requestAnimationFrame(ApplyWheelPitch);
      window.clearTimeout(WheelTimer); WheelTimer = window.setTimeout(() => FinishWheel(), 180);
    }
    Canvas.addEventListener("pointerdown", PointerDown, true); Canvas.addEventListener("pointermove", PointerMove);
    Canvas.addEventListener("pointerleave", PointerLeave);
    Canvas.addEventListener("pointerup", PointerUp); Canvas.addEventListener("pointercancel", PointerUp);
    Canvas.addEventListener("lostpointercapture", PointerUp); window.addEventListener("keydown", KeyDown);
    window.addEventListener("blur", WindowBlur);
    window.addEventListener("pointerdown", FinishWheelOnPointer, true);
    Canvas.addEventListener("contextmenu", ContextMenu);
    Canvas.addEventListener("wheel", Wheel, { capture: true, passive: false });
    function Invalidate() {
      NeedsDraw = true;
      if (!Disposed && !Animation && !document.hidden) Animation = requestAnimationFrame(Animate);
    }
    function Animate() {
      Animation = 0;
      if (Disposed || document.hidden) return;
      const Changed = DragPlane || WheelActive || JointDrag ? false : Controls.update();
      const Flash = Latest.current.targetEditing && !Latest.current.previewReachable;
      const FlashStep = Flash ? Math.floor(performance.now() / 140) : -1;
      if (FlashStep !== LastTargetFlash) { LastTargetFlash = FlashStep; NeedsDraw = true; }
      if (NeedsDraw || Changed) { NeedsDraw = false; Draw(); }
      if (Flash || Changed && Controls.enableDamping || Latest.current.SelectedJoint && !ReducedMotion && performance.now() - SelectionAt < 240) Invalidate();
    }
    Controls.addEventListener("change", Invalidate);
    document.addEventListener("visibilitychange", Invalidate);
    Runtime.current = { Renderer, Fit, Draw, Invalidate }; Fit(); Invalidate();
    return () => {
      Disposed = true; cancelAnimationFrame(Animation); cancelAnimationFrame(WheelFrame); window.clearTimeout(WheelTimer);
      Resize.disconnect(); Controls.removeEventListener("change", Invalidate); Controls.dispose();
      document.removeEventListener("visibilitychange", Invalidate);
      Canvas.removeEventListener("pointerdown", PointerDown, true); Canvas.removeEventListener("pointermove", PointerMove);
      Canvas.removeEventListener("pointerleave", PointerLeave);
      Canvas.removeEventListener("pointerup", PointerUp); Canvas.removeEventListener("pointercancel", PointerUp); Canvas.removeEventListener("wheel", Wheel, true);
      Canvas.removeEventListener("lostpointercapture", PointerUp); window.removeEventListener("keydown", KeyDown);
      window.removeEventListener("blur", WindowBlur);
      window.removeEventListener("pointerdown", FinishWheelOnPointer, true);
      Canvas.removeEventListener("contextmenu", ContextMenu);
      const Geometries = new Set<Three.BufferGeometry>(), Materials = new Set<Three.Material>();
      Scene.traverse(Object => { if (Object instanceof Three.Mesh || Object instanceof Three.Line || Object instanceof Three.Sprite) {
        if (!(Object instanceof Three.Sprite)) Geometries.add(Object.geometry); for (const Material of Array.isArray(Object.material) ? Object.material : [Object.material]) Materials.add(Material);
      } });
      Geometries.forEach(Geometry => Geometry.dispose()); Materials.forEach(Material => Material.dispose());
      MarkerCore.dispose(); MarkerHalo.dispose();
      SelectedShell.dispose(); SelectedServo.dispose(); ShellMaterial.dispose(); ServoMaterial.dispose();
      LabelTextures.forEach(Texture => Texture.dispose());
      EnvironmentMap.dispose(); Renderer.dispose(); Renderer.forceContextLoss(); Canvas.remove(); Runtime.current = null;
    };
  }, []);
  UseEffect(() => { Runtime.current?.Invalidate(); }, [Props.angles, Props.previewAngles, Props.proposalAngles,
    Props.joints, Props.target, Props.TargetVisible, Props.TargetReadOnly, Props.previewReachable, Props.targetEditing, Props.targetSnapping, Props.pitchGuide, Props.targetEstimate, Props.objectBox, Props.Settings, Props.SelectedJoint, Props.SelectedAxis, Props.Servos, Props.Mode, Reference, Measure, LiveSide]);
  const ToggleReference = () => { SetReference(Value => !Value); window.setTimeout(() => { Runtime.current?.Draw(); Runtime.current?.Fit(); }, 0); };
  return <div className="canvas-shell cad-shell">
    <div ref={Host} className="cad-host" />
    <div className="cad-tools" aria-label="CAD view controls">
      <button className={`cad-alignment ${Reference ? "active" : Props.alignmentRecorded ? "recorded" : ""}`} onClick={ToggleReference}
        title={Reference ? "Return to live pose" : "CAD reference alignment"} aria-label={Reference ? "Return to live pose" : "CAD reference alignment"} aria-pressed={Reference}><TargetIcon size={16} /></button>
      <button onClick={() => Runtime.current?.Fit()} title="Fit model in view" aria-label="Fit model in view"><Expand size={16} /></button>
      <button className={Measure ? "active" : ""} onClick={() => SetMeasure(Value => !Value)} title="Show axis spacing" aria-label="Show axis spacing"><Ruler size={16} /></button>
      <button title="Full screen" aria-label="Full screen" onClick={() => {
        const Shell = Host.current?.parentElement;
        if (document.fullscreenElement) void document.exitFullscreen(); else void Shell?.requestFullscreen();
      }}><Maximize2 size={16} /></button>
    </div>
    <div ref={MeasureLabels} className="cad-measurements" hidden={!Measure}>{["elbow_flex", "wrist_flex"].map(Name => {
      const Joint = CadModel.Joints.find(Joint => Joint.Name === Name)!;
      return <span key={Name}>{(Math.hypot(...Joint.Origin.slice(0, 3).map(Row => Row[3])) * 10).toFixed(2)} mm <small>axis spacing</small></span>;
    })}</div>
    <DraggableViewCube BodyRef={ViewCubeBody} Hidden={!Props.Settings.ShowViewCube} OnView={View => Runtime.current?.Fit(View)} />
    {(!Loaded || !BodyLoaded) && !Failure && <div className="cad-loading"><Box size={28} /><strong>Loading robot assembly</strong><span>Cart, body, head and SO-101 arms</span></div>}
    {Failure && <div className="cad-loading cad-error"><strong>CAD view unavailable</strong><span>{Failure}</span></div>}
    {Reference && <div className="cad-reference-card"><strong>Match this reference pose</strong>
      <p>This is the original CAD zero pose. With the arm supported and torque off, align every joint, wrist rotation, and gripper to this model. Recording reads the encoders without moving the arm.</p>
      <button disabled={!Props.canCaptureAlignment || Recording} onClick={async () => {
        SetRecording(true); try { if (await Props.onCaptureAlignment()) SetReference(false); } finally { SetRecording(false); }
      }}><TargetIcon size={15} />{Recording ? "Recording..." : "I've matched this pose, record alignment"}</button>
      {!Props.canCaptureAlignment && <small>Connect the arm and switch to Coast to record.</small>}
      <small>Workspace position and camera calibration still need a physical check after alignment.</small>
    </div>}
    {Props.SelectedJoint && <div className={`cad-selection-badge${JointDragging ? " dragging" : ""}`}><span>JOINT LINK</span><strong>{formatName(Props.SelectedJoint)}</strong><button type="button" aria-label="Clear joint selection" onClick={() => Props.OnSelectJoint(null)}>×</button>
      {Props.OnJointChange && !Reference && <em>{JointDragging ? "Turning" : "Drag the knob, ring or handle to turn"} · target {trackAngleFromRaw(Props.Servos[Props.SelectedJoint], Props.Servos[Props.SelectedJoint].targetRaw).toFixed(1)}°</em>}</div>}
    <div className="cad-footer" hidden={!Props.Settings.ShowBottomText}><div><div className="cad-legend"><span><i className="cad-live-dot" />{Reference ? "Reference pose" : "Controlled arm"}</span>
      {!Reference && Props.TargetVisible !== false && <span role="status"><i className="cad-target-dot" />{Props.targetSnapping ? "Finding nearest valid target" : Props.previewReachable ? "Target reachable" : Props.targetEditing ? "Outside workspace · release to snap" : "Target outside workspace"}</span>}
      {!Reference && Props.Settings.ShowJointMarkers && <span><i className="cad-joint-dot" />Joint handles: solid live arm, hollow target pose</span>}
      {!Reference && !Props.TargetReadOnly && <span><i style={{ background: Props.pitchGuide === null ? "#75828b" : "#59e695" }} />
        {Props.pitchGuide === null ? "Checking wrist angles" : Props.pitchGuide.length ? "Green arc: acceptable wrist angles" : "No acceptable wrist angles here"}</span>}</div>
      {!Reference && <div className="cad-context-note" hidden={!Props.Settings.ShowHints}>Origin: arm midpoint at bin rim · +Y forward<br />Body and other arm: reference poses</div>}</div>
      <div className="cad-help" hidden={!Props.Settings.ShowHints}><RotateCcw size={12} /> Left drag empty space to pan · Right drag to orbit · Scroll empty space to zoom<br />Drag target in the screen plane · Scroll target for wrist angle · Click a joint handle to select that joint, then drag its knob, ring or handle to turn it</div></div>
  </div>;
});
// Memoised: App re-renders on every keystroke in the AI box; the view re-renders, and redraws, only for its own props.
export default Memo(CadCanvas);
