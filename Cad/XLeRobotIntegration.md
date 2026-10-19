# XLeRobot assembly in the GUI

The workspace opens with the complete robot: IKEA RASKOG cart, cart casters, mobile drive platform, mounting body, camera mast/head, and two stock SO-101 arms. The controlled arm uses its existing live pose and target preview. The other arm and the head are static reference poses.

Use **Robot** to frame the entire assembly, **Arm** to focus on the controlled arm, and the full-screen button for a larger inspection view. ISO, Top, Front, orbit, pan and zoom remain available. **Live arm: right/left** changes which side displays the connected arm and remembers the selection in this browser. It does not select a motor bus or change commands. Right is the initial display default.

## Geometry and provenance

The body was recovered from the existing local XLeRobot source folder (`../XLeRobot-main`, a clone beside this repository), originally from [Vector-Wangel/XLeRobot](https://github.com/Vector-Wangel/XLeRobot). The asset bundle includes its Apache 2.0 license and a manifest of source SHA-256 hashes. No network access or sibling repository is required at GUI runtime.

- Cart, casters, mounting body and camera head: simulation/Maniskill/assets/xlerobot/xlerobot.urdf and its STL meshes.
- Mobile drive platform, three omni wheels and associated hardware: simulation/mujoco/xlerobot.xml. Compiled MuJoCo mesh and world transforms preserve the source geometry's recentering.
- Both arms: the original stock SO-101 assets already used by this project's CAD kinematics. The older SO-100 arm meshes in the body repository are replaced by these SO-101 parts.
- Power pack, handle and computer: generic box shapes, with schematic dimensions and placement. They are not product CAD.

The cart and drive geometry combine the two upstream simulation references, including the cart's original casters. This is a nominal assembly for context, not a measured model of this particular physical build. Body/head poses and the other arm do not claim live telemetry. Cables and unmodeled equipment are omitted.

## Coordinates and kinematics

Body vertices are baked in centimeters, with source +X forward rotated to GUI +Y forward. The SO-101 mesh remains in meters and is scaled to centimeters by its existing renderer. These conversions occur once.

The source arm mount positions are retained. The replacement SO-101 arms face the cart's forward direction; this is a visual mounting adaptation from the older source arm frames. For the selected side, the assembly translation is:

```text
BodyOffset = ExistingLivePanPosition - BodyModelPanPosition[SelectedSide]
```

The other arm is placed at its own body-model pan position plus that translation. Thus changing display sides moves the surrounding body and parked arm, leaving the controlled arm, target values, FK/IK, workspace calibration and motor mapping unchanged. The grid follows the cart wheels for display; it does not impose a minimum tip height.

Basket avoidance now uses the conservative cart/basket envelope and the moving SO-101 CAD parts. See [BasketAvoidance.md](BasketAvoidance.md) for collision geometry, planning, execution and limitations. The renderer continues to use the CAD-based arm kinematics described in [GuiIntegration.md](GuiIntegration.md).

## Files and regeneration

- AI Controlled WebApp/src/components/CadCanvas.tsx: renderer and controls.
- AI Controlled WebApp/src/lib/XLeRobotAssembly.ts: assembly placement and perspective camera fit.
- AI Controlled WebApp/src/lib/XLeRobotScene.json: dimensions, mounts, part metadata and source hashes.
- AI Controlled WebApp/public/Cad/XLeRobotBody.glb: bundled body geometry, centimeters.
- AI Controlled WebApp/public/Cad/XLeRobotSource.json and XLeRobotLicense.txt: provenance and license.
- Cad/PrepareXLeRobotAssets.py: offline asset generator, requiring NumPy, trimesh and MuJoCo in the existing Cad virtual environment.

Regenerate from the repository root, using the existing sibling source checkout:

```powershell
& "Cad/.venv/Scripts/python.exe" "Cad/PrepareXLeRobotAssets.py"
```

New implementation identifiers use PascalCase. Existing React/Three.js methods, URDF joint names and source asset names retain their framework or file-format spelling.

## Verification

The assembly test checks all 32 body parts, both mount choices and 576 projected bounding-box corners across multiple viewport shapes and camera presets. The isolated Edge GUI test loads both arms and the body, exercises whole-robot/arm views, camera presets, dimensions, side selection, reference mode and full screen, and checks that current/target coordinates are unchanged by display-side selection. It intercepts bridge traffic and verifies no motor commands or alignment writes occur.

The existing 180-pose CAD kinematics regression, frontend regressions and production build are also checked. These validate the software model; they do not measure the physical robot.

```powershell
node "AI Controlled WebApp/TestXLeRobotAssembly.mjs"
```

With the isolated development web server on port 5186:

```powershell
node "AI Controlled WebApp/TestCadUi.mjs"
```
