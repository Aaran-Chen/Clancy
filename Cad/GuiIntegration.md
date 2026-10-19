# SO-101 CAD integration

The arm workspace renders the original SO-101 follower parts with separate articulated links, dark servo bodies, lighting, shadows, camera presets, dimension labels, and target previews. The geometry is bundled locally, so rendering does not require a CAD service.

## Source and files

Geometry and joint origins come from [TheRobotStudio SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100/tree/eecbe3e0a9ebb23e25ad7b2759b03884c6660903/Simulation/SO101), commit eecbe3e0a9ebb23e25ad7b2759b03884c6660903, using so101_new_calib.urdf. The source license is included beside the GUI assets. The local GLB contains 17 visual mesh instances; each original URDF visual transform is baked into its owning link's mesh. Joint transforms remain separate and animate the mesh instances.

- Original STEP, URDF and source meshes: ~/Downloads/SO101Cad (written by Cad/FetchSources.py).
- Shared joint transforms: back end/CadModel.json.
- Independent encoder reference: back end/CadAlignment.json.
- Browser renderer: AI Controlled WebApp/src/components/CadCanvas.tsx.
- Browser FK/IK: AI Controlled WebApp/src/lib/CadKinematics.ts.
- Backend FK/IK: back end/CadKinematics.py, selected by arm_geometry.json.
- Asset generator: Cad/PrepareGuiAssets.py. It requires the downloaded source package and the Cad virtual environment.

The original names required by URDF, the existing bridge API, React and Three.js keep their established spelling. New implementation identifiers use PascalCase.

## Kinematics

All active FK uses the product of the original joint transforms, not the former estimated line lengths. The browser and backend share the same matrices and encoder calibration. Distances are centimeters in the app and meters in the source meshes; mesh placement scales meters to centimeters exactly once.

For each revolute joint, q = (OffsetDegrees + Direction * EncoderDeltaDegrees) * pi / 180. EncoderDeltaDegrees uses the existing motor sign and configured circular encoder arc. Direction makes one raw step up turn every arm joint's URDF angle up; wrist_roll's arc is inverted in gui_limits.json, so its Direction is -1 like the other four arm joints. The child transform is T_child = T_parent * T_origin * Rz(q). The fixed gripper-frame transform supplies the tool center point. All source joint axes are local Z.

The CAD shoulder-to-elbow axis spacing is 116.0000211 mm; elbow-to-wrist-flex spacing is 135.0001852 mm. The tool center is 7.9030106 mm off the wrist-roll axis. A half-turn of that wrist therefore moves the nominal tool center 15.8060212 mm, which the old planar approximation missed. The source tool center is attached to the fixed gripper body; opening the moving jaw changes its visible mesh but does not redefine this tool center.

IK solves shoulder pan, shoulder lift, elbow flex and wrist flex for XYZ and tool pitch. Wrist roll and gripper opening remain at their current readings. Tool pitch is measured from the transformed tool Z axis relative to the pan's radial direction. This is a four-variable position/pitch solve, not arbitrary six-axis orientation control.

Analytic elbow and pan branches seed a bounded numerical refinement against the complete rigid chain. Its residual is [dx, dy, dz, 10 * wrapped_pitch_error_radians]. Converged candidates take priority; elbow preference and travel select between them. The Python backend uses its own bounded Levenberg-Marquardt with an analytic Jacobian (CadKinematics.BoundedLeastSquares); the browser uses bounded damped least squares. Both respect configured encoder limits.

Targets and pitch are not silently shifted to a convenient reachable pose. Send requires a position residual no greater than 1 mm and pitch error no greater than 0.25 degrees, then repeats that check after rounding to integer encoder steps. Existing motion preflight, load and sampled tip-height checks still run. The solver does not establish physical calibration or implement whole-arm collision detection.

## Physical reference

CAD dimensions are authoritative for the stock design; the initial encoder offsets are unverified. They preserve the prior home link orientations so the upgrade has a defined starting pose. Saving a new Home no longer changes the geometry's zero reference.

Use **Align with physical arm** in the CAD view. With the arm supported and in Coast, match every joint, wrist rotation and gripper to the displayed official zero pose. The record button reads encoder feedback and saves a separate calibration, backing up the previous reference. It sends no motor movement commands. An operator-recorded pose is not a metrology result: assess alignment using physical reference surfaces or fixtures and measure the resulting tool positions independently.

The saved workspace anchor remains tied to its original raw pose. CAD cannot determine the arm's mounting location, table height, camera intrinsics, backlash, servo compliance, or the exact contact point on a held object. Check workspace coordinates and recalibrate the camera after changing the physical reference. The GUI explicitly shows whether a reference has been recorded.

## Verification

Numerical checks cover 120 poses sampled from official joint ranges and 60 poses within this project's configured encoder limits, with varying held wrist roll. Browser FK agrees with Python at floating-point precision. Browser IK's maximum position residual was 0.000009591 mm across these 180 test poses. The backend's 60 encoder-limit cases had a maximum position residual after integer encoder rounding of 0.443739 mm. These are nominal model errors, not measured physical accuracy.

## Out of reach

A target the arm cannot reach is still previewed: the dashboard shows the pose
it would get closest in. That pose is now chosen by tip distance alone. The
solve that answers a reachable target weighs wrist pitch alongside position
(one radian of pitch counts as ten centimetres), and out of reach it would give
up position to hold the pitch, so a target dragged toward the arm could make the
previewed tip retreat from it: the operator saw the arm refusing to follow. The
wrist pitch is already chosen before this (the closest reachable one), so out of
reach the best few candidates are refined once more against position only and
the closest tip wins. Measured walking a target from y=70 cm to y=32.5 cm: the
preview no longer moves away from the target at any step, and the closest
approach improves from 3.35 cm to 2.43 cm at y=46. Reachable targets, the
planner and the calibration scan are unchanged; they never use these poses.
TestCartesianReachability covers both.

When a target is reachable but no checked route gets there, the refusal now says
so, and a refusal caused by the arm's own pose is reported as that instead of
being buried behind wording that sounded like the target was unreachable.

The source-transform check compares the exported model to the original URDF reader. The earlier independent MuJoCo comparison checked 103 poses. Additional tests cover unreachable targets, nonfinite input, fixed wrist roll, the off-axis tool point, Home independence, and reference capture with a fake motor bus.

GUI tests load all 17 parts in Edge, exercise ISO/Top/Front/reference views and dimension controls, check two desktop sizes, and verify that no hardware commands are sent. Production TypeScript/Vite build, Home regressions, motor safety, workspace sampling, marker tracking, calibration recording and calibration UI checks were also run.

The separate synthetic camera-intrinsics stress suite is not fully passing. The pre-CAD snapshot already fails its noisy case; with the CAD pose distribution the noisy and strong-distortion cases fail accuracy assertions. Mild-distortion and rendered-image cases pass. This is an outstanding camera-estimation limitation, and CAD IK residuals must not be used as evidence of camera-guided physical accuracy. Logs are in back end/TestArtifacts/CameraBaseline.log and CameraCad.log.

Run the primary CAD checks from the repository root:

```powershell
python "back end/TestCadKinematics.py"
node "AI Controlled WebApp/TestCadKinematics.mjs"
node "AI Controlled WebApp/TestFrontend.mjs"
npm run build --prefix "AI Controlled WebApp"
```

With the development web server running on port 5186, run the isolated browser test:

```powershell
node "AI Controlled WebApp/TestCadUi.mjs"
```

That browser test intercepts bridge traffic and does not connect to the physical arm.
