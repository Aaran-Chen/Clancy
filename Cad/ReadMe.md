# SO-101 CAD reference build

Finished files and the measurement report are written to `~/Downloads/SO101Cad` (see FetchSources.py).

The reference scripts in this folder run offline. The GUI and controller now use the shared CAD geometry; see [GuiIntegration.md](GuiIntegration.md) for the renderer, IK math, physical alignment, and verification results.

Run the reference FK from the repository root. Joint inputs, when supplied as a JSON argument, use the official URDF names and radians, not the app's GUI angles or raw encoder positions.

```powershell
python Cad\ReferenceKinematics.py
```

The existing isolated environment has the validation packages. To recreate it:

```powershell
python -m venv Cad\.venv
& Cad\.venv\Scripts\python.exe -m pip install -r Cad\Requirements.txt
```

Run the independent FK tests:

```powershell
& Cad\.venv\Scripts\python.exe Cad\TestReferenceKinematics.py
```

Check the STEP solids:

```powershell
& Cad\.venv\Scripts\python.exe Cad\VerifyCad.py
```

To rebuild the deliverable, run FetchSources.py, the tests, VerifyCad.py, BuildReferenceScene.py using Blender, and BuildMeasurements.py, in that order. FetchSources.py checks original files against the pinned upstream tree in UpstreamTree.json. The Blender scene builder uses the installed Blender 5.1 executable on this PC. The final measurement builder reads the existing project's current geometry for comparison.

CadQuery's high-level import triggered a shutdown access violation in this Windows environment. The final CAD validation uses Open CASCADE directly and completes without that crash. The original assembly imports but includes invalid solids; individual printed-part STEP files pass. See the generated report for the limits of the nominal geometry reference and the remaining physical calibration work.

External API names and original upstream file/joint names keep their required spelling. New script identifiers use PascalCase.
