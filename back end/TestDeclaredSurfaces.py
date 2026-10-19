"""DeclaredSurfaces: the hand-entered surfaces besides the desk, their rules and reloading.

The web app reads the same file with the same rules (AI Controlled WebApp/TestDeclaredSurfaces.mjs
checks the two agree case by case).

Run:  python TestDeclaredSurfaces.py
"""
import json
import os
import unittest
import unittest.mock
from pathlib import Path

import DeclaredSurfaces as DS

Root = Path(__file__).resolve().parent
Scratch = Root / "TestArtifacts" / "DeclaredSurfaces"
Scratch.mkdir(parents=True, exist_ok=True)
Platform = {"Name": "box platform", "Kind": "platform", "TopZCm": 12.7, "XRangeCm": [-25.0, -3.0], "YRangeCm": [30.0, 44.0]}


class DeclaredSurfaceTests(unittest.TestCase):
    def TestTheFileIsValidAndItsExampleIsAPlatform(Self):
        # Whatever is declared for the real arm reads without a problem (the committed file declares nothing).
        Spec = json.loads((Root / "DeclaredSurfaces.json").read_text(encoding="utf-8"))
        Self.assertEqual(DS.Validate(Spec)[1], [])
        Surfaces, Problems = DS.Validate({"Surfaces": [Spec["Example"]]})
        Self.assertEqual(Problems, [])
        Self.assertEqual(Surfaces[0]["Name"], "box platform")
        Self.assertAlmostEqual(Surfaces[0]["TopZCm"] - DS.TableZCm, 15.0, places=6)

    def TestAPlatformAndAShelfAreRead(Self):
        Shelf = {**Platform, "Name": "low shelf", "Kind": "shelf", "TopZCm": -8.0, "BoardAboveZCm": 6.0, "Frame": "cart"}
        Surfaces, Problems = DS.Validate({"Surfaces": [Platform, Shelf, {**Platform, "Name": "spare", "Enabled": False}]})
        Self.assertEqual(Problems, [])
        Self.assertEqual([Surface["Name"] for Surface in Surfaces], ["box platform", "low shelf"])
        Self.assertIsNone(Surfaces[0]["BoardAboveZCm"])
        Self.assertEqual(Surfaces[1]["BoardAboveZCm"], 6.0)
        Self.assertEqual(Surfaces[0]["Frame"], "cart")

    def TestBrokenEntriesAreLeftOutWithAReason(Self):
        Cases = [
            ({**Platform, "TopZCm": -2.0}, "that is the desk"),
            ({**Platform, "TopZCm": 50.0}, "beyond the arm's reach"),
            ({**Platform, "TopZCm": "12"}, "TopZCm must be a number"),
            ({**Platform, "Kind": "table"}, "Kind must be one of platform, shelf"),
            ({**Platform, "XRangeCm": [-3.0, -25.0]}, "XRangeCm -3..-25 must run from min to max"),
            ({**Platform, "YRangeCm": [30.0]}, "YRangeCm must be [min, max]"),
            ({**Platform, "Kind": "shelf"}, "a shelf needs BoardAboveZCm"),
            ({**Platform, "Kind": "shelf", "BoardAboveZCm": 15.0}, "at least 5 cm above TopZCm 12.7"),
            ({**Platform, "BoardAboveZCm": 30.0}, "a platform has no board above"),
            ({**Platform, "Frame": "world"}, "Frame must be one of cart"),
            ({**Platform, "Name": "  "}, "Name must be 1 to 40 characters"),
            ({**Platform, "Enabled": "yes"}, "Enabled must be true or false"),
        ]
        for Entry, Why in Cases:
            with Self.subTest(Why):
                Surfaces, Problems = DS.Validate({"Surfaces": [Platform | {"Name": "kept"}, Entry]})
                Self.assertEqual([Surface["Name"] for Surface in Surfaces], ["kept"], "a valid neighbour stays")
                Self.assertEqual(len(Problems), 1)
                Self.assertIn(Why, Problems[0])
                Self.assertTrue(Problems[0].startswith("surface 2"), Problems[0])
        Surfaces, Problems = DS.Validate({"Surfaces": [Platform, {**Platform, "Name": "Box  Platform"}]})
        Self.assertEqual((len(Surfaces), Problems), (1, ["surface 2 (Box Platform): the name is used twice"]))
        Self.assertEqual(DS.Validate([]), ([], ["the file must hold a JSON object with a Surfaces list"]))
        Self.assertEqual(DS.Validate({"Surfaces": None}), ([], ["Surfaces must be a list"]))

    def TestTheFileIsReadAgainWhenItChanges(Self):
        File = Scratch / "Reload.json"
        File.write_text(json.dumps({"Surfaces": [Platform]}), encoding="utf-8")
        Self.assertEqual([Surface["Name"] for Surface in DS.Load(File)], ["box platform"])
        Before = File.stat().st_mtime_ns
        File.write_text(json.dumps({"Surfaces": [{**Platform, "Name": "moved platform", "TopZCm": 10.0}]}), encoding="utf-8")
        if File.stat().st_mtime_ns == Before:  # a coarse clock: make the change visible
            os.utime(File, ns=(Before + 1_000_000, Before + 1_000_000))
        Self.assertEqual([(Surface["Name"], Surface["TopZCm"]) for Surface in DS.Load(File)], [("moved platform", 10.0)])
        File.write_text("{not json", encoding="utf-8")
        os.utime(File, ns=(Before + 2_000_000, Before + 2_000_000))
        Self.assertEqual(DS.Load(File), [], "an unreadable file declares nothing")
        Self.assertIn("unreadable", DS.Problems(File)[0])
        Self.assertEqual(DS.Load(Scratch / "Missing.json"), [])
        Self.assertEqual(DS.Problems(Scratch / "Missing.json"), [])

    def TestTheBridgeReportsTheDeclaredSurfaces(Self):
        import bridge_server as Bridge
        File = Scratch / "Bridge.json"
        File.write_text(json.dumps({"Surfaces": [Platform, {**Platform, "Name": "bad", "TopZCm": -2.0}]}), encoding="utf-8")
        with unittest.mock.patch.object(DS, "SpecPath", File):
            Reply = Bridge.declared_surfaces()
        Self.assertEqual([Surface["Name"] for Surface in Reply["surfaces"]], ["box platform"])
        Self.assertEqual(len(Reply["problems"]), 1)
        Self.assertEqual(Reply["table_z_cm"], DS.TableZCm)

    def TestContainsAndTheCartFrame(Self):
        Surface = DS.Validate({"Surfaces": [Platform]})[0][0]
        Self.assertTrue(DS.Contains(Surface, -10.0, 35.0))
        Self.assertFalse(DS.Contains(Surface, -2.0, 35.0))
        Self.assertTrue(DS.Contains(Surface, -2.0, 35.0, MarginCm=1.5))
        Self.assertIs(DS.SurfaceToDisplay(Surface), Surface)
        with Self.assertRaises(ValueError):
            DS.SurfaceToDisplay({**Surface, "Frame": "world"})


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestDeclaredSurfaces` runs the Test* methods too (unittest names this hook)."""
    Prefixed = unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return unittest.TestSuite(Prefixed.loadTestsFromTestCase(Case) for Case in (DeclaredSurfaceTests,))


if __name__ == "__main__":
    Loader = unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Result = unittest.TextTestRunner(verbosity=2).run(Loader.loadTestsFromTestCase(DeclaredSurfaceTests))
    raise SystemExit(not Result.wasSuccessful())
