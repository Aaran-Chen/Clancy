"""Red tape detection and 3D camera fitting without camera or motor access."""
import base64
import json
from pathlib import Path
import unittest
from unittest.mock import patch

import cv2
import numpy as np
import RecordedCaptures as RC
import SelfCalibration as SC

Root = Path(__file__).resolve().parent
Artifacts = Root / "TestArtifacts"
# The 2026-09-13 calibration frames these regressions were found on (RecordedCaptures: not on disk, the test fails).
Day0913 = RC.Day("2026-09-13") / "selfcal"
Artifacts.mkdir(exist_ok=True)


def Encode(Image):
    _, Data = cv2.imencode(".jpg", Image)
    return "data:image/jpeg;base64," + base64.b64encode(Data).decode("ascii")


class RedTapeTests(unittest.TestCase):
    def setUp(Self):
        Self.SessionPatch = patch.object(SC, "SessionPath", Artifacts / "RedSession.json")
        Self.CapturePatch = patch.object(SC, "CapturesDir", Artifacts / "RedCaptures")
        Self.SessionPatch.start()
        Self.CapturePatch.start()
        SC.ResetSession()
        SC._RedTracker.clear()

    def tearDown(Self):
        Self.SessionPatch.stop()
        Self.CapturePatch.stop()
        SC._Session.clear()
        SC._Session.update({"image_size": None, "mirrored": None, "observations": []})
        SC._RedTracker.clear()
        try:
            (Artifacts / "RedSession.json").unlink()
        except OSError:
            pass
        Base = Artifacts / "RedCaptures"
        if Base.exists():
            for Child in sorted(Base.rglob("*"), reverse=True):
                try:
                    if Child.is_file() or Child.is_symlink():
                        Child.unlink()
                    else:
                        Child.rmdir()
                except OSError:
                    pass
            try:
                Base.rmdir()
            except OSError:
                pass

    def TestSavedTipAndAbsentTip(Self):
        Folder = Day0913
        RC.Need(Folder / "092856_964_pose0_miss.jpg", What="the 2026-09-13 09:28 calibration frame")
        Image = cv2.imread(str(Folder / "092856_964_pose0_miss.jpg"))
        Result = SC.CheckFrames([Encode(cv2.flip(Image, 1))], True,
                                Hint={"u": 941 / 1280, "v": 346 / 720}, Initialize=True)
        Self.assertTrue(Result["detected"], Result.get("reason"))
        Self.assertLess(np.linalg.norm(np.array(SC._RedTracker["center"]) - [942, 346]), 3)
        Image = cv2.imread(str(Folder / "094500_875_pose24_miss.jpg"))
        Self.assertIsNotNone(Image)
        Result = SC.CheckFrames([Encode(cv2.flip(Image, 1))], True)
        Self.assertFalse(Result["detected"], "Cable/wood must not replace an absent tip")

    def TestGrayBeigeLandmarkAndRedTracking(Self):
        Image = np.full((480, 640, 3), 210, np.uint8)
        cv2.rectangle(Image, (80, 90), (170, 112), (160, 178, 183), -1)
        cv2.rectangle(Image, (290, 290), (312, 310), (25, 60, 230), -1)
        Result = SC.CheckFrames([Encode(Image)] * 3, False, Hint={"u": 300 / 640, "v": 300 / 480}, Initialize=True)
        Self.assertTrue(Result["detected"])
        Image[280:320, 280:325] = 210
        cv2.rectangle(Image, (325, 275), (347, 295), (25, 60, 230), -1)
        Result = SC.CheckFrames([Encode(Image)] * 3, False)
        Self.assertTrue(Result["detected"])
        Image[265:310, 320:355] = 210
        Self.assertFalse(SC.CheckFrames([Encode(Image)], False)["detected"])

    def TestSelectionRequiredForAmbiguousRedObjects(Self):
        Image = np.full((480, 640, 3), 220, np.uint8)
        cv2.circle(Image, (200, 200), 12, (0, 20, 240), -1)
        cv2.circle(Image, (300, 250), 12, (0, 20, 240), -1)
        Self.assertFalse(SC.CheckFrames([Encode(Image)], False)["detected"])

    def TestSavedWoodSelectionIsRejectedAndVisibleTapeAccepted(Self):
        # Warm floorboards have the orange tape's hue and saturation (80% of the pixels around this
        # click); red over green (1.25 here, the tape 4.2) is what rejects them. A rejected selection
        # also ends the previous tracking identity. The click this test used to make, at 875,660, is
        # now a 210 px candidate, and probably the tip tape through the basket (the next click took it
        # as the tape): size no longer tells the wiring from the tape, the solve does (below).
        Folder = Day0913
        RC.Need(Folder / "201658_424_check_red_miss.jpg", Folder / "202205_283_pose14_red_hit.jpg", What="the 2026-09-13 20:16-20:22 calibration frames")
        Start = cv2.imread(str(Folder / "201658_424_check_red_miss.jpg"))
        Tape = cv2.imread(str(Folder / "202205_283_pose14_red_hit.jpg"))
        Hsv = cv2.cvtColor(Start, cv2.COLOR_BGR2HSV)[266:287, 588:609].reshape(-1, 3)
        Self.assertGreater(np.mean((Hsv[:, 0] <= 18) & (Hsv[:, 1] >= 85) & (Hsv[:, 2] >= 65)), 0.6, "the wood has the tape's hue")
        Result = SC.CheckFrames([Encode(Start)] * 3, False, Hint={"u": 598, "v": 276}, Initialize=True)
        Self.assertFalse(Result["detected"])
        Self.assertFalse(SC._RedTracker)
        Result = SC.CheckFrames([Encode(Tape)] * 3, False, Hint={"u": 523, "v": 361}, Initialize=True)
        Self.assertTrue(Result["detected"], Result.get("reason"))
        Self.assertLess(np.linalg.norm(np.array(Result["centroid"]) - [523, 361]), 5)
        Self.assertGreater(SC._RedTracker["seed_area"], 1000)
        Result = SC.CheckFrames([Encode(Start)] * 3, False, Hint={"u": 598, "v": 276}, Initialize=True)
        Self.assertFalse(Result["detected"])
        Self.assertFalse(SC._RedTracker, "A rejected selection must invalidate the previous tracking identity")

    def TestRecordedWireTrackingIsReassignedToTheTape(Self):
        # 2026-09-13 22:31: at pose 12 the live tracker sat on the orange servo wire by the wrist
        # (927,552; captures/2026-09-13/selfcal/223623_287_pose12_red_hit.jpg) while the tip tape was
        # at 779,249. Solve's RobustRedAssignment picks each pose's blob by camera consistency from all
        # of its red blobs, rebuilt here from the archived frames and joints as AddRedProbeObservation
        # keeps them, and must move pose 12 onto the tape while agreeing where the tracker held it.
        Run = Day0913 / "runs" / "20260913_223143_8d4054581401"
        RC.Need(Run / "Session.json", Run / "observations", What="the 2026-09-13 22:31 calibration run")
        Session = json.loads((Run / "Session.json").read_text(encoding="utf-8"))
        Tracked = {Observation["pose_index"]: Observation["corners"][0] for Observation in Session["observations"]}
        Poses = []
        for Archive in sorted((Run / "observations").glob("*_pose*.json")):
            Record = json.loads(Archive.read_text(encoding="utf-8"))
            Images = [SC.DecodeDataUrl(Url) for Url in Record["images"]]
            Options = [SC.DetectRedDetailed(cv2.flip(Image, 1) if Record["mirrored"] else Image)["options"] for Image in Images]
            Tip, Frame = SC.GripperFrame(Record["joints"])
            Poses.append({"pose_index": Record["pose_index"], "tip": Tip.tolist(), "frame": Frame.tolist(),
                          "candidates": SC.StableRedCandidates(Options)})
        Assigned, Diagnostics = SC.RobustRedAssignment(Poses, Session["image_size"])
        Self.assertIsNotNone(Assigned, Diagnostics)
        Self.assertGreaterEqual(len(Assigned), 20, Diagnostics)
        Picked = {Observation["pose_index"]: Observation["corners"][0] for Observation in Assigned}
        Self.assertLess(np.linalg.norm(np.subtract(Tracked[12], [927, 552])), 3, "the tracker's pick is the wire")
        Self.assertLess(np.linalg.norm(np.subtract(Picked[12], [779, 249])), 5, Picked.get(12))
        for Index in (0, 1, 17, 18, 19, 20):
            with Self.subTest(pose=Index):
                Self.assertLess(np.linalg.norm(np.subtract(Picked[Index], Tracked[Index])), 5)

    def TestTrackingCannotShrinkOntoWireFragments(Self):
        # The floor is the selection's area over RedTrackingAreaRatio, halved (167 px for a 4000 px
        # seed): a 150 px fragment within that ratio of the last tracked area is still refused, a
        # 200 px one is tracked. Real seeds are 100-270 px, so on the hardware the solve's
        # RobustRedAssignment, not this floor, keeps the tape from the wiring.
        for Bottom, Tracked in ((415, False), (420, True)):
            with Self.subTest(area=10 * (Bottom - 400)):
                Image = np.full((720, 1280, 3), 210, np.uint8)
                cv2.rectangle(Image, (500, 400), (510, Bottom), (0, 20, 240), -1)
                SC._RedTracker.clear()
                SC._RedTracker.update(center=[505, 410], area=400, seed_area=4000, background=[])
                Centroid = SC.DetectRedDetailed(Image, Track=True)["centroid"]
                Self.assertEqual(Centroid is not None, Tracked, Centroid)

    def TestRejectedPosesKeepOriginalFramesAndJoints(Self):
        Images = [Encode(np.full((120, 160, 3), Level, np.uint8)) for Level in (160, 180, 200)]
        Joints = [{"x": 0., "y": 0., "z": 0.}, {"x": 0., "y": 4., "z": 5.}, {"x": 0., "y": 8., "z": 7.}]
        Archives = []
        for Attempt in range(2):
            Result = SC.AddObservation(Images, Joints, True, PoseIndex=4, Method="red")
            Self.assertTrue(Result["ok"], Result)
            Self.assertFalse(Result["detected"])
            Archive = Path(Result["capture_archive"])
            Saved = json.loads(Archive.read_text(encoding="utf-8"))
            Self.assertEqual(Saved["images"], Images)
            Self.assertEqual(Saved["joints"], Joints)
            Self.assertTrue(Saved["mirrored"])
            Self.assertEqual(Saved["pose_index"], 4)
            Archives.append(Archive)
        Self.assertNotEqual(*Archives, "Repeating a pose must not overwrite its earlier capture")
        Self.assertEqual(SC._Session["observations"], [])

    def TestArchiveFailureIsReportedBeforeDetection(Self):
        with patch.object(Path, "open", side_effect=OSError("disk full")), patch.object(SC, "AddRedProbeObservation") as Detect:
            Result = SC.AddObservation([], [], False, Method="red")
        Self.assertFalse(Result["ok"])
        Self.assertIn("archive", Result["error"])
        Detect.assert_not_called()

    def TestSmallSelectionRejectedAtEveryCameraSize(Self):
        # The selection floor (40 px at 1280x720, at least 20) scales with the frame: a 6 px square
        # scaled to each size is too small, a 10 px one is the tape.
        for Width, Height in ((1920, 1080), (1280, 720), (640, 360)):
            Scale = Width / 1280
            for Side, Accepted in ((6, False), (10, True)):
                with Self.subTest(Width=Width, Side=Side):
                    Image = np.full((Height, Width, 3), 210, np.uint8)
                    cv2.rectangle(Image, (int(500 * Scale), int(400 * Scale)),
                                  (int((500 + Side) * Scale), int((400 + Side) * Scale)), (0, 20, 240), -1)
                    Result = SC.DetectRedDetailed(Image, Hint={"u": (500 + Side / 2) / 1280, "v": (400 + Side / 2) / 720})
                    if Accepted:
                        Self.assertIsNotNone(Result["centroid"], Result["reason"])
                    else:
                        Self.assertIsNone(Result["centroid"])
                        Self.assertIn("too small", Result["reason"] or "")

    def TestClippedBackgroundDoesNotOverrideSelectionFailure(Self):
        Image = np.full((720, 1280, 3), 210, np.uint8)
        cv2.rectangle(Image, (800, 0), (850, 40), (0, 20, 240), -1)
        cv2.rectangle(Image, (500, 400), (506, 406), (0, 20, 240), -1)
        Result = SC.DetectRedDetailed(Image, Hint={"u": 503, "v": 403})
        Self.assertIn("too small", Result["reason"] or "")

    def TestClippedPatchIsRejectedEvenWithInteriorCentroid(Self):
        Image = np.full((480, 640, 3), 210, np.uint8)
        cv2.rectangle(Image, (280, 0), (330, 35), (0, 20, 240), -1)
        Result = SC.CheckFrames([Encode(Image)] * 3, False, Hint={"u": 305, "v": 17}, Initialize=True)
        Self.assertFalse(Result["detected"])
        Self.assertIn("cut off", Result["reason"])
        Self.assertFalse(SC._RedTracker)

    def TestNearbySideSliverCannotReplaceMainTape(Self):
        Image = np.full((480, 640, 3), 210, np.uint8)
        cv2.rectangle(Image, (270, 250), (305, 290), (0, 20, 240), -1)
        cv2.rectangle(Image, (309, 265), (317, 280), (0, 20, 240), -1)
        SC._RedTracker.update(center=[350, 270], area=1400, background=[])
        Result = SC.DetectRedDetailed(Image, Track=True, MaxJumpFrac=0.3)
        Self.assertIsNotNone(Result["centroid"])
        Self.assertLess(Result["centroid"][0], 300)
        # Even when the main patch fails the area gate (1400 px is 14x the tracked 100 px, over
        # RedTrackingAreaRatio), its sliver is not a valid substitute for the rigid marker point.
        SC._RedTracker["area"] = 100
        Self.assertIsNone(SC.DetectRedDetailed(Image, Track=True, MaxJumpFrac=0.3)["centroid"])

    def TestHalfTheFramesCannotApproveAnObservation(Self):
        Blank = np.full((480, 640, 3), 210, np.uint8)
        Image = Blank.copy()
        cv2.rectangle(Image, (290, 290), (312, 310), (25, 60, 230), -1)
        Images = [Encode(Image), Encode(Blank)]
        SC._RedTracker.update(center=[301, 300], area=440, background=[])
        Self.assertFalse(SC.CheckFrames(Images, False)["detected"])
        Joints = [{"x": 0, "y": 0, "z": 0}, {"x": 0, "y": 10, "z": 5}, {"x": 0, "y": 12, "z": 5}]
        Result = SC.AddObservation(Images, Joints, False, 0, Method="red")
        Self.assertFalse(Result["detected"])
        Self.assertFalse(SC._Session["observations"])

    def TestSavedClippingAndSideFragmentRegressions(Self):
        Folder = Day0913
        RC.Need(Folder / "162122_330_pose13_red_hit.jpg", Folder / "162228_149_pose26_red_hit.jpg", What="the 2026-09-13 16:21-16:22 calibration frames")
        Clipped = cv2.imread(str(Folder / "162122_330_pose13_red_hit.jpg"))
        Split = cv2.imread(str(Folder / "162228_149_pose26_red_hit.jpg"))
        Self.assertIsNone(SC.DetectRedDetailed(Clipped, Hint={"u": 778, "v": 33})["centroid"])
        SC._RedTracker.update(center=[1011, 220], area=3000, background=[])
        Result = SC.DetectRedDetailed(Split, Track=True, MaxJumpFrac=0.3)
        Self.assertIsNotNone(Result["centroid"])
        Self.assertLess(Result["centroid"][0], 820)
        Self.assertGreater(Result["area"], 3000)

    def TestSinglePointThreeDimensionalFit(Self):
        Rng = np.random.default_rng(91)
        Eye = np.array([20., -35., 48.])
        Forward = np.array([0., 20., 1.]) - Eye
        Forward /= np.linalg.norm(Forward)
        Right = np.cross(Forward, [0., 0., 1.])
        Right /= np.linalg.norm(Right)
        Rotation = np.vstack([Right, np.cross(Forward, Right), Forward])
        Translation = -Rotation @ Eye
        Rvec, _ = cv2.Rodrigues(Rotation)
        Intrinsic = np.array([[900., 0., 639.5], [0., 900., 359.5], [0., 0., 1.]])
        Offset = np.array([0.4, -0.6, 0.8])
        Tips = Rng.uniform([-12, 12, -2], [12, 28, 17], (36, 3))
        Observations = []
        for Index, Tip in enumerate(Tips):
            Frame, _ = cv2.Rodrigues(Rng.normal(0, 0.5, 3))
            Pixel, _ = cv2.projectPoints((Tip + Frame @ Offset)[None], Rvec, Translation, Intrinsic, None)
            Pixel = Pixel.reshape(1, 2) + Rng.normal(0, 0.15, (1, 2))
            Observations.append({"tip": Tip.tolist(), "frame": Frame.tolist(), "corners": Pixel.tolist(), "method": "red"})
        SC._Session.update(image_size=[1280, 720], mirrored=False, observations=Observations)
        Report = SC.Solve(TableZ=-7.17, Save=False)
        Self.assertTrue(Report["ok"], Report.get("error"))
        Self.assertFalse(Report["saved"])
        Model = Report["model"]
        Self.assertEqual(len(Model["marker_corners_gripper_cm"]), 1)
        Truth = np.array([[X, Y, -7.17] for X in (-10., 0., 10.) for Y in (15., 22., 29.)])
        Pixels, _ = cv2.projectPoints(Truth, Rvec, Translation, Intrinsic, None)
        Recovered, Valid = SC._RayToPlane(Model, Pixels[:, 0, 0], Pixels[:, 0, 1], -7.17)
        Self.assertTrue(Valid.all())
        Error = float(np.sqrt(np.mean(np.sum((Recovered - Truth) ** 2, axis=1))))
        print(f"Verified red-tip 3D camera table RMS: {Error:.4f} cm")
        Self.assertLess(Error, 0.5)

    def TestStationaryBlobRejected(Self):
        SC._Session.update(image_size=[640, 480], mirrored=False, observations=[
            {"tip": [I, I % 3, I % 5], "frame": np.eye(3).tolist(), "corners": [[200, 200]]}
            for I in range(16)])
        Result = SC.Solve(Save=False)
        Self.assertFalse(Result["ok"])
        Self.assertIn("stationary", Result["error"])


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestRedTape` runs the Test* methods too (unittest names this hook)."""
    Prefixed = unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return Prefixed.loadTestsFromTestCase(RedTapeTests)


if __name__ == "__main__":
    Loader = unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Result = unittest.TextTestRunner(verbosity=2).run(Loader.loadTestsFromTestCase(RedTapeTests))
    raise SystemExit(not Result.wasSuccessful())
