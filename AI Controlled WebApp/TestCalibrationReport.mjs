// What a /selfcal/solve report means for the user (src/lib/CalibrationReport.ts) and how the
// semi-automatic store shows it (src/lib/SemiCalibration.ts): every computed camera is saved and
// applied, its quality problems are warnings, only the user rejects one, and a run that computed
// no camera keeps the current calibration. No browser, bridge or arm.
import { build as Build } from "esbuild";
import { test as Test } from "node:test";
import Assert from "node:assert/strict";
const Load = async File => {
  const Bundle = await Build({ entryPoints: [File], bundle: true, write: false, platform: "node", format: "esm" });
  return import(`data:text/javascript;base64,${Buffer.from(Bundle.outputFiles[0].text).toString("base64")}`);
};
const Report = await Load("src/lib/CalibrationReport.ts"), Semi = await Load("src/lib/SemiCalibration.ts");

const Problems = ["fit error 0.74 cm at the marker (11.0 px) exceeds 0.6 cm",
  "the camera moved 0.21 deg / +0.05 cm during calibration (desk in depth, first against last third of the poses; limits 0.15 deg / 0.3 cm): lock the camera head and recalibrate"];

Test("a clean fit is calibrated and saved", () => {
  const Outcome = Report.CalibrationOutcomeOf({ ok: true, computed: true, saved: true, quality_ok: true, problems: [] });
  Assert.equal(Outcome.Kind, "saved");
  Assert.equal(Outcome.Title, "Calibrated and saved.");
  Assert.equal(Outcome.RunOutcome, "calibrated");
});

Test("each former rejection reason is a warning on a saved calibration", () => {
  const Outcome = Report.CalibrationOutcomeOf({ ok: true, computed: true, saved: true, quality_ok: false, problems: Problems,
    quality_warning: "Check this calibration. Reject it to restore the previous one." });
  Assert.equal(Outcome.Kind, "saved_with_warnings");
  Assert.equal(Outcome.Title, "Saved and applied, with warnings.");
  Assert.deepEqual(Outcome.Problems, Problems, "every problem is shown with its numbers");
  Assert.match(Outcome.Detail, /Reject it/);
  Assert.equal(Outcome.RunOutcome, "calibrated with warnings");
});

Test("a run that computed no camera keeps the current calibration", () => {
  const Outcome = Report.CalibrationOutcomeOf({ ok: false, computed: false, saved: false,
    error: "Could not compute a calibration: only 4 poses have a click; at least 6 are needed to compute a camera." });
  Assert.equal(Outcome.Kind, "not_computed");
  Assert.match(Outcome.Title, /current calibration is still in use/);
  Assert.match(Outcome.Detail, /only 4 poses/);
  Assert.equal(Outcome.RunOutcome, "not computed");
});

Test("a computed camera the bridge could not write is not called rejected", () => {
  const Outcome = Report.CalibrationOutcomeOf({ ok: false, computed: true, saved: false, problems: [],
    error: "The calibration was computed but could not be saved: disk full." });
  Assert.equal(Outcome.Kind, "not_saved");
  Assert.match(Outcome.Title, /Computed but not saved/);
});

Test("an old bridge that still rejects asks for a restart", () => {
  const Outcome = Report.CalibrationOutcomeOf({ ok: false, problems: Problems.slice(0, 1), error: "Calibration rejected: fit error 0.74 cm" });
  Assert.equal(Outcome.Kind, "not_saved");
  Assert.match(Outcome.Title, /Restart the bridge/);
});

Test("a rejected calibration says so", () => {
  const Outcome = Report.CalibrationOutcomeOf({ ok: true, computed: true, saved: true, problems: Problems, rejected: true });
  Assert.equal(Outcome.Kind, "rejected");
  Assert.match(Outcome.Title, /Rejected by you/);
});

Test("the reject question and result name the calibrations by their fit time", () => {
  Assert.equal(Report.FittedClock("2026-10-10T14:02:33"), "14:02");
  Assert.equal(Report.FittedClock(null), null);
  Assert.equal(Report.RejectQuestion("2026-10-10T14:02:33", "2026-10-09T21:10:05"),
    "Reject the calibration fitted at 14:02? The calibration it replaced (fitted at 21:10) is restored and used again.");
  Assert.equal(Report.RejectedText({ ok: true, rejected_fitted_at: "2026-10-10T14:02:33", restored_fitted_at: "2026-10-09T21:10:05",
    restored_mode: "auto", calibrated: true, drift_baseline: true }),
  "Calibration fitted at 14:02 rejected: the calibration fitted at 21:10 is in use again.");
  Assert.match(Report.RejectedText({ ok: true, rejected_fitted_at: "2026-10-10T14:02:33", restored_fitted_at: "2026-10-09T21:10:05",
    restored_mode: "auto", calibrated: true, drift_baseline: false }), /no camera-check baseline/);
  Assert.match(Report.RejectedText({ ok: true, rejected_fitted_at: "2026-10-10T14:02:33", calibrated: false }),
    /there was no calibration before it, so the camera is not calibrated now/);
  Assert.match(Report.RejectedText({ ok: true, restored_fitted_at: "2026-07-24T10:00:00", restored_mode: "manual", calibrated: true }),
    /the manual \(probe\) calibration fitted at 10:00 is in use again/);
});

Test("the semi-automatic store shows a saved fit with warnings, and a rejection", () => {
  const Store = new Semi.SemiCalStore();
  Store.Begin(12);
  Store.SetReport({ ok: true, computed: true, saved: true, quality_ok: false, problems: Problems, observations_used: 8, fitted_at: "2026-10-10T14:02:33" });
  Assert.equal(Store.Snapshot().Phase, "review");
  Assert.equal(Store.Snapshot().Message, "Saved and applied from 8 clicks, with 2 warnings");
  Store.MarkRejected("Calibration fitted at 14:02 rejected: the calibration fitted at 21:10 is in use again.");
  Assert.equal(Store.Snapshot().Report.rejected, true);
  Assert.match(Store.Snapshot().Message, /rejected/);
  Store.SetReport({ ok: false, computed: false, error: "Could not compute a calibration: only 4 poses have a click." });
  Assert.match(Store.Snapshot().Message, /^Not computed: Could not compute a calibration/);
  Store.SetReport({ ok: true, computed: true, saved: true, quality_ok: true, problems: [], observations_used: 14 });
  Assert.equal(Store.Snapshot().Message, "Calibrated and saved from 14 clicks");
});

Test("six clicks are enough to fit; ten are recommended", () => {
  Assert.equal(Semi.SemiCalMinClicks, 6);
  Assert.equal(Semi.SemiCalRecommendedClicks, 10);
});
