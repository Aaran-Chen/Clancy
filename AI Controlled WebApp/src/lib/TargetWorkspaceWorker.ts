import { SetCadAlignment, type CadAlignment } from "./CadKinematics";
import { FindValidAlongLine, type TargetWorkspaceContext } from "./TargetWorkspace";
import type { TargetPose } from "./robotModel";
import type { PitchGuideSegment } from "./PitchGuide";
import { BeyondArmReach, FindReachableWristTarget } from "./AutomaticWrist";
import { SnapAiTarget, type AiSnapRules, type SnapInput } from "./AiTargetSnap";

// Worker event/API names are imposed by the browser.
// Kind "AiSnap": an AI proposal's target snap (AiTargetSnap.ts), answered as { Result }.
// A manual release has half a second (App.tsx TargetSnapLimitMs): the wrist-only answer comes first,
// from the cached pitch guide when there is one, else 4 deg samples refined to 0.005 deg. Over 80
// targets off by 10-180 deg of wrist: median 0.12 s, 90% under 0.19 s (0.46 s with 0.75 deg samples),
// none missed and 90% the same pitch; a valid window narrower than 4 deg can be passed over for the next.
// A position no wrist angle reaches: the reachable edge of the drag line (FindValidAlongLine), ~40 ms.
const ReleaseWristSearch = { StepDeg: 4 };
self.onmessage = (Event: MessageEvent<{ Requested: TargetPose; Previous: TargetPose;
  Context: TargetWorkspaceContext; Alignment: CadAlignment; Kind?: "Position" | "Pitch"; Guide?: PitchGuideSegment[] }
  | { Kind: "AiSnap"; Input: SnapInput; Rules?: AiSnapRules; Alignment: CadAlignment }>) => {
  const Data = Event.data;
  if (Data.Kind === "AiSnap") {
    try {
      if (!SetCadAlignment(Data.Alignment)) throw new Error("Invalid CAD alignment");
      self.postMessage({ Result: SnapAiTarget(Data.Input, Data.Rules) });
    } catch (Error) { self.postMessage({ Result: null, Error: String(Error) }); }
    return;
  }
  try {
    if (!SetCadAlignment(Data.Alignment)) throw new Error("Invalid CAD alignment");
    self.postMessage({ Target: Data.Kind === "Pitch"
      ? FindReachableWristTarget(Data.Requested, Data.Context, Data.Guide, ReleaseWristSearch)
      : (BeyondArmReach(Data.Requested) ? null : FindReachableWristTarget(Data.Requested, Data.Context, Data.Guide, ReleaseWristSearch))
        ?? FindValidAlongLine(Data.Requested, Data.Previous, Data.Context) });
  } catch (Error) { self.postMessage({ Target: null, Error: String(Error) }); }
};
