import { SetCadAlignment, type CadAlignment } from "./CadKinematics";
import { BuildPitchGuide, type PitchGuideSegment } from "./PitchGuide";
import { FindReachableWristTarget } from "./AutomaticWrist";
import type { TargetPose } from "./robotModel";
import type { TargetWorkspaceContext } from "./TargetWorkspace";

self.onmessage = (Event: MessageEvent<{ Target: TargetPose; Context: TargetWorkspaceContext; Alignment: CadAlignment; Guide?: PitchGuideSegment[] }>) => {
  try {
    if (!SetCadAlignment(Event.data.Alignment)) throw new Error("Invalid CAD alignment");
    const Segments = Event.data.Guide ?? BuildPitchGuide(Event.data.Target, Event.data.Context);
    const Resolved = FindReachableWristTarget(Event.data.Target, Event.data.Context, Segments);
    self.postMessage({ Segments: !Segments.length && Resolved ? BuildPitchGuide(Resolved, Event.data.Context) : Segments, Resolved });
  } catch { self.postMessage({ Segments: [] }); }
};
