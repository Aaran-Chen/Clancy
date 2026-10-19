import { SetCadAlignment, type CadAlignment } from "./CadKinematics";
import { TaskObjectReachability, type LocatedObject, type ReachContext, type ReachPlanned } from "./AiMeasurements";

// The task object's reach verdict and GRASP PLAN (AiMeasurements.ts TaskObjectReachability), off the page
// thread. Planned on the page, a grasp held it up to 11.8 s (2026-10-04 04:56:12Z), and meanwhile no click
// was handled, Emergency Shutoff and Stop and hold included (2026-10-07 audit M4). The page posts
// { Object, Context, Planned, Alignment } and gets { Reach } (null with Error when planning threw).
// Worker event/API names are imposed by the browser.
self.onmessage = (Event: MessageEvent<{ Object: LocatedObject; Context: ReachContext; Planned?: ReachPlanned; Alignment: CadAlignment }>) => {
  try {
    if (!SetCadAlignment(Event.data.Alignment)) throw new Error("Invalid CAD alignment");
    self.postMessage({ Reach: TaskObjectReachability(Event.data.Object, Event.data.Context, Event.data.Planned) });
  } catch (Error) { self.postMessage({ Reach: null, Error: String(Error) }); }
};
