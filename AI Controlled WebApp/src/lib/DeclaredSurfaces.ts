import Frame from "../../../back end/WorkspaceFrame.json";
import Spec from "../../../back end/DeclaredSurfaces.json";

// DECLARED SURFACES (back end/DeclaredSurfaces.json, entered by hand): the supports besides the
// desk a task object may stand on, a platform or box top open above, or a shelf board with another
// board above it, in the AI's display frame (cm). The bridge measures an object on a surface only
// inside one of these (DeclaredSurfaces.py, the same rules: TestDeclaredSurfaces checks they agree),
// and a task holds the tip and the gripper above the surface its object stands on (AiTableFloor.ts
// TaskFloorFor). Depth alone cannot tell a platform from a desk that moved or a camera that drooped.
// Frame "cart": the arm's display frame with the cart parked, the only one today (SurfaceToDisplay).

export type SurfaceKind = "platform" | "shelf";
export type DeclaredSurface = { Name: string; Kind: SurfaceKind; TopZCm: number; XRangeCm: [number, number]; YRangeCm: [number, number];
  // A shelf: the underside of the board above (display Z); a platform: null.
  BoardAboveZCm: number | null; Frame: "cart" };

export const SurfaceRules = {
  // Where a surface may be (display Z): the arm's tip reaches about Z -13 to +43 with the cart parked.
  MinTopZCm: -15, MaxTopZCm: 45,
  // A surface this near the desk height is the desk.
  DeskSeparationCm: 1.5,
  MinExtentCm: 2, MaxAbsXCm: 80, YLimitsCm: [-10, 120] as [number, number],
  // A shelf's board above stands at least this far over the surface (the gripper is 5.5 cm tall at a level wrist).
  MinClearHeightCm: 5,
  MaxNameChars: 40,
};
const TableZCm = Frame.TableZCm;
const Kinds: SurfaceKind[] = ["platform", "shelf"];
const Frames = ["cart"];

const Finite = (Value: unknown): Value is number => typeof Value === "number" && Number.isFinite(Value);
// Python's "{:g}" for the numbers a spec holds, so both sides word a problem alike.
const G = (Value: number) => Number(Value.toPrecision(6)).toString();

function ReadRange(Value: unknown, Name: string, Low: number, High: number): { Range: [number, number] | null; Problem: string | null } {
  if (!Array.isArray(Value) || Value.length !== 2 || !Value.every(Finite)) return { Range: null, Problem: `${Name} must be [min, max] in cm` };
  const [Start, End] = Value as [number, number];
  if (End - Start < SurfaceRules.MinExtentCm) {
    return { Range: null, Problem: `${Name} ${G(Start)}..${G(End)} must run from min to max over at least ${G(SurfaceRules.MinExtentCm)} cm` };
  }
  if (Start < Low || End > High) return { Range: null, Problem: `${Name} ${G(Start)}..${G(End)} is outside ${G(Low)}..${G(High)} cm` };
  return { Range: [Start, End], Problem: null };
}

/** One entry: the surface as the code uses it, or null and why it is left out (both null: disabled). */
export function ValidateSurface(Entry: unknown, Index: number): { Surface: DeclaredSurface | null; Problem: string | null } {
  let Where = `surface ${Index + 1}`;
  const Fail = (Problem: string) => ({ Surface: null, Problem: `${Where}: ${Problem}` });
  if (!Entry || typeof Entry !== "object" || Array.isArray(Entry)) return { Surface: null, Problem: `${Where} must be an object` };
  const Item = Entry as Record<string, unknown>;
  const Name = typeof Item.Name === "string" ? Item.Name.split(/\s+/).filter(Boolean).join(" ") : "";
  if (Name) Where = `${Where} (${Name})`;
  const Enabled = "Enabled" in Item ? Item.Enabled : true;
  if (typeof Enabled !== "boolean") return Fail("Enabled must be true or false");
  if (!Enabled) return { Surface: null, Problem: null };
  if (!Name || Name.length > SurfaceRules.MaxNameChars) return Fail(`Name must be 1 to ${SurfaceRules.MaxNameChars} characters`);
  const Kind = Item.Kind;
  if (!Kinds.includes(Kind as SurfaceKind)) return Fail(`Kind must be one of ${Kinds.join(", ")}`);
  const Frame_ = "Frame" in Item ? Item.Frame : "cart";
  if (!Frames.includes(Frame_ as string)) return Fail(`Frame must be one of ${Frames.join(", ")} (a driving base will add others)`);
  const Top = Item.TopZCm;
  if (!Finite(Top)) return Fail("TopZCm must be a number (cm, display Z)");
  if (Top < SurfaceRules.MinTopZCm || Top > SurfaceRules.MaxTopZCm) {
    return Fail(`TopZCm ${G(Top)} is outside ${G(SurfaceRules.MinTopZCm)}..${G(SurfaceRules.MaxTopZCm)}, beyond the arm's reach`);
  }
  if (Math.abs(Top - TableZCm) < SurfaceRules.DeskSeparationCm) {
    return Fail(`TopZCm ${G(Top)} is within ${G(SurfaceRules.DeskSeparationCm)} cm of the desk (Z ${G(TableZCm)}): that is the desk`);
  }
  const X = ReadRange(Item.XRangeCm, "XRangeCm", -SurfaceRules.MaxAbsXCm, SurfaceRules.MaxAbsXCm);
  if (X.Problem) return Fail(X.Problem);
  const Y = ReadRange(Item.YRangeCm, "YRangeCm", ...SurfaceRules.YLimitsCm);
  if (Y.Problem) return Fail(Y.Problem);
  const Board = Item.BoardAboveZCm ?? null;
  if (Kind === "shelf") {
    if (!Finite(Board)) return Fail("a shelf needs BoardAboveZCm, the underside of the board above (cm, display Z)");
    if (Board < Top + SurfaceRules.MinClearHeightCm) {
      return Fail(`BoardAboveZCm ${G(Board)} must be at least ${G(SurfaceRules.MinClearHeightCm)} cm above TopZCm ${G(Top)}`);
    }
  } else if (Board !== null) return Fail("a platform has no board above (BoardAboveZCm null); declare a shelf for one");
  return { Surface: { Name, Kind: Kind as SurfaceKind, TopZCm: Top, XRangeCm: X.Range!, YRangeCm: Y.Range!,
    BoardAboveZCm: Kind === "shelf" ? Board as number : null, Frame: "cart" }, Problem: null };
}

/** The usable surfaces of a parsed spec and the problems with the rest. */
export function ValidateSurfaces(Value: unknown): { Surfaces: DeclaredSurface[]; Problems: string[] } {
  if (!Value || typeof Value !== "object" || Array.isArray(Value)) return { Surfaces: [], Problems: ["the file must hold a JSON object with a Surfaces list"] };
  const Entries = "Surfaces" in Value ? (Value as Record<string, unknown>).Surfaces : [];
  if (!Array.isArray(Entries)) return { Surfaces: [], Problems: ["Surfaces must be a list"] };
  const Surfaces: DeclaredSurface[] = [], Problems: string[] = [];
  Entries.forEach((Entry, Index) => {
    const { Surface, Problem } = ValidateSurface(Entry, Index);
    if (Problem) Problems.push(Problem);
    else if (Surface) {
      if (Surfaces.some(Other => Other.Name.toLowerCase() === Surface.Name.toLowerCase())) Problems.push(`surface ${Index + 1} (${Surface.Name}): the name is used twice`);
      else Surfaces.push(Surface);
    }
  });
  return { Surfaces, Problems };
}

const Loaded = ValidateSurfaces(Spec);
/** The surfaces declared in back end/DeclaredSurfaces.json (the page reloads when the file changes), and the problems with it. */
export const DeclaredSurfaceList: readonly DeclaredSurface[] = Loaded.Surfaces;
export const DeclaredSurfaceProblems: readonly string[] = Loaded.Problems;

/**
 * The page-load warning about back end/DeclaredSurfaces.json, or null when it has no problems: the bridge's
 * own reading (GET /surfaces "problems", what it measures objects with) and this page's (DeclaredSurfaceProblems,
 * built in when the page loaded), each problem once. A surface with a problem is left out of both, so an object
 * on it is measured as on no declared surface. Bridge: the /surfaces reply, or null when it did not answer.
 */
export function SurfaceProblemsNote(Bridge: unknown, Page: readonly string[] = DeclaredSurfaceProblems): string | null {
  const FromBridge = Bridge && typeof Bridge === "object" && Array.isArray((Bridge as { problems?: unknown }).problems)
    ? ((Bridge as { problems: unknown[] }).problems).filter((Item): Item is string => typeof Item === "string" && Boolean(Item.trim())) : [];
  const Problems = [...new Set([...FromBridge, ...Page])];
  if (!Problems.length) return null;
  return `back end/DeclaredSurfaces.json has ${Problems.length === 1 ? "a problem" : `${Problems.length} problems`}, and those surfaces are left out: `
    + `${Problems.join("; ")}. Fix the file, then reload the page.`;
}

/** A declared surface in the arm's display frame: Frame "cart" already is (a driving base would move it). */
export function SurfaceToDisplay(Surface: DeclaredSurface): DeclaredSurface {
  if (Surface.Frame !== "cart") throw new Error(`surface frame ${Surface.Frame} is not supported`);
  return Surface;
}

/** Whether display X, Y lies over a surface's extent, grown by MarginCm. */
export function SurfaceContains(Surface: DeclaredSurface, Point: { x: number; y: number }, MarginCm = 0) {
  const [[X0, X1], [Y0, Y1]] = [Surface.XRangeCm, Surface.YRangeCm];
  return Point.x >= X0 - MarginCm && Point.x <= X1 + MarginCm && Point.y >= Y0 - MarginCm && Point.y <= Y1 + MarginCm;
}

/** How a surface is named to the model and the user: "the platform "box platform" (Z 12.7, 15.0 cm above the desk)". */
export function SurfaceText(Surface: DeclaredSurface) {
  const Above = Surface.TopZCm - TableZCm;
  return `the ${Surface.Kind} "${Surface.Name}" (Z ${Surface.TopZCm.toFixed(1)}, ${Math.abs(Above).toFixed(1)} cm ${Above >= 0 ? "above" : "below"} the desk)`;
}
