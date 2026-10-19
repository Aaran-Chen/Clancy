import type { LocatedObject } from "./AiMeasurements";
import { HasVerifiedDepthFrame } from "./SceneGeometry";

// The object a task acts on: read from the task as the user typed it, then
// found among each check-in's measured objects. The 16:05 task "Grab the
// black thingon the desk" took the glued typo "thingon" as the label and
// never matched the USB drive the model itself pointed at; "Grabe the eraser"
// and "Move the gripper tip ... above the red eraser" had no object at all,
// which also switched the camera gate off.

export type TaskObjectQuery = {
  // The concrete noun ("eraser"); null for a generic one ("the black thing").
  Head: string | null;
  Generic: boolean;
  Colours: string[];
  Sizes: string[];
  // The phrase's nouns, the head last ("usb", "drive").
  Nouns: string[];
  // Where the task says the object is. Not where it goes: in "put the eraser
  // in the bin" the bin is the destination, and the eraser is on the desk.
  // "surface": on a shelf, a platform or the like (back end/DeclaredSurfaces.json).
  Support: "desk" | "bin" | "surface" | null;
  // "black thing", "red eraser": the TASK OBJECT line's name and the bridge's label for it.
  Name: string;
  // "black thing on the desk": the bridge's strict task query (at most 80 characters).
  Phrase: string;
  // The verb was read through a typo ("Grabe"), and the glued words split ("thingon -> thing on").
  VerbFuzzy: boolean;
  Repaired: string[];
  // The verb the object was read after ("grab"), and whether it moves the arm
  // itself ("move the gripper above the eraser") rather than acting on the object.
  Verb: string;
  ArmMoves: boolean;
};

const Verbs = ["grab", "grasp", "pick", "take", "get", "fetch", "bring", "hold", "lift", "carry", "touch", "tap", "poke", "press", "push",
  "pull", "slide", "drag", "flip", "move", "put", "place", "drop", "stack", "hand", "give", "throw", "toss", "remove", "insert", "open",
  "reach", "point", "hover", "approach", "go", "collect", "knock", "grip", "catch", "retrieve", "lower", "raise", "position", "center",
  "centre", "align"];
// Verbs that move the arm itself: with the arm (or nothing) as their object,
// the object is the one after the preposition ("move the gripper tip to ... above the eraser").
const MotionVerbs = new Set(["move", "go", "hover", "reach", "lower", "raise", "bring", "position", "center", "centre", "align"]);
// Verbs whose "in the bin" / "on the desk" is where the object goes.
const PlaceVerbs = new Set(["put", "place", "drop", "throw", "toss", "insert", "stack", "move"]);
// "hand" is a verb only when something is handed ("hand me the eraser", not "raise your hand").
const Handed = new Set(["me", "us", "it", "them", "over", "this", "that", "the", "a", "an", "some", "my", "your"]);
// The arm's own parts, singular (plurals are singularised before the check).
const ArmWords = new Set(["arm", "gripper", "tip", "claw", "jaw", "wrist", "hand", "finger", "fingertip", "end", "effector", "yourself"]);
const NotObjects = new Set(["it", "them", "this", "that", "these", "those", "something", "anything", "everything", "nothing", "left", "right",
  "up", "down", "forward", "back", "backward", "backwards", "closer", "farther", "further", "home", "cm", "mm", "inch", "inches", "centimeter",
  "centimeters", "centimetre", "centimetres", "camera", "look", "picture", "photo", "me", "us", "you", "i", "we", "desk", "table", "workspace",
  "scene", "area", "center", "centre", "middle", "position", "degree", "degrees", "x", "y", "z", "side", "corner", "edge", "front", "bottom",
  "origin", "start", "wide", "wider", "fully", "more", "all", "way", "bit", "spot", "ahead"]);
const GenericNouns = new Set(["thing", "object", "item", "one", "stuff", "piece"]);
const Colours = new Set(["black", "white", "red", "blue", "green", "yellow", "orange", "purple", "pink", "grey", "brown", "silver", "gold",
  "golden", "dark", "light", "beige", "clear", "transparent"]);
const Sizes = new Set(["small", "little", "tiny", "big", "large", "long", "short", "tall", "flat", "thin", "round"]);
const Materials = new Set(["wooden", "plastic", "metal", "metallic", "rubber", "glass", "paper", "cardboard"]);
// The phrase ends at a preposition, a conjunction, or the adverbs, politeness
// words and clauses after it ("the eraser please", "the pen that is ...").
const Stops = new Set(["in", "into", "on", "onto", "to", "from", "and", "or", "but", "then", "with", "without", "near", "next", "over", "under",
  "at", "by", "off", "up", "down", "back", "of", "for", "toward", "towards", "beside", "behind", "above", "below", "inside", "out", "around",
  "please", "now", "carefully", "gently", "quickly", "slowly", "softly", "fast", "again", "that", "which", "who", "where", "so", "if", "when",
  "while", "until", "using", "there", "here", "too", "also", "first", "is", "are", "sitting", "lying"]);
const Spatial = new Set(["above", "over", "to", "toward", "towards", "near", "onto", "beside", "behind", "next", "at", "by", "around"]);
const Skip = new Set(["the", "a", "an", "my", "your", "some", "that", "this", "those", "these", "about", "around", "roughly", "approximately",
  "directly", "just", "only", "exactly", "straight", "right", "top", "of", "front", "cm", "centimeters", "centimetres", "mm", "inches"]);
const Articles = new Set(["the", "a", "an", "my", "your", "some"]);
const Determiners = new Set([...Articles, "this", "that", "these", "those", "his", "her", "its", "our", "their"]);
// Common desk objects: a word the user meant (never a typo of a verb), and a
// left part that lets a glued word split before any label is known ("eraseron").
const DeskNouns = new Set(["eraser", "rubber", "pen", "pencil", "marker", "cup", "mug", "bowl", "glass", "bottle", "can", "box", "ball", "cube",
  "tape", "phone", "mouse", "keyboard", "book", "notebook", "paper", "usb", "drive", "stick", "key", "coin", "battery", "cable", "wire", "cord",
  "plier", "scissor", "spoon", "fork", "toy", "lid", "cap", "card", "ruler", "clip", "stapler", "remote", "charger", "sock", "tool", "wrench",
  "screwdriver", "lego", "brick", "dice", "die", "sharpener", "highlighter", "case", "wallet", "watch", "earbud", "tube", "block", "bin",
  "basket", "bucket", "carton", "button", "sponge", "glove", "spool", "binder", "bag", "tong"]);
// Ordinary words one letter from a verb ("told"/"hold", "plate"/"place", "oven"/"open").
const NotTypos = new Set(["told", "held", "list", "each", "more", "mode", "five", "make", "match", "joint", "crop", "being", "side", "live",
  "gave", "hard", "plate", "oven", "cake", "fake", "lake", "wake", "bake", "cold", "bold", "fold", "sold", "hole", "role", "rope", "lock",
  "hook", "look", "took", "cook", "lift", "gift", "pull", "full", "shop", "stop", "step", "slice", "price", "prize", "space", "piece"]);
// A task led by one of these asks about the scene: its words are not read as misspelt verbs.
const QuestionLeads = new Set(["describe", "tell", "what", "whats", "where", "how", "why", "which", "who", "list", "count", "explain", "is",
  "are", "does", "do", "did"]);
// Labels the grounding model used for the same things in the logs (eraser/rubber, cup/bowl/mug/bucket, usb drive).
const Synonyms = [["eraser", "rubber"], ["cup", "mug", "bowl", "bucket", "glass"], ["usb", "drive", "flash", "thumb", "stick", "dongle"],
  ["plier", "tong", "wrench"], ["cable", "wire", "cord"], ["notebook", "book", "paper"], ["box", "carton"], ["pen", "marker", "highlighter"]];
// Adverbs end the phrase ("gingerly"), except nouns that end the same way.
const LyNouns = new Set(["jelly", "belly", "lolly", "holly", "lily", "fly", "family", "assembly", "butterfly", "dragonfly", "supply"]);
// Parts of the robot are never the task object.
const RobotWords = new Set(["robot", "robotic", "arm", "gripper", "claw", "jaw"]);

export const Singular = (Word: string) => Word.replace(/(?:x|s|ch|sh)es$/, Match => Match.slice(0, -2)).replace(/([^s])s$/, "$1");
const Colour = (Word: string) => (Word === "gray" ? "grey" : Word);
const IsArm = (Word: string) => ArmWords.has(Word) || ArmWords.has(Singular(Word));
// Checked before and after singularising: "jaws" is a jaw, but Singular("this") is "thi".
const IsNotObject = (Word: string) => NotObjects.has(Word) || NotObjects.has(Singular(Word));

/** Damerau-Levenshtein (optimal string alignment) distance. */
export function EditDistance(A: string, B: string): number {
  const D = Array.from({ length: A.length + 1 }, (_, I) => [I, ...Array<number>(B.length).fill(0)]);
  for (let J = 1; J <= B.length; J++) D[0][J] = J;
  for (let I = 1; I <= A.length; I++) {
    for (let J = 1; J <= B.length; J++) {
      D[I][J] = Math.min(D[I - 1][J] + 1, D[I][J - 1] + 1, D[I - 1][J - 1] + (A[I - 1] === B[J - 1] ? 0 : 1));
      if (I > 1 && J > 1 && A[I - 1] === B[J - 2] && A[I - 2] === B[J - 1]) D[I][J] = Math.min(D[I][J], D[I - 2][J - 2] + 1);
    }
  }
  return D[A.length][B.length];
}

function Known(Word: string) {
  return Verbs.includes(Word) || Stops.has(Word) || Skip.has(Word) || GenericNouns.has(Word) || Colours.has(Colour(Word)) || Sizes.has(Word)
    || Materials.has(Word) || IsNotObject(Word) || IsArm(Word) || DeskNouns.has(Word) || DeskNouns.has(Singular(Word)) || NotTypos.has(Word)
    || QuestionLeads.has(Word);
}

/** Split words glued to a preposition or an article ("thingon" -> "thing on"), never a real word ("carton", "button"). */
function Repair(Tokens: string[], Repaired: string[]): string[] {
  const Content = (Part: string) => Part.length >= 3 && Known(Part) && !Stops.has(Part) && !Articles.has(Part);
  return Tokens.flatMap(Word => {
    if (Word.length < 5 || Known(Word) || /\d/.test(Word)) return [Word];
    for (let Cut = Word.length - 2; Cut >= 2; Cut--) {
      const Left = Word.slice(0, Cut), Right = Word.slice(Cut);
      if ((Content(Left) && (Stops.has(Right) || Articles.has(Right))) || (Articles.has(Left) && Content(Right))) {
        Repaired.push(`${Word} -> ${Left} ${Right}`);
        return [Left, Right];
      }
    }
    return [Word];
  });
}

const PhraseMaxChars = 80;

/**
 * The object a task acts on, or null. The verb may carry one typo ("Grabe",
 * "pickk"), only when the task has no exact verb, the word is not an ordinary
 * one where it stands ("the plate", "told"), and the task is not a question
 * or a description: a false verb then only asks for measured positions.
 */
export function ParseTaskObject(Task: string): TaskObjectQuery | null {
  const Repaired: string[] = [];
  const Tokens = Repair(Task.toLowerCase().match(/[a-z]+(?:-[a-z]+)*|\d+(?:\.\d+)?/g) ?? [], Repaired);
  const IsVerbAt = (Index: number) => Verbs.includes(Tokens[Index]) && (Tokens[Index] !== "hand" || Handed.has(Tokens[Index + 1] ?? ""));
  let Starts = Tokens.flatMap((_, Index) => (IsVerbAt(Index) ? [Index] : []));
  let VerbFuzzy = false;
  if (!Starts.length && !QuestionLeads.has(Tokens[0] ?? "")) {
    // One typo in the verb: anywhere for words of 4+ letters, and from 3 letters for the first word ("grb the eraser").
    const Fix = (Word: string, Index: number) => Word.length >= (Index === 0 ? 3 : 4) && !Known(Word)
      && !(Index > 0 && Determiners.has(Tokens[Index - 1]))
      ? Verbs.find(Verb => Verb.length >= 4 && EditDistance(Word, Verb) === 1) : undefined;
    Starts = Tokens.flatMap((Word, Index) => (Fix(Word, Index) ? [Index] : []));
    for (const Index of Starts) Tokens[Index] = Fix(Tokens[Index], Index)!;
    VerbFuzzy = Starts.length > 0;
  }
  for (const Start of Starts) {
    const Verb = Tokens[Start];
    let Index = Start + 1;
    if (["up", "down", "out", "away"].includes(Tokens[Index])) Index++;
    if ((Verb === "reach" && Tokens[Index] === "for") || (Verb === "point" && (Tokens[Index] === "at" || Tokens[Index] === "to"))
      || (Verb === "hover" && Tokens[Index] === "over")) Index++;
    while (Index < Tokens.length && (Articles.has(Tokens[Index]) || Tokens[Index] === "me" || Tokens[Index] === "us")) Index++;
    // The arm (or nothing) as the object: the object follows the first spatial
    // preposition. Without one the arm itself is what moves ("open the gripper wider").
    const NoDirectObject = Index >= Tokens.length || Spatial.has(Tokens[Index]) || Skip.has(Tokens[Index]) || /^\d/.test(Tokens[Index]);
    // Then the verb moves the arm, not the object: "on the desk" says where the object is.
    const ArmMoves = IsArm(Tokens[Index] ?? "") || (MotionVerbs.has(Verb) && NoDirectObject);
    if (ArmMoves) {
      while (Index < Tokens.length && (IsArm(Tokens[Index]) || Articles.has(Tokens[Index]))) Index++;
      const Next = Tokens.slice(Index).findIndex(Word => Spatial.has(Word));
      if (Next < 0) continue;
      Index += Next + 1;
    }
    while (Index < Tokens.length && (Skip.has(Tokens[Index]) || /^\d/.test(Tokens[Index]) || Spatial.has(Tokens[Index]))) Index++;
    const Words: string[] = [];
    let End = Index;
    for (; End < Tokens.length && Words.length < 4; End++) {
      const Word = Tokens[End];
      if (Stops.has(Word) || (/ly$/.test(Word) && !LyNouns.has(Word)) || (Words.length && IsVerbAt(End))) break;
      if (Articles.has(Word) || IsNotObject(Word) || IsArm(Word) || /^\d/.test(Word)) continue;
      Words.push(Singular(Colour(Word)));
    }
    if (!Words.length) continue;
    const Adjective = (Word: string, At: number) => Colours.has(Word) || Sizes.has(Word) || (Materials.has(Word) && At < Words.length - 1);
    const Adjectives = Words.filter(Adjective);
    const Nouns = Words.filter((Word, At) => !Adjective(Word, At));
    // "usb thing", "chess piece": the noun before a generic one names it.
    const Concrete = Nouns.filter(Word => !GenericNouns.has(Word));
    const Head = Nouns.length && !GenericNouns.has(Nouns[Nouns.length - 1]) ? Nouns[Nouns.length - 1] : Concrete[Concrete.length - 1] ?? null;
    const Rest = Tokens.slice(End, End + 6).join(" ");
    const Where = Rest.match(/^(?:(?:that|which) (?:is |s )?)?(on|onto|in|inside|from|off) (?:top of )?(?:the |my |your )?(desk|table|bin|basket|shelf|platform|stand|riser|ledge|pedestal)\b/);
    const Destination = Where && PlaceVerbs.has(Verb) && !ArmMoves && Where[1] !== "from" && Where[1] !== "off";
    const Support = !Where || Destination ? null : Where[2] === "desk" || Where[2] === "table" ? "desk"
      : Where[2] === "bin" || Where[2] === "basket" ? "bin" : "surface";
    const Name = [...Adjectives, ...(Head ? Nouns : ["thing"])].join(" ").slice(0, PhraseMaxChars);
    const Phrase = (Name + (Support === "desk" ? " on the desk" : Support === "bin" ? " in the bin" : Support === "surface" ? ` on the ${Where![2]}` : ""))
      .slice(0, PhraseMaxChars);
    return { Head, Generic: !Head, Colours: Words.filter(Word => Colours.has(Word)), Sizes: Words.filter(Word => Sizes.has(Word)),
      Nouns: Head ? Nouns : [], Support, Name, Phrase, VerbFuzzy, Repaired, Verb, ArmMoves };
  }
  return null;
}

// What the task does to its object. A grasp gets the fixed-jaw GRASP PLAN
// (GraspPlan.ts); a contact task touches it, so the guard lets the gripper
// reach it; anything else (hover, reach, point, moving the arm to it) only
// goes above it.
const GraspVerbs = new Set(["grab", "grasp", "pick", "take", "get", "fetch", "bring", "hold", "lift", "carry", "put", "place", "drop", "stack",
  "hand", "give", "throw", "toss", "remove", "insert", "collect", "grip", "catch", "retrieve", "move"]);
const ContactVerbs = new Set(["touch", "tap", "poke", "press", "push", "pull", "slide", "drag", "flip", "knock", "open"]);
export type TaskIntentKind = "grasp" | "contact" | "approach";
export function TaskIntent(Query: Pick<TaskObjectQuery, "Verb" | "ArmMoves"> | null | undefined): TaskIntentKind {
  if (!Query) return "approach";
  if (ContactVerbs.has(Query.Verb)) return "contact";
  return GraspVerbs.has(Query.Verb) && !Query.ArmMoves ? "grasp" : "approach";
}

// --- Which measured object it is ---------------------------------------------

type Point = { x: number; y: number };
export type TaskObjectSource = "query" | "name" | "position" | "colour" | "estimate";
export type TaskObjectBinding = {
  Object: LocatedObject;
  Source: TaskObjectSource;
  // From where it was last measured (position), from the model's estimate
  // (estimate), otherwise from where it was last measured if it was.
  DistanceCm: number | null;
};
export type TaskObjectResolution = { Binding: TaskObjectBinding | null; Candidates: LocatedObject[] };

// A task object re-measured under a new label is the same object within this
// distance of where it was last measured: above twice the largest jitter seen
// (1.26 cm), below half the smallest gap to another object (6.2 cm).
export const TrackCm = 3;
// Once measured, the task object is only ever re-bound within this distance of
// where it was last measured (or of the tip after a gripper close, as the jaws
// carry it). With the object hidden under the gripper the task query matched
// the mouse 72 cm away and the task ended "out of reach" (2026-09-29 22:47).
export const RebindGateCm = 8;
// The model's target_estimate_cm names an object within this distance of it.
export const EstimateGateCm = 3;
// Objects closer than this are one thing measured under two labels (cup/bowl, pliers/cable in the logs).
const SameSpotCm = 1.5;
// A tracked object's base may move this much between check-ins; a box on the
// jaws hovering over the spot sits far higher.
const TrackBaseCm = 3;

const Xy = (A: Point, B: Point) => Math.hypot(A.x - B.x, A.y - B.y);
/**
 * Whether a measured object stands where the task says: on the desk (not in the bin, not on a shelf or platform), in
 * the bin, or on a shelf or platform (a declared surface, back end/DeclaredSurfaces.json). With nothing measured on a
 * declared surface (none declared), "on the shelf" picks among them all, as before such surfaces were measured.
 */
const SupportFits = (Query: TaskObjectQuery, Object_: LocatedObject, Objects: LocatedObject[]) => Query.Support === "desk"
  ? Object_.supportBranch !== "bin" && Object_.supportBranch !== "surface"
  : Query.Support === "bin" ? Object_.supportBranch !== "desk" && Object_.supportBranch !== "surface"
    : Query.Support === "surface" ? Object_.supportBranch === "surface" || !Objects.some(Other => Other.supportBranch === "surface") : true;
// Mirrors IsUnreliable (AiMeasurements.ts), which imports this file: of two
// labels at one spot, a box the bridge refined is picked over one it did not.
const PlanarExtrapolated = (Object_: LocatedObject) => Object_.extrapolated && !HasVerifiedDepthFrame(Object_);
const Reliable = (Object_: LocatedObject) => !PlanarExtrapolated(Object_) && !Object_.unstable && !Object_.taskQueryOnly && Object_.refined !== false;
const Best = (Objects: LocatedObject[]) => Objects.find(Object_ => Reliable(Object_) && Object_.depthMeasured)
  ?? Objects.find(Reliable) ?? Objects[0] ?? null;
const LabelWords = (Label: string) => Label.toLowerCase().split(/[^a-z]+/).filter(Boolean).map(Word => Singular(Colour(Word)));
const LabelColours = (Label: string) => LabelWords(Label).filter(Word => Colours.has(Word));
const LabelHead = (Label: string) => LabelWords(Label).filter(Word => !Colours.has(Word) && !Sizes.has(Word)).pop() ?? null;
// "small black object", "black", "black object on tape" (its noun before the preposition): a label that names no kind of thing.
const GenericLabel = (Label: string) => {
  const Words = LabelWords(Label), Cut = Words.findIndex(Word => Stops.has(Word));
  const Head = LabelHead((Cut < 0 ? Words : Words.slice(0, Cut)).join(" "));
  return !Head || GenericNouns.has(Head);
};
// A kind of desk object the detector has names for. The user's own word for
// anything else may never appear in a label: on 2026-09-30 05:24-05:27 the
// detector called the "black servo motor thing" "small black object", "black
// object on tape", "small black box", "usb drive" or "battery", never a servo.
const KnownKind = (Word: string) => DeskNouns.has(Word) || Synonyms.some(Group => Group.includes(Word)) || RobotWords.has(Word);
const RobotPart = (Label: string) => LabelWords(Label).some(Word => RobotWords.has(Word));
const ColourConflict = (Query: TaskObjectQuery, Label: string) => {
  const Named = LabelColours(Label);
  return Query.Colours.length > 0 && Named.length > 0 && !Named.some(Word => Query.Colours.includes(Word));
};
// The same word, a synonym, or a typo of 5+ letters in a word the user typed that is not itself a known one.
const SameThing = (Asked: string, Word: string) => Asked === Word
  || Synonyms.some(Group => Group.includes(Asked) && Group.includes(Word))
  || (!DeskNouns.has(Asked) && Math.min(Asked.length, Word.length) >= 5 && EditDistance(Asked, Word) <= (Math.min(Asked.length, Word.length) >= 8 ? 2 : 1));

/**
 * Whether a label names the query's head noun: the same word, a synonym or a
 * typo, never with a clashing colour. A synonym or typo must keep the phrase's
 * other nouns: a "glue stick" is not the "usb drive".
 */
export function NamesQuery(Query: TaskObjectQuery, Label: string): boolean {
  if (!Query.Head || ColourConflict(Query, Label)) return false;
  const Words = LabelWords(Label);
  if (Words.includes(Query.Head)) return true;
  if (!Words.some(Word => SameThing(Query.Head!, Word))) return false;
  return Query.Nouns.filter(Word => Word !== Query.Head && !GenericNouns.has(Word)).every(Word => Words.includes(Word));
}

// Of the labels at one spot, the one the task most likely means: one naming it,
// one saying its colour, one naming no kind of thing, then the best measured.
// The servo on the clear tape roll was "tape", "tape roll" and "small black
// object" within 1 cm of each other (2026-09-30 05:24).
function Preferred(Query: TaskObjectQuery, Spot: LocatedObject[]) {
  return Best(Spot.filter(Object_ => NamesQuery(Query, Object_.label)))
    ?? Best(Spot.filter(Object_ => LabelColours(Object_.label).some(Word => Query.Colours.includes(Word))))
    ?? Best(Spot.filter(Object_ => GenericLabel(Object_.label))) ?? Best(Spot);
}

function Clusters(Objects: LocatedObject[]): LocatedObject[][] {
  const Groups: LocatedObject[][] = [];
  for (const Object_ of Objects) {
    const Group = Groups.find(Members => Members.some(Other => Xy(Other, Object_) <= SameSpotCm));
    if (Group) Group.push(Object_); else Groups.push([Object_]);
  }
  return Groups;
}

// One spot among these objects, or the one nearest where the object was last
// measured; null when they are at several spots and nothing tells them apart.
function OneSpot(Objects: LocatedObject[], LastSeen: LocatedObject | null) {
  const Spots = Clusters(Objects);
  if (Spots.length === 1) return Best(Spots[0]);
  if (Spots.length > 1 && LastSeen) return Objects.slice().sort((A, B) => Xy(A, LastSeen) - Xy(B, LastSeen))[0];
  return null;
}

/**
 * This check-in's measurement of the task object, and the other candidates
 * when none is bound. Rules in order: named and picked by the bridge's task
 * query; picked by the task query when the phrase has colour or size words
 * and the label is compatible; named (label, synonym or typo); picked by the
 * task query; tracked within TrackCm of where it was last measured (the
 * grounding renamed 7 of 17 small desk objects between check-ins); for a
 * generic phrase, the one label that says its colour; for a noun no label
 * names, the one stand-in label (StandIn) that says its colour, else the
 * stand-ins as candidates. LastSeen is in the AI's frame; Track is off once
 * the gripper closed (the object may be in the jaws).
 */
export function ResolveTaskObject(Query: TaskObjectQuery, Objects: LocatedObject[],
  Options: { LastSeen?: LocatedObject | null; Track?: boolean; Tip?: Point | null } = {}): TaskObjectResolution {
  const LastSeen = Options.LastSeen ?? null;
  const Tip = Options.Track === false ? Options.Tip ?? null : null;
  const NearLast = (Object_: LocatedObject) => !LastSeen || Xy(Object_, LastSeen) <= RebindGateCm
    || (Tip != null && Xy(Object_, Tip) <= RebindGateCm);
  const Distance = (Object_: LocatedObject) => (LastSeen ? Xy(Object_, LastSeen) : null);
  const Bound = (Object_: LocatedObject, Source: TaskObjectSource, Candidates: LocatedObject[]): TaskObjectResolution =>
    ({ Binding: { Object: Object_, Source, DistanceCm: Distance(Object_) }, Candidates });
  const Usable = Objects.filter(Object_ => !RobotPart(Object_.label) && NearLast(Object_) && SupportFits(Query, Object_, Objects));
  // A label without a colour does not escape a clash through a same-spot twin ("box" beside "blue box").
  const SiblingClash = (Object_: LocatedObject) => Objects.some(Other => Other !== Object_ && Xy(Other, Object_) <= SameSpotCm
    && ColourConflict(Query, Other.label));
  const Fits = (Object_: LocatedObject) => !ColourConflict(Query, Object_.label) && !SiblingClash(Object_);
  let Named = Usable.filter(Object_ => NamesQuery(Query, Object_.label) && !SiblingClash(Object_));
  // Another noun of the phrase, as a label's own noun ("pen" of "the pen holder").
  if (!Named.length) Named = Usable.filter(Object_ => Fits(Object_) && Query.Nouns.slice(0, -1).includes(LabelHead(Object_.label) ?? ""));
  const Queried = Usable.filter(Object_ => Object_.taskMatch && Fits(Object_));
  // A label that may be the object under another name: any for a generic
  // phrase; for a noun, one naming no kind of thing, or any when the app does
  // not know the noun as a desk object (KnownKind).
  const StandIn = (Object_: LocatedObject) => !RobotWords.has(Query.Head ?? "")
    && (Query.Generic || GenericLabel(Object_.label) || !KnownKind(Query.Head ?? ""));
  // The spots whose label says the phrase's colour, where the task says (a black cable in the bin is not the desk's black thing).
  const ColourSpots = (Objects: LocatedObject[]) => Clusters(Objects.filter(Object_ => LabelColours(Object_.label).some(Word => Query.Colours.includes(Word))
    && (Query.Support === "bin" || Object_.supportBranch !== "bin")));

  const Both = OneSpot(Named.filter(Object_ => Object_.taskMatch), LastSeen);
  if (Both) return Bound(Both, "query", Named.filter(Object_ => Object_.taskMatch));
  // "black thing": the query is more specific than any label. Never a label
  // naming another kind of thing ("mouse" for "red eraser").
  if (Query.Colours.length || Query.Sizes.length) {
    const Compatible = Queried.filter(Object_ => Query.Generic || GenericLabel(Object_.label));
    // A box only the query found, outside the calibrated area, yields to the one
    // label inside it that says the colour: the "black thing" query boxed the far
    // mouse or a black shirt in 10 of 15 recorded queries, and at 2026-09-30 05:28
    // a box 42 cm out ended the task OUT OF REACH beside the "black object on tape".
    if (!LastSeen && Compatible.length && Compatible.every(Object_ => Object_.taskQueryOnly && PlanarExtrapolated(Object_))) {
      const Spots = ColourSpots(Usable.filter(Object_ => Fits(Object_) && StandIn(Object_) && !PlanarExtrapolated(Object_) && !Object_.taskQueryOnly));
      if (Spots.length === 1) return Bound(Best(Spots[0])!, "colour", Spots[0]);
    }
    const Picked = OneSpot(Compatible, LastSeen);
    if (Picked) return Bound(Picked, "query", Compatible);
  }
  if (Named.length) {
    const Picked = OneSpot(Named, LastSeen);
    return Picked ? Bound(Picked, "name", Named) : { Binding: null, Candidates: Clusters(Named).map(Spot => Best(Spot)!) };
  }
  const Picked = OneSpot(Queried, LastSeen);
  if (Picked) return Bound(Picked, "query", Queried);
  if (LastSeen) {
    if (Options.Track === false) return { Binding: null, Candidates: [] };
    const Size = (Object_: LocatedObject) => (Object_.sizeCm ? Math.max(...Object_.sizeCm) : null);
    const Seen = Size(LastSeen);
    const Near = Usable.filter(Object_ => Xy(Object_, LastSeen) <= TrackCm && Math.abs(Object_.z - LastSeen.z) <= TrackBaseCm
      && (Size(Object_) == null || Seen == null || Size(Object_)! <= 2.5 * Seen + 2) && Fits(Object_));
    const Nearest = Near.sort((A, B) => Xy(A, LastSeen) - Xy(B, LastSeen))[0];
    return Nearest ? Bound(Nearest, "position", Near) : { Binding: null, Candidates: [] };
  }
  const Open = Usable.filter(Fits);
  if (Queried.length) return { Binding: null, Candidates: Clusters(Queried).map(Spot => Best(Spot)!) };
  if (Query.Generic && Query.Colours.length) {
    // Only where the task says (a black cable in the bin is not the desk's black thing).
    const Coloured = Open.filter(Object_ => LabelColours(Object_.label).some(Word => Query.Colours.includes(Word))
      && (Query.Support === "bin" || Object_.supportBranch !== "bin"));
    const Spots = Clusters(Coloured);
    if (Spots.length === 1) return Bound(Best(Spots[0])!, "colour", Spots[0]);
    if (Spots.length > 1) return { Binding: null, Candidates: Spots.map(Spot => Best(Spot)!) };
  }
  // A noun no label names and the query did not find ("black servo motor
  // thing" at 05:24, which the model was told was NOT detected while the
  // "small black object" sat in view): the labels that may be it stand in. The
  // one such spot labelled the phrase's colour is bound; otherwise they are
  // the candidates the model may point at with target_estimate_cm.
  if (Query.Head) {
    const StandIns = Open.filter(Object_ => StandIn(Object_) && (Query.Support === "bin" || Object_.supportBranch !== "bin"));
    if (Query.Colours.length) {
      const Spots = ColourSpots(StandIns);
      if (Spots.length === 1) return Bound(Best(Spots[0])!, "colour", Spots[0]);
      if (Spots.length > 1) return { Binding: null, Candidates: Spots.map(Spot => Best(Spot)!) };
    }
    return { Binding: null, Candidates: Clusters(StandIns).map(Spot => Preferred(Query, Spot)!) };
  }
  return { Binding: null, Candidates: Query.Generic ? Clusters(Open).map(Spot => Best(Spot)!) : [] };
}

/**
 * After a reply, while the task object was never bound: the one measured
 * object within GateCm of the model's target_estimate_cm (the AI's frame),
 * unless its label names another kind of thing than the task's noun ("Grab
 * the eraser" never binds the binder the model pointed at), a clashing
 * colour, or the other surface (desk or bin) than the task says.
 */
export function BindFromEstimate(Query: TaskObjectQuery, Objects: LocatedObject[], Estimate: Point | null,
  GateCm = EstimateGateCm): TaskObjectBinding | null {
  if (!Estimate) return null;
  const Near = Objects.filter(Object_ => Xy(Object_, Estimate) <= GateCm && !RobotPart(Object_.label) && SupportFits(Query, Object_, Objects));
  const Spots = Clusters(Near);
  if (Spots.length !== 1) return null;
  const Spot = Spots[0];
  if (Spot.some(Object_ => ColourConflict(Query, Object_.label))) return null;
  // A noun the app does not know as a desk object ("servo") names no label it could clash with; the robot is never it.
  if (Query.Head && RobotWords.has(Query.Head)) return null;
  if (Query.Head && KnownKind(Query.Head) && !Spot.some(Object_ => NamesQuery(Query, Object_.label) || GenericLabel(Object_.label))) return null;
  const Pick = Preferred(Query, Spot)!;
  return { Object: Pick, Source: "estimate", DistanceCm: Xy(Pick, Estimate) };
}

// --- An answer to the model's question ------------------------------------------

/**
 * The task the model asked a question in, kept for the next prompt: its text,
 * the question, its object. Stopped: the app stopped it instead, and Question
 * is why and what it asked the user to do before sending it again.
 */
export type AskedTask = { Task: string; Question: string; Query: TaskObjectQuery | null; Stopped?: boolean };
// What an answer opens with before it names an object ("yes, it's the small black object", "no, the black box").
const AnswerLeads = new Set(["yes", "yeah", "yep", "yup", "ya", "ok", "okay", "sure", "correct", "right", "exactly", "no", "nope", "nah",
  "it", "its", "s", "is", "that", "thats", "this", "i", "mean", "meant", "please", "so"]);
// A prompt that asks about the scene, or commands the arm, is no answer.
const AskLeads = new Set(["describe", "tell", "what", "whats", "where", "how", "why", "which", "who", "list", "count", "explain"]);
const CommandWords = new Set(["close", "rotate", "tilt", "turn", "stop", "home", "relax", "wave", "calibrate", "reset", "cm", "mm", "inch", "inches",
  "degree", "degrees"]);
// A verb on these goes on with the asked task ("grab that", "go ahead", "carry on").
const Pronouns = new Set(["it", "that", "this", "them", "one", "ahead", "on"]);
const QuestionMaxChars = 200;

/**
 * A reply to the model's question goes on with the asked task. On 2026-09-30
 * 05:25-05:28 "yes, its the small black object on the desk", "the black box
 * near the tape roll" and "yes grab that" each started a new task with no
 * object and no memory of the question, and the model went for the can. The
 * object is the one the answer describes, with the asked task's verb, else
 * the asked task's own. Null when nothing was asked, or the prompt is a task
 * of its own: one naming its object, a question, or another command ("move up
 * 3 cm", "open the gripper").
 */
export function ContinueAskedTask(Answer: string, Asked: AskedTask | null): { Task: string; Query: TaskObjectQuery | null; Stopped?: boolean } | null {
  const Text = Answer.trim();
  if (Asked?.Stopped) return ContinueStoppedTask(Text, Asked);
  if (!Asked || !Text || /\?\s*$/.test(Text) || ParseTaskObject(Text)) return null;
  const Tokens = Text.toLowerCase().match(/[a-z]+(?:-[a-z]+)*|\d+(?:\.\d+)?/g) ?? [];
  if (AskLeads.has(Tokens[0] ?? "") || Tokens.some(Word => IsArm(Word) || CommandWords.has(Word))) return null;
  // A verb only on "it" or "that" ("yes grab that", "pick it up"); any other is a command of its own.
  const VerbAt = Tokens.findIndex(Word => Verbs.includes(Word));
  if (VerbAt >= 0 && !Pronouns.has(Tokens[VerbAt + 1] ?? "")) return null;
  let Start = 0;
  while (Start < Tokens.length && AnswerLeads.has(Tokens[Start])) Start++;
  const Rest = VerbAt >= 0 ? [] : Tokens.slice(Start);
  const Task = OriginalTask(Asked.Task);
  const Verb = Asked.Query?.Verb ?? (Task.toLowerCase().match(/[a-z]+/g) ?? []).find(Word => Verbs.includes(Word)) ?? null;
  // It describes an object when its words start with an article or say a colour, size or kind.
  const Describes = Rest.length > 0 && (Articles.has(Rest[0]) || Rest.some(Word => Colours.has(Colour(Word)) || Sizes.has(Word)
    || DeskNouns.has(Singular(Word)) || GenericNouns.has(Word)));
  const Described = Verb && Describes ? ParseTaskObject(`${Verb} ${Rest.join(" ")}`) : null;
  // "the left one" says nothing the asked object did not.
  const Query = Described && !(Described.Generic && !Described.Colours.length && !Described.Sizes.length) ? Described : Asked.Query;
  const Question = Asked.Question.trim().replace(/\s+/g, " ").slice(0, QuestionMaxChars);
  return { Task: `${Task} (you asked: "${Question}"; the user answered: "${Text}")`, Query };
}

// The task as first typed, without what an earlier answer or stop added to it.
const OriginalTask = (Task: string) => Task.replace(/ \((?:you asked|the app stopped it): [\s\S]*$/, "").trim();
// What a user says back when they did what a stop asked, or want it tried again.
const GoAheads = new Set(["ok", "okay", "k", "kk", "yes", "yeah", "yep", "yup", "ya", "sure", "alright", "fine", "cool", "good", "great",
  "done", "did", "do", "it", "that", "this", "those", "them", "i", "ive", "ve", "have", "has", "is", "its", "s", "now", "try", "again", "retry",
  "go", "ahead", "continue", "carry", "on", "proceed", "resume", "please", "pls", "plz", "thanks", "thank", "you", "so", "and", "then", "all",
  "set", "ready", "there", "fixed", "finished", "redo", "once", "more", "time"]);
// A report of what the user did ("i moved the battery away"): its object words are not a new task.
const Reports = new Set(["moved", "put", "placed", "pushed", "pulled", "shifted", "slid", "turned", "rotated", "removed", "took", "cleared",
  "set", "stood", "made", "gave", "brought", "fixed"]);
const MaxGoAheadWords = 10;

/**
 * A short go-ahead after the app stopped a task and asked the user to do
 * something and send it again ("ok i did that", "done", "try again", "now
 * grab it", "i moved the battery away"): the stopped task goes on, its object
 * and why it stopped included. On 2026-10-01 07:56 "ok i did that" became a
 * task of its own with no object, and the model went for the tape roll; at
 * 08:00 "now grab it" had the model ask which object. A task of its own (a
 * verb on an object: "grab the can"), a question or another command is not one.
 */
function ContinueStoppedTask(Text: string, Stopped: AskedTask): { Task: string; Query: TaskObjectQuery | null; Stopped: true } | null {
  if (!Text || /\?\s*$/.test(Text)) return null;
  const Tokens = Text.toLowerCase().replace(/[’']/g, "").match(/[a-z]+(?:-[a-z]+)*|\d+(?:\.\d+)?/g) ?? [];
  const First = Tokens[0] ?? "";
  if (!First || Tokens.length > MaxGoAheadWords || AskLeads.has(First) || (QuestionLeads.has(First) && !GoAheads.has(First))) return null;
  if (Tokens.some(Word => IsArm(Word) || CommandWords.has(Word) || /^\d/.test(Word))) return null;
  // A verb leads a task of its own unless it acts on "it" ("now grab it", "go ahead", "go"), or
  // someone did it ("i put the battery away"; "put the tape in the basket" is a task).
  const Followers = new Set([...Pronouns, "again", "now", "please"]);
  const Subjects = new Set(["i", "ive", "we", "weve", "you", "have", "has", "just", "already"]);
  const Imperative = Tokens.some((Word, Index) => Verbs.includes(Word) && !(Index > 0 && Subjects.has(Tokens[Index - 1]))
    && Tokens[Index + 1] !== undefined && !Followers.has(Tokens[Index + 1]));
  // A misspelt verb on an object ("ok grabe the cup") too; a report reads as one ("i moved the can": move).
  if (Imperative || (ParseTaskObject(Text)?.VerbFuzzy && !Tokens.some(Word => Reports.has(Word)))) return null;
  // It says it is done or goes ahead: one of those words, or only a verb on "it".
  if (!Tokens.some(Word => GoAheads.has(Word) || Reports.has(Word) || Verbs.includes(Word))) return null;
  const Why = Stopped.Question.trim().replace(/\s+/g, " ").slice(0, StopReasonMaxChars);
  return { Task: `${OriginalTask(Stopped.Task)} (the app stopped it: "${Why}"; the user answered: "${Text}")`, Query: Stopped.Query, Stopped: true };
}
const StopReasonMaxChars = 320;
