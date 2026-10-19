import { chromium as Chromium } from "playwright";
import { CadReadyMs, LaunchOptions, StartUiServer } from "./UiTestLib.mjs";
import { readFileSync as ReadFile, mkdirSync as MakeDirectory } from "node:fs";
import { execFileSync as Execute } from "node:child_process";
import { fileURLToPath as FilePath } from "node:url";
import Path from "node:path";
import Assert from "node:assert/strict";

// The dashboard fits the window: no page scroll, no panel or column scrollbar,
// nothing clipped, and the key controls inside the viewport, in Manual and AI
// mode, from 1280x720 to 3840x2160 and on the user's own screen (3840x2160 at
// 225 % scaling: a maximised and a full-screen browser, and the maximised one
// at 110 % and 125 % browser zoom, each at its real device pixel ratio), for
// the default and user-resized tilings and the heaviest content (a chained
// approval card, IK issues, a long reply, the task strip, the camera gate
// banner, the selected joint's dial, a bridge update and a protection trip).
// Text stays readable, and Clancy's pixel art stays crisp (one art pixel is a
// whole number of device pixels). Mocked bridge, camera and model: nothing
// reaches the real bridge or arm, and the test never approves a move.
// It serves this checkout on 5191-5199 itself (UiTestLib.mjs); CadTestUrl picks a test server of your own.
// NoScrollShots=<folder> moves the screenshots; NoScrollReport=1 lists every
// problem instead of stopping at the first state that has one.
const Root = Path.dirname(FilePath(import.meta.url)), Backend = Path.resolve(Root, "../back end");
const Python = ReadFile(Path.join(Root, "../.backend-python.txt"), "utf8").trim();
const State = JSON.parse(Execute(Python, ["-c", "import json, bridge_server; print(json.dumps(bridge_server.BridgeController().state_snapshot()))"], { cwd: Backend, encoding: "utf8" }));
Object.assign(State, { arm_connected: true, torque: true, port: "FAKE", motion_epoch: 5, motion_route: null, motion_fault: null });
for (const Servo of Object.values(State.servos)) Servo.moving = false;
// Closed jaws, so "open the gripper and move up" is a two-step chained motion: the
// tallest chain card. A bridge that opens the gripper on the way shows one step, so
// this layout check runs as a bridge without it.
Object.assign(State.servos.gripper, { present_raw: 699, target_raw: 699 });
delete State.gripper_with_tip_version;
const Output = process.env.NoScrollShots ?? Path.join(Backend, "TestArtifacts", "NoScroll"); MakeDirectory(Output, { recursive: true });
const Report = process.env.NoScrollReport === "1";
// Each group runs the whole sequence in a browser at that device pixel ratio. Zoom is the browser
// zoom the group stands for: it divides the CSS viewport and multiplies the pixel ratio.
const Groups = [
  { Name: "", Dpr: 1, Zoom: 1, Sizes: [[1280, 720], [1366, 768], [1600, 900], [1920, 1080], [2560, 1440], [3840, 2160]] },
  // 3840x2160 at 225 %: 1707 CSS px wide; maximised Edge with tabs and the taskbar leaves about 830 px, full screen 960.
  { Name: "Dpr2.25-", Dpr: 2.25, Zoom: 1, Sizes: [[1707, 830], [1707, 960]] },
  { Name: "Dpr2.25-Zoom110-", Dpr: 2.475, Zoom: 1.1, Sizes: [[1552, 755]] },
  { Name: "Dpr2.25-Zoom125-", Dpr: 2.8125, Zoom: 1.25, Sizes: [[1365, 664]] },
];
// Text floor in CSS px times the browser zoom, after any fit scale: the default tiling's panels fit
// unscaled (they compact first), so their text keeps its full size; a column the user narrowed to its
// minimum may scale a little.
const MinimumFontPx = 11, ResizedMinimumFontPx = 10;
const Server = await StartUiServer();
const Browser = await Chromium.launch(LaunchOptions({ args: ["--use-fake-device-for-media-stream", "--use-fake-ui-for-media-stream"] }));
const Errors = [], Commands = [], Timers = new Set(), Problems = [], Shots = [];
let Replies = [];
const CameraNotVerified = { ok: false, error_code: "camera_not_verified", error: "Object positions are blocked: the calibration has no camera-position baseline.",
  camera_check: { ok: false, angle_deg: null, offset_cm: null, reason: "no baseline", reference_fitted_at: null, seq: null } };

async function OpenPage(Dpr) {
  const Context = await Browser.newContext({ viewport: { width: 1280, height: 720 }, deviceScaleFactor: Dpr, permissions: ["camera"] });
  const Page = await Context.newPage();
  Page.on("pageerror", Error => Errors.push(Error.message));
  await Page.route("**/session-log/**", Route => Route.fulfill({ json: { ok: true } }));
  await Page.route("**/bridge-launch/**", Route => Route.fulfill({ json: { ok: true } }));
  await Page.route("**/depth-server/**", Route => Route.fulfill({ json: {} }));
  await Page.route("**/ollama/**", async Route => {
    const Url = new URL(Route.request().url());
    if (Url.pathname.endsWith("/tags")) return Route.fulfill({ json: { models: [{ name: "NoScrollTest", size: 1000 }] } });
    if (Url.pathname.endsWith("/show")) return Route.fulfill({ json: { capabilities: ["vision"] } });
    if (Url.pathname.endsWith("/ps")) return Route.fulfill({ json: { models: [{ name: "NoScrollTest", context_length: 32768 }] } });
    if (!Url.pathname.endsWith("/api/chat")) return Route.fulfill({ json: { data: [{ id: "NoScrollTest" }] } });
    const Body = Route.request().postDataJSON();
    const Content = Body.messages.length === 0 ? "" : JSON.stringify(Replies.shift() ?? { response_to_user: "Nothing to do.", task_status: "done", motion_proposal: null });
    return Route.fulfill({ json: { model: Body.model, message: { role: "assistant", content: Content }, done: true, done_reason: Content ? "stop" : "load",
      prompt_eval_count: 1200, eval_count: 340 } }).catch(() => {});
  });
  // The camera check fails closed: the calibration has no drift baseline, so a
  // grasp task stops with the camera gate banner, which every later capture keeps
  // up, while a task that needs no object positions still runs.
  await Page.route("**:8000/**", Route => {
    const Url = new URL(Route.request().url());
    if (Url.pathname === "/ports") return Route.fulfill({ json: ["FAKE"] });
    if (Url.pathname === "/ik/preview") return Route.fulfill({ json: { ok: false } });
    if (Url.pathname === "/locate") return Route.fulfill({ json: CameraNotVerified });
    if (Route.request().method() !== "GET") Commands.push({ Path: Url.pathname });
    return Route.fulfill({ json: { ok: true, calibrated: false, points: [], point_count: 0 } });
  });
  await Page.routeWebSocket("**/ws", Socket => {
    Socket.send(JSON.stringify(State));
    const Timer = setInterval(() => Socket.send(JSON.stringify(State)), 100); Timers.add(Timer);
    Socket.onClose(() => { clearInterval(Timer); Timers.delete(Timer); });
    Socket.onMessage(Message => {
      const Command = JSON.parse(String(Message));
      Commands.push(Command);
      Socket.send(JSON.stringify({ type: "status", ok: true, text: Command.type === "set_motion_speed" ? `Motion speed set to ${Command.percent}%` : "OK" }));
    });
  });
  // xpath selectors are not CSS: resolve them in the page.
  await Page.addInitScript(() => {
    const Native = Document.prototype.querySelectorAll;
    Document.prototype.querySelectorAll = function (Selector) {
      if (!Selector.startsWith("xpath=")) return Native.call(this, Selector);
      const Result = document.evaluate(Selector.slice(6), document, null, XPathResult.ORDERED_NODE_SNAPSHOT_TYPE, null);
      return Array.from({ length: Result.snapshotLength }, (_, Index) => Result.snapshotItem(Index));
    };
  });
  return { Context, Page };
}

// Every problem in the current state: page scroll, a scrolling or clipped panel,
// a key control outside the window or covered, text below the floor, blurred pixel art.
const Inspect = (Page, Controls, FontFloor) => Page.evaluate(({ Controls, FontFloor }) => {
  const Found = [];
  const Name = Element => `${Element.tagName.toLowerCase()}${Element.dataset.section ? `[${Element.dataset.section}]` : ""}${[...Element.classList].map(Class => `.${Class}`).join("")}`;
  const Shown = Element => Element.getClientRects().length > 0 && getComputedStyle(Element).visibility !== "hidden";
  const Page = document.scrollingElement;
  if (Page.scrollHeight > innerHeight || Page.scrollWidth > innerWidth) Found.push(`page scrolls: ${Page.scrollWidth}x${Page.scrollHeight} in ${innerWidth}x${innerHeight}`);
  // The floating camera and the CAD canvas are overlays and a WebGL surface, not panels.
  const Panels = [...document.querySelectorAll(".app-shell *")].filter(Element => !Element.closest(".cad-host, .camera-window-layer") && Shown(Element));
  for (const Element of Panels) {
    const Style = getComputedStyle(Element);
    if (/(auto|scroll)/.test(Style.overflowY) && Element.scrollHeight > Element.clientHeight + 1) Found.push(`${Name(Element)} scrolls: ${Element.scrollHeight} > ${Element.clientHeight}`);
    if (/(auto|scroll)/.test(Style.overflowX) && Element.scrollWidth > Element.clientWidth + 1) Found.push(`${Name(Element)} scrolls sideways: ${Element.scrollWidth} > ${Element.clientWidth}`);
  }
  // Columns and sections clip instead of scrolling: their content must fit too.
  for (const Element of document.querySelectorAll(".dashboard-layout aside, .dashboard-layout .scene-column, .dashboard-layout .column-fit, .dashboard-section, .dashboard-section-content, .ai-reply-text, .settings-content")) {
    if (!Shown(Element)) continue;
    if (Element.scrollHeight > Element.clientHeight + 1 && !(Element.matches(".ai-reply-text") && Element.closest(".ai-reply")?.dataset.truncated === "true"))
      Found.push(`${Name(Element)} clips: ${Element.scrollHeight} > ${Element.clientHeight}`);
  }
  // Key controls: inside the window and the first thing under the pointer
  // (the floating camera, which the user places, is lifted for the check).
  const Layer = document.querySelector(".camera-window-layer");
  if (Layer) Layer.style.visibility = "hidden";
  for (const [Label, Selector, Expected] of Controls) {
    const Elements = [...document.querySelectorAll(Selector)].filter(Shown);
    // Expected 0: any number, at least one.
    if (Expected ? Elements.length !== Expected : !Elements.length) { Found.push(`${Label}: ${Elements.length} shown, expected ${Expected || "some"}`); continue; }
    for (const Element of Elements) {
      const Box = Element.getBoundingClientRect();
      if (Box.left < 0 || Box.top < 0 || Box.right > innerWidth + 0.5 || Box.bottom > innerHeight + 0.5 || Box.width < 4 || Box.height < 4) {
        Found.push(`${Label} outside the window: ${Math.round(Box.left)},${Math.round(Box.top)} ${Math.round(Box.width)}x${Math.round(Box.height)}`); continue;
      }
      const Hit = document.elementFromPoint(Box.left + Box.width / 2, Box.top + Box.height / 2);
      if (!Hit || !(Hit === Element || Element.contains(Hit))) Found.push(`${Label} is covered by ${Hit ? Name(Hit) : "nothing"}`);
    }
  }
  if (Layer) Layer.style.visibility = "";
  // Readable text: the smallest rendered in the columns, the top bar, the mode rail and the
  // settings drawer, fit scale included.
  let Smallest = null;
  for (const Area of document.querySelectorAll(".dashboard-layout aside, .topbar, .mode-rail, .settings-panel")) {
    if (!Shown(Area)) continue;
    const Walker = document.createTreeWalker(Area, NodeFilter.SHOW_TEXT);
    for (let Node = Walker.nextNode(); Node; Node = Walker.nextNode()) {
      const Parent = Node.parentElement;
      if (!Parent || !Node.textContent.trim() || Parent.closest("svg, [aria-hidden='true'], [popover]") || !Shown(Parent)) continue;
      const Size = parseFloat(getComputedStyle(Parent).fontSize) * (Parent.currentCSSZoom ?? 1);
      if (!Smallest || Size < Smallest.Size) Smallest = { Size, Text: Node.textContent.trim().slice(0, 40), Element: Name(Parent) };
    }
  }
  if (Smallest && Smallest.Size < FontFloor - 0.05) Found.push(`text below ${FontFloor.toFixed(1)} px: "${Smallest.Text}" (${Smallest.Element}) at ${Smallest.Size.toFixed(1)} px`);
  // Clancy's 48x48 art: each art pixel a whole number of device pixels, its corner on a device pixel
  // (within layout rounding, 1/64 px), whatever the column's fit scale.
  const Bot = document.querySelector(".ai-companion .pixel-bot-portrait");
  if (Bot && Shown(Bot)) {
    const Box = Bot.getBoundingClientRect(), Art = Box.width * devicePixelRatio / 48, X = Box.left * devicePixelRatio, Y = Box.top * devicePixelRatio;
    const Off = Value => Math.abs(Value - Math.round(Value));
    if (Off(Art) > 0.01 || Off(X) > 0.06 || Off(Y) > 0.06 || Math.abs(Box.width - Box.height) > 0.01)
      Found.push(`Clancy is not pixel-crisp: one art pixel is ${Art.toFixed(3)} device px, corner at ${X.toFixed(2)},${Y.toFixed(2)}`);
  }
  return Found;
}, { Controls, FontFloor });

const Always = [["Emergency stop", ".topbar-emergency", 1], ["Motion speed", ".motion-throttle input[type=range]", 1],
  ...["Coast", "Home", "Set Home", "Stop and hold"].map(Text => [Text, `xpath=//button[normalize-space()='${Text}']`, 1])];
const ManualControls = [...Always, ["Send", "xpath=//button[normalize-space()='Send']", 1], ["Joint sliders", ".servo-row input[type=range]", 6],
  ["Target inputs", ".target-grid input", 3], ["Wrist pitch", ".wide-field input", 1]];
const AiControls = Extra => [...Always, ["AI input", "input[aria-label='AI task']", 1], ["Send to AI", "button[title='Send to AI']", 1], ...Extra];
const LongReply = "Opening the gripper first, then raising the tip 3 cm straight up so the jaws clear the eraser and the cup beside it. "
  + "The camera check failed, so I am not using measured object positions: this move is relative to where the tip is now, and nothing here depends on where the objects are. "
  + "After both parts finish I will look again; if the jaws are not clear of the cup I will stop and ask before moving closer. "
  + "You can block this at any time, and Stop and hold stops the arm mid-move.";

async function RunGroup(Group) {
  Replies = [];
  const { Context, Page } = await OpenPage(Group.Dpr);
  const SizeName = ([Width, Height]) => `${Group.Name}${Width}x${Height}`;
  const Check = async (Label, Controls, Resized = false) => {
    for (const [Width, Height] of Group.Sizes) {
      await Page.setViewportSize({ width: Width, height: Height });
      await Page.waitForTimeout(350);
      const File = `${Label}-${SizeName([Width, Height])}.png`;
      await Page.screenshot({ path: Path.join(Output, File), animations: "disabled" });
      Shots.push(File);
      // Measured on a settled frame: Clancy is snapped to the device grid on the frame after a layout change
      // (AiCompanion's observers), so a measurement in the same frame as a change can find him a fraction off.
      await Page.evaluate(() => new Promise(Done => requestAnimationFrame(() => requestAnimationFrame(Done))));
      const Found = await Inspect(Page, Controls, (Resized ? ResizedMinimumFontPx : MinimumFontPx) / Group.Zoom);
      // The report also lists each column's density step, fit scale and its sections' natural heights.
      if (Report) console.log(`${Label} ${SizeName([Width, Height])}`, await Page.evaluate(() => [...document.querySelectorAll(".dashboard-layout aside")]
        .filter(Column => Column.getClientRects().length).map(Column => {
          const Content = Column.querySelector(":scope > .column-fit"), Scale = Content.style.zoom;
          Content.style.zoom = "";
          const Natural = `${Column.className} ${Column.clientWidth}x${Column.clientHeight} needs ${Column.scrollHeight} (step ${Content.dataset.fitDensity ?? 0}, scale ${Scale || 1}): `
            + [...Column.querySelectorAll(".dashboard-section")].filter(Section => Section.getClientRects().length)
              .map(Section => `${Section.dataset.section} ${Math.round(Section.getBoundingClientRect().height)}/${Section.scrollHeight}`
                + (Section.dataset.section === "AiControl" ? ` [${[...Section.querySelectorAll(":scope > .dashboard-section-content > *, .ai-motion-copy > *")]
                  .map(Child => `${Child.className.split(" ")[0] || Child.tagName}:${Math.round(Child.getBoundingClientRect().height)}`).join(" ")}]` : "")).join(", ");
          Content.style.zoom = Scale;
          return Natural;
        }).join("\n  ")));
      if (Found.length) {
        Problems.push(...Found.map(Problem => `${Label} ${SizeName([Width, Height])}: ${Problem}`));
        if (!Report) Assert.fail(`${Label} at ${SizeName([Width, Height])}:\n  ${Found.join("\n  ")}`);
      }
    }
  };
  const Divider = Index => Page.getByRole("separator").nth(Index);
  const Submit = async Text => {
    await Page.getByPlaceholder("Describe a task or movement").fill(Text);
    await Page.getByRole("button", { name: "Send to AI", exact: true }).click();
  };
  try {
    await Page.goto(Server.Url, { waitUntil: "domcontentloaded" });
    await Page.locator("canvas[data-cad-ready=true][data-assembly-ready=true]").waitFor({ timeout: CadReadyMs });
    await Page.evaluate(() => document.fonts.ready);
    Assert.ok(await Page.evaluate(() => document.fonts.check("400 16px Play") && document.fonts.check("700 16px Play")), "The layout is measured in the real font");
    const Ratio = await Page.evaluate(() => devicePixelRatio);
    Assert.ok(Math.abs(Ratio - Group.Dpr) < 1e-4, `The group's device pixel ratio: ${Ratio}, expected ${Group.Dpr}`);
    Assert.equal(await Page.getByRole("button", { name: "Manual control mode", exact: true }).getAttribute("aria-pressed"), "true");

    // MANUAL: the default tiling, idle; then the settings drawer over it, in columns.
    await Check("Manual-Default", ManualControls);
    await Page.getByRole("button", { name: "Settings", exact: true }).click();
    await Check("Settings", [["Close settings", "button[aria-label='Close settings']", 1], ["Reset display settings", ".settings-reset", 1],
      ["Settings controls", ".settings-panel input, .settings-panel select, .settings-panel .action-button", 0]]);
    await Page.getByRole("button", { name: "Close settings", exact: true }).click();
    // A selected joint opens its precision dial; a stale bridge and a protection
    // trip add their notices to Motion.
    await Page.getByRole("button", { name: "elbow flex", exact: true }).click();
    await Page.getByRole("slider", { name: "elbow flex precision dial" }).waitFor();
    State.speed_control_version = "throttle-v1";
    State.protection = { ...State.protection, trip: { reason: "elbow_flex stalled for 2.0 s at 58 C while holding; torque released so the arm can coast", coast_confirmed: true, at_s: 12 } };
    await Page.locator("#bridge-update-status").waitFor();
    await Page.locator(".protection-status.tripped").waitFor();
    const DialControls = [...ManualControls, ["Precision dial", ".joint-fine-control svg", 1], ["Fine adjust", ".joint-fine-step button", 2],
      ["Load bridge update", "#bridge-update-status button", 1]];
    await Check("Manual-Heavy", DialControls);
    // User-resized: both side columns at their minimum widths, then at their widest.
    await Divider(1).press("End"); await Divider(0).press("End");
    await Check("Manual-NarrowColumns", DialControls, true);
    await Divider(0).press("Home");
    await Check("Manual-WideColumns", DialControls, true);
    await Page.getByRole("button", { name: "Reset column widths", exact: true }).click();
    State.speed_control_version = "throttle-v2";
    State.protection = { ...State.protection, trip: null };
    await Page.locator("#bridge-update-status").waitFor({ state: "detached" });

    // AI: idle, then the heaviest card: the camera gate banner (a grasp task the
    // camera check stopped), a task in progress with a chained two-step proposal,
    // an estimated object position and a long reply.
    await Page.getByRole("button", { name: "AI control mode", exact: true }).click();
    await Page.waitForFunction(() => document.querySelector("video")?.dataset.captureReady === "true");
    await Page.locator('.ai-companion[data-state="ready"]').waitFor();
    await Check("Ai-Idle", AiControls([]));
    // The live view expanded over the dashboard: nothing scrolls and its own controls stay in reach.
    await Page.getByRole("button", { name: "Full screen live view", exact: true }).click();
    for (const [Width, Height] of Group.Sizes) {
      await Page.setViewportSize({ width: Width, height: Height }); await Page.waitForTimeout(350);
      const Expanded = await Page.evaluate(() => {
        const Window = document.querySelector(".camera-window.call-expanded"), Box = Window?.getBoundingClientRect(), Found = [];
        if (!Box) return ["no expanded live view"];
        if (document.scrollingElement.scrollHeight > innerHeight || document.scrollingElement.scrollWidth > innerWidth) Found.push("page scrolls");
        if (Box.left < 0 || Box.top < 0 || Box.right > innerWidth + 0.5 || Box.bottom > innerHeight + 0.5) Found.push(`live view outside the window: ${Math.round(Box.left)},${Math.round(Box.top)} ${Math.round(Box.width)}x${Math.round(Box.height)}`);
        for (const Label of ["Exit full screen live view", "Minimize Live Feed"]) {
          const Button = Window.querySelector(`button[aria-label='${Label}']`), Inner = Button?.getBoundingClientRect();
          const Hit = Inner && document.elementFromPoint(Inner.left + Inner.width / 2, Inner.top + Inner.height / 2);
          if (!Hit || !Button.contains(Hit)) Found.push(`${Label} is not reachable`);
        }
        return Found;
      });
      const File = `Ai-LiveViewExpanded-${SizeName([Width, Height])}.png`;
      await Page.screenshot({ path: Path.join(Output, File), animations: "disabled" }); Shots.push(File);
      if (Expanded.length) {
        Problems.push(...Expanded.map(Problem => `Ai-LiveViewExpanded ${SizeName([Width, Height])}: ${Problem}`));
        if (!Report) Assert.fail(`Ai-LiveViewExpanded at ${SizeName([Width, Height])}:\n  ${Expanded.join("\n  ")}`);
      }
    }
    await Page.getByRole("button", { name: "Exit full screen live view", exact: true }).click();
    await Submit("Grab the eraser");
    await Page.locator(".camera-gate-banner").waitFor();
    await Page.waitForFunction(() => !document.querySelector(".ai-task-strip") && !document.querySelector("input[aria-label='AI task']").disabled);
    Replies.push({ response_to_user: LongReply, task_status: "continue", target_estimate_cm: { left: 5, farther: 35, up: -2.3 },
      motion_proposal: { move_delta_cm: { left: 0, farther: 0, up: 3 }, gripper_before: 0.8 } });
    await Submit("open the gripper and move up 3 cm");
    await Page.getByRole("button", { name: "Approve", exact: true }).waitFor({ timeout: 20000 })
      .catch(async Failure => { throw new Error(`${Failure.message}\nAI panel: ${await Page.locator(".ai-panel").innerText()}`); });
    Assert.equal(await Page.locator(".ai-chain-steps li").count(), 2, "A chained two-step card");
    Assert.equal(await Page.locator(".ai-task-strip").count(), 1, "The task strip");
    Assert.equal(await Page.locator(".camera-gate-banner").count(), 1, "The camera gate banner stays");
    Assert.equal(await Page.locator(".ai-target-estimate").count(), 1, "The estimated object position");
    const CardControls = AiControls([["Approve", "xpath=//button[normalize-space()='Approve']", 1], ["Block", "xpath=//button[normalize-space()='Block']", 1],
      ["Stop task", ".ai-task-strip button", 1]]);
    await Check("Ai-ChainCard", CardControls);
    // At the group's first (shortest) size a long reply that does not fit shows its start; its
    // button opens all of it, unscrolled, above the workspace. At 1280x720 it must be shortened.
    const [FirstWidth, FirstHeight] = Group.Sizes[0];
    await Page.setViewportSize({ width: FirstWidth, height: FirstHeight }); await Page.waitForTimeout(350);
    const Truncated = await Page.locator(".ai-reply").getAttribute("data-truncated") === "true";
    if (FirstWidth === 1280 && FirstHeight === 720) Assert.ok(Truncated, "The long reply is shortened at 1280x720");
    if (Truncated) {
      // The button sits under the live view's default spot: the keyboard reaches it either way.
      await Page.getByRole("button", { name: "Show the whole AI reply", exact: true }).focus();
      await Page.keyboard.press("Enter");
      const FullReply = Page.getByRole("dialog", { name: "AI reply", exact: true });
      await FullReply.waitFor();
      Assert.ok((await FullReply.innerText()).replace(/\s+/g, " ").includes(LongReply.trim()), "The whole reply");
      Assert.ok(await FullReply.evaluate(Element => Element.scrollHeight <= Element.clientHeight + 1 && Element.getBoundingClientRect().bottom <= innerHeight), "The whole reply fits unscrolled");
      const File = `Ai-FullReply-${SizeName([FirstWidth, FirstHeight])}.png`;
      await Page.screenshot({ path: Path.join(Output, File), animations: "disabled" }); Shots.push(File);
      await Page.keyboard.press("Escape");
      await FullReply.waitFor({ state: "hidden" });
    }
    await Divider(0).press("Home");
    await Check("Ai-ChainCard-NarrowAi", CardControls, true);
    await Divider(0).press("End");
    await Check("Ai-ChainCard-WideAi", CardControls, true);
    await Page.getByRole("button", { name: "Reset column widths", exact: true }).click();
    // A proposal the IK check rejects every revision: its card shows the issues and a long reply.
    // Block sits under the live view's default spot at some sizes: the keyboard reaches it.
    await Page.getByRole("button", { name: "Block", exact: true }).focus();
    await Page.keyboard.press("Enter");
    await Page.waitForFunction(() => !document.querySelector(".ai-task-strip") && !document.querySelector("input[aria-label='AI task']").disabled);
    const OutOfReach = { response_to_user: "Moving the tip 60 cm up and 40 cm farther to look over the top shelf of the basket from above.", task_status: "continue",
      motion_proposal: { move_delta_cm: { left: 0, farther: 40, up: 60 } } };
    Replies.push(OutOfReach, OutOfReach, OutOfReach, OutOfReach, OutOfReach);
    await Submit("look over the top shelf from high above");
    await Page.locator(".ai-issue-text").first().waitFor({ timeout: 30000 });
    await Page.getByRole("button", { name: "Clear", exact: true }).waitFor();
    // The card shows its issues at the first refusal while the model is asked again (four revisions): measure it
    // once every revision was refused and Clancy stopped thinking, not while his text still changes (on a slow run
    // that moved him under the measurement, "corner at 286.98,349.24").
    for (const Started = Date.now(); Replies.length; await Page.waitForTimeout(100))
      Assert.ok(Date.now() - Started < 60000, `the app asked for every revision (${Replies.length} replies left)`);
    await Page.waitForFunction(() => document.querySelector(".ai-companion")?.dataset.state !== "thinking", null, { timeout: 60000 });
    await Page.locator(".ai-issue-text").first().waitFor();
    await Check("Ai-IssuesCard", AiControls([["Clear", "xpath=//button[normalize-space()='Clear']", 1]]));
    await Page.getByRole("button", { name: "Clear", exact: true }).focus();
    await Page.keyboard.press("Enter");
  } finally {
    await Context.close();
  }
}

try {
  for (const Group of Groups) await RunGroup(Group);
  Assert.deepEqual(Commands.filter(Command => ["send_ik", "move_servo", "go_home", "toggle_torque", "connect"].includes(Command.type)), [], "The layout checks never move the arm");
  Assert.deepEqual(Errors, []);
  if (Problems.length) throw new Error(`${Problems.length} layout problem(s):\n  ${Problems.join("\n  ")}`);
  console.log(`PASS: no page scroll, no panel scrollbar, nothing clipped and every key control in view at `
    + `${Groups.map(Group => Group.Sizes.map(Size => `${Size.join("x")}${Group.Dpr === 1 ? "" : ` (pixel ratio ${Group.Dpr}${Group.Zoom === 1 ? "" : `, ${Math.round(Group.Zoom * 100)} % zoom`})`}`).join(", ")).join(", ")}; `
    + `Manual (idle; settings drawer; selected joint's dial with a bridge update and a protection trip; narrowest and widest side columns) and AI (idle; live view expanded; banner, task strip, `
    + `estimate, chained card and long reply at the default, narrowest and widest AI column; the whole reply opened unscrolled; IK issues card); text at least ${MinimumFontPx} px `
    + `(${ResizedMinimumFontPx} px in a column resized to its minimum; CSS px times browser zoom); Clancy pixel-crisp; `
    + `${Shots.length} screenshots in ${Output}; no motion commands or browser errors.`);
} finally {
  for (const Timer of Timers) clearInterval(Timer);
  await Browser.close(); await Server.Close();
}
