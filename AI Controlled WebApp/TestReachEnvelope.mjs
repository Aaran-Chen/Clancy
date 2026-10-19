// THE REACH ENVELOPE with the cart parked: at each tip height Z and wrist pitch, how far from the right
// arm's pan axis the tip (the fixed jaw's inner pad) reaches, with the app's own IK and joint limits
// (Kinematic) and with its CAD basket, cart and body contact too (Collision: what the app accepts).
// The grid is cached in src/lib/ReachEnvelope.json; this test computes it again and fails on any change,
// so a change to the IK, the limits or the collision model shows up as a changed envelope.
// The desk floor, the rest of the gripper and any shelf are not part of it.
//   node TestReachEnvelope.mjs           check against the cached grid
//   node TestReachEnvelope.mjs --write   compute the grid and save it (after a deliberate model change)
import { build as Build } from 'esbuild';
import { createRequire as CreateRequire } from 'node:module';
import { test as Test } from 'node:test';
import Assert from 'node:assert/strict';
import Fs from 'node:fs';
import Path from 'node:path';
import { fileURLToPath as FilePath } from 'node:url';

const Root = Path.dirname(FilePath(import.meta.url));
const CachePath = Path.join(Root, 'src/lib/ReachEnvelope.json');
const Bundle = await Build({ stdin: { contents: `export * from './CadKinematics'; export * from './robotModel';`, resolveDir: Path.join(Root, 'src/lib'), loader: 'ts' },
  bundle: true, platform: 'node', format: 'cjs', write: false });
const Module = { exports: {} };
new Function('require', 'module', 'exports', Bundle.outputFiles[0].text)(CreateRequire(import.meta.url), Module, Module.exports);
const Api = Module.exports;

// The grid: every 5 cm of tip height, 1 cm of radius, five wrist pitches (0 level, -90 jaw straight
// down, +30 tilted up), straight ahead (azimuth 0) and swung 50 deg to the arm's left for the low rows.
const Grid = { Pitches: [30, 0, -30, -60, -90], ZCm: Array.from({ length: 13 }, (_, Index) => -15 + 5 * Index), RMaxCm: 52,
  Swung: { AzimuthDeg: 50, ZCm: [-10, -5, 0, 5, 10] } };

/** Runs of whole-cm radii as [from, to] pairs: [27, 30], [32, 35] for 27..30 and 32..35. */
function Runs(Radii) {
  const Out = [];
  for (const R of Radii) {
    const Last = Out[Out.length - 1];
    if (Last && R === Last[1] + 1) Last[1] = R; else Out.push([R, R]);
  }
  return Out;
}

/** The envelope at one azimuth (deg from +Y toward the arm's left), as Reach.cjs of 2026-10-05 measured it. */
function Envelope(AzimuthDeg, Zs) {
  const Pan = Api.mirrorDisplayPoint({ x: 0, y: 0, z: 0 });
  const Limits = Api.GetIkAngleLimits(Api.INITIAL_SERVOS), Home = Api.currentAngles(Api.INITIAL_SERVOS);
  const Kinematic = {}, Collision = {};
  for (const Pitch of Grid.Pitches) {
    Kinematic[Pitch] = {}; Collision[Pitch] = {};
    for (const Z of Zs) {
      const Reached = [], Accepted = [];
      for (let R = 0; R <= Grid.RMaxCm; R++) {
        const Display = { x: Pan.x + R * Math.sin(AzimuthDeg * Math.PI / 180), y: Pan.y + R * Math.cos(AzimuthDeg * Math.PI / 180), z: Z };
        const Target = Api.modelPointFromDisplayPoint(Display);
        if (!Api.CadSolve(Target, Pitch, 'up', Home, Limits, false).Reachable) continue;
        Reached.push(R);
        // Either elbow branch, clear of the basket, cart and body, as the app's preview accepts it.
        if (['up', 'down'].some(Elbow => Api.solveIkPreview({ ...Target, wristPitch: Pitch }, Elbow).reachable)) Accepted.push(R);
      }
      Kinematic[Pitch][Z] = Runs(Reached); Collision[Pitch][Z] = Runs(Accepted);
    }
  }
  return { Kinematic, Collision };
}

function Compute() {
  const Pan = Api.mirrorDisplayPoint({ x: 0, y: 0, z: 0 });
  return {
    Version: 1,
    About: 'The right arm\'s tip reach with the cart parked (TestReachEnvelope.mjs): for each wrist pitch (deg, 0 level, -90 jaw down) and tip height Z (display, cm; the desk is -2.3), the runs [from, to] of whole-cm radii from the pan axis the IK reaches. Kinematic: joint limits only; Collision: also clear of the CAD basket, cart and body. The desk floor, the rest of the gripper and any shelf are not part of it.',
    PanAxisCm: { x: Number(Pan.x.toFixed(3)), y: Number(Pan.y.toFixed(3)) },
    Pitches: Grid.Pitches, ZCm: Grid.ZCm, RMaxCm: Grid.RMaxCm,
    Ahead: { AzimuthDeg: 0, ...Envelope(0, Grid.ZCm) },
    Swung: { AzimuthDeg: Grid.Swung.AzimuthDeg, ZCm: Grid.Swung.ZCm, Collision: Envelope(Grid.Swung.AzimuthDeg, Grid.Swung.ZCm).Collision },
  };
}

const Started = Date.now();
const Fresh = Compute();
const Seconds = (Date.now() - Started) / 1000;
if (process.argv.includes('--write')) {
  Fs.writeFileSync(CachePath, JSON.stringify(Fresh, null, 1) + '\n');
  console.log(`wrote ${CachePath} in ${Seconds.toFixed(1)} s`);
  process.exit(0);
}
const Cached = JSON.parse(Fs.readFileSync(CachePath, 'utf8'));
const Span = (Rows, Pitch, Z) => { const Row = Rows[Pitch][Z]; return Row.length ? [Row[0][0], Row[Row.length - 1][1]] : null; };
const Ahead = Fresh.Ahead.Collision;

Test('the envelope the app\'s IK and collision model give matches the cached grid', () => {
  Assert.ok(Seconds < 120, `${Seconds.toFixed(1)} s`);
  Assert.deepEqual(Fresh, Cached, 'the reach changed: if the change is meant, run node TestReachEnvelope.mjs --write and review the diff');
});

Test('at desk height the arm reaches over the basket only with the wrist pitched down 30 deg or more', () => {
  // Spot checks of 2026-10-05: at Y 33 a tip at Z 0 is refused at a near-level wrist (the upper arm or the wrist link
  // meets the body or the basket) and works at -30; the level wrist starts at Z 5.
  Assert.deepEqual(Ahead[0][0], []);
  Assert.deepEqual(Ahead[0][-5], []);
  Assert.ok(Span(Ahead, -30, 0)[0] <= 33 && Span(Ahead, -30, 0)[1] >= 33);
  Assert.deepEqual(Span(Ahead, 0, 5), [25, 43]);
});

Test('a platform 7 to 17 cm above the desk is reached at a level wrist across the whole working band', () => {
  for (const Z of [5, 10, 15]) {
    const [From, To] = Span(Ahead, 0, Z);
    Assert.ok(From <= 25 && To >= 43, `Z ${Z}: ${From}..${To}`);
  }
  // A steep wrist reaches it only close in: -60 at Z 15 is 15..27 cm, none at Z 20.
  Assert.deepEqual(Span(Ahead, -60, 15), [15, 27]);
  Assert.equal(Span(Ahead, -60, 20), null);
});

Test('low surfaces end about 10 cm under the desk, and only with a steep wrist', () => {
  for (const Pitch of Fresh.Pitches) Assert.equal(Span(Ahead, Pitch, -15), null, `pitch ${Pitch} at Z -15`);
  Assert.deepEqual(Span(Ahead, -60, -10), [31, 35]);
  Assert.deepEqual(Span(Ahead, -90, -10), [27, 27]);
  for (const Pitch of [30, 0, -30]) Assert.equal(Span(Ahead, Pitch, -10), null, `pitch ${Pitch} at Z -10`);
  // Kinematically the arm goes lower; the basket's front rim is what stops it.
  Assert.ok(Span(Fresh.Ahead.Kinematic, -90, -15));
  // Swung 50 deg to the side, nothing low opens up.
  for (const Pitch of Fresh.Pitches) Assert.equal(Span(Fresh.Swung.Collision, Pitch, -10), null, `swung, pitch ${Pitch} at Z -10`);
});

Test('high shelves: a level wrist to Z 35, a wrist tilted up 30 deg to Z 40, nothing at 45', () => {
  Assert.deepEqual(Span(Ahead, 0, 35), [15, 23]);
  Assert.equal(Span(Ahead, 0, 40), null);
  Assert.deepEqual(Span(Ahead, 30, 40), [5, 29]);
  for (const Pitch of Fresh.Pitches) Assert.equal(Span(Ahead, Pitch, 45), null, `pitch ${Pitch} at Z 45`);
});
