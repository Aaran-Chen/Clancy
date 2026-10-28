import { build as Build } from 'esbuild';
import { execFileSync as ExecFile } from 'node:child_process';
import { existsSync as Exists, mkdirSync as MakeDir, mkdtempSync as MakeTemp, readFileSync as ReadFile, rmSync as Remove, writeFileSync as WriteFile } from 'node:fs';
import { createRequire as CreateRequire } from 'node:module';
import { tmpdir as TempDir } from 'node:os';
import { test as Test } from 'node:test';
import Assert from 'node:assert/strict';
import Path from 'node:path';
import { fileURLToPath as FilePath } from 'node:url';
import { RecordedCaptures } from './UiTestLib.mjs';

// Every recorded /locate reply of 2026-10-03, 10-04 and 10-06 (grounding.jsonl), through the camera gate as
// 74097be had it and as it is now: the same Code, Title and Detail, and the same desk note. The no-desk check
// (back end/RobotReference.py) adds fields and a note for replies that carry robot_check, and none of these
// does. A camera_check_failed is titled by the bridge's sentence now (audit 2 M6). The recordings are found as back end/RecordedCaptures.py finds them (SO101_REAL_CAPTURES; in a git worktree
// the main checkout's); without them, or without git, the test fails (RECORDING MISSING): it used to skip.
const Root = Path.dirname(FilePath(import.meta.url));
const Repo = Path.join(Root, '..');

const Captures = RecordedCaptures();
const Days = ['2026-10-03', '2026-10-04', '2026-10-06'];
const Base = '74097be';

async function Load(LibDir) {
  // Packages (three) come from this app's node_modules wherever the lib sits.
  const Output = await Build({ stdin: { contents: `export * from './AiMeasurements';`, resolveDir: LibDir, loader: 'ts' },
    bundle: true, platform: 'node', format: 'cjs', write: false, loader: { '.json': 'json' }, nodePaths: [Path.join(Root, 'node_modules')] });
  const Module = { exports: {} };
  new Function('require', 'module', 'exports', Output.outputFiles[0].text)(CreateRequire(import.meta.url), Module, Module.exports);
  return Module.exports;
}

// The app's lib at Base and the back end JSON it imports, laid out as in the checkout, in a temporary folder.
function CheckoutAt(Commit) {
  const Git = (...Args) => ExecFile('git', ['-C', Repo, ...Args], { maxBuffer: 256 << 20 });
  const Folder = MakeTemp(Path.join(TempDir(), 'CameraGateReplay-'));
  const Files = [...Git('ls-tree', '-r', '--name-only', Commit, 'AI Controlled WebApp/src/lib').toString().trim().split('\n'),
    ...Git('ls-tree', '--name-only', Commit, 'back end/').toString().trim().split('\n').filter(Name => Name.endsWith('.json'))];
  for (const File of Files) {
    const Target = Path.join(Folder, File);
    MakeDir(Path.dirname(Target), { recursive: true });
    WriteFile(Target, Git('show', `${Commit}:${File}`));
  }
  return Folder;
}

const Absent = Days.map(Day => Path.join(Captures, Day, 'recording', 'grounding.jsonl')).filter(File => !Exists(File));
const Records = Days.flatMap(Day => {
  const File = Path.join(Captures, Day, 'recording', 'grounding.jsonl');
  return Exists(File) ? ReadFile(File, 'utf8').trim().split('\n').map(Line => JSON.parse(Line)).filter(Row => Row.kind === 'locate') : [];
});
let Old = null;
let Folder = null;
let Unavailable = Absent.length ? `RECORDING MISSING: the recorded /locate replies (grounding.jsonl): ${Absent.join(', ')} not on this machine. `
  + 'Restore them (SO101_REAL_CAPTURES names another captures folder).' : null;
try {
  if (!Unavailable) {
    Folder = CheckoutAt(Base);
    Old = await Load(Path.join(Folder, 'AI Controlled WebApp', 'src', 'lib'));
  }
} catch (Problem) {
  Unavailable = `the camera gate at ${Base} could not be loaded (this replay needs git and that commit of this repo): ${Problem.message}`;
}
const New = await Load(Path.join(Root, 'src', 'lib'));

Test('every recorded camera check reaches the same gate and note as before the no-desk check', () => {
  Assert.equal(Unavailable, null);
  let Compared = 0;
  const Codes = {};
  try {
    for (const Record of Records) {
      // What the bridge returned to the app (the record adds its own fields beside these).
      const Reply = { ok: Record.ok, error_code: Record.error_code, error: Record.error, camera_check: Record.camera_check,
        ...(Record.ok ? { objects: Record.objects ?? [], withheld: Record.withheld ?? [], rms_cm: Record.rms_cm ?? null } : {}) };
      const Before = Old.ReadLocateResponse(Reply), After = New.ReadLocateResponse(Reply);
      for (const Needs of [true, false]) {
        for (const Source of ['depth', 'webcam']) {
          const Expected = Old.CameraGateFailure(Before, Needs, Source);
          // Since audit 2 M6 a failed check is titled by the bridge's sentence (74097be said "the desk was not measured" for all).
          if (Expected?.Code === 'camera_check_failed' && After.error) Object.assign(Expected, { Title: After.error, Detail: After.cameraCheck?.Reason ?? '' });
          Assert.deepEqual(New.CameraGateFailure(After, Needs, Source), Expected, `${Record.time} needs=${Needs} ${Source}`);
        }
      }
      Assert.equal(New.DeskRemeasuredNote(After?.cameraCheck ?? null), Old.DeskRemeasuredNote(Before?.cameraCheck ?? null));
      Assert.equal(New.RobotCheckNote(After?.cameraCheck ?? null), null, 'no recorded reply came from the robot path');
      const Gate = New.CameraGateFailure(After, true);
      Codes[Gate?.Code ?? 'passed'] = (Codes[Gate?.Code ?? 'passed'] ?? 0) + 1;
      Compared += 1;
    }
  } finally {
    if (Folder) Remove(Folder, { recursive: true, force: true });
  }
  Assert.ok(Compared > 0);
  console.log(`  ${Compared} recorded replies, the same gate before and after: ${JSON.stringify(Codes)}`);
});
