// Declared surfaces (back end/DeclaredSurfaces.json): the app's reader (lib/DeclaredSurfaces.ts) keeps
// and refuses exactly what the bridge's (DeclaredSurfaces.py) does, case by case, with the same reasons.
import { build as Build } from 'esbuild';
import { createRequire as CreateRequire } from 'node:module';
import { execFileSync as Execute } from 'node:child_process';
import { test as Test } from 'node:test';
import Assert from 'node:assert/strict';
import Fs from 'node:fs';
import Path from 'node:path';
import { fileURLToPath as FilePath } from 'node:url';

const Root = Path.dirname(FilePath(import.meta.url));
const Backend = Path.join(Root, '../back end');
const Bundle = await Build({ stdin: { contents: `export * from './DeclaredSurfaces';`, resolveDir: Path.join(Root, 'src/lib'), loader: 'ts' },
  bundle: true, platform: 'node', format: 'cjs', write: false });
const Module = { exports: {} };
new Function('require', 'module', 'exports', Bundle.outputFiles[0].text)(CreateRequire(import.meta.url), Module, Module.exports);
const Api = Module.exports;

const Platform = { Name: 'box platform', Kind: 'platform', TopZCm: 12.7, XRangeCm: [-25, -3], YRangeCm: [30, 44] };
const Cases = [
  { Surfaces: [] },
  {},
  { Surfaces: null },
  [],
  { Surfaces: [Platform] },
  { Surfaces: [Platform, { ...Platform, Name: 'low shelf', Kind: 'shelf', TopZCm: -8, BoardAboveZCm: 6, Frame: 'cart' }] },
  { Surfaces: [{ ...Platform, Enabled: false }, { ...Platform, Name: '  spaced   name ' }] },
  { Surfaces: [Platform, { ...Platform, Name: 'Box  Platform' }] },
  ...[{ TopZCm: -2 }, { TopZCm: 50 }, { TopZCm: '12' }, { TopZCm: null }, { Kind: 'table' }, { XRangeCm: [-3, -25] }, { YRangeCm: [30] },
    { YRangeCm: [30, 200] }, { XRangeCm: [-1, 0.5] }, { Kind: 'shelf' }, { Kind: 'shelf', BoardAboveZCm: 15 }, { BoardAboveZCm: 30 },
    { Frame: 'world' }, { Frame: null }, { Name: '  ' }, { Name: 'x'.repeat(41) }, { Enabled: 'yes' }, { Enabled: null }]
    .map(Change => ({ Surfaces: [{ ...Platform, Name: 'kept' }, { ...Platform, ...Change }] })),
  { Surfaces: ['not an object', Platform] },
];

Test('the file reads without a problem (the committed one declares nothing), and its example is a platform 15 cm above the desk', () => {
  Assert.deepEqual(Api.DeclaredSurfaceProblems, []);
  const Spec = JSON.parse(Fs.readFileSync(Path.join(Backend, 'DeclaredSurfaces.json'), 'utf8'));
  const { Surfaces, Problems } = Api.ValidateSurfaces({ Surfaces: [Spec.Example] });
  Assert.deepEqual(Problems, []);
  Assert.equal(Surfaces[0].Kind, 'platform');
  Assert.match(Api.SurfaceText(Surfaces[0]), /^the platform "box platform" \(Z 12\.7, 15\.0 cm above the desk\)$/);
});

Test('a broken entry is left out with its reason and a valid one beside it stays', () => {
  const { Surfaces, Problems } = Api.ValidateSurfaces({ Surfaces: [{ ...Platform, Name: 'kept' }, { ...Platform, TopZCm: -2 }] });
  Assert.deepEqual(Surfaces.map(Surface => Surface.Name), ['kept']);
  Assert.deepEqual(Problems, ['surface 2 (box platform): TopZCm -2 is within 1.5 cm of the desk (Z -2.3): that is the desk']);
  Assert.ok(Api.SurfaceContains(Surfaces[0], { x: -10, y: 35 }));
  Assert.ok(!Api.SurfaceContains(Surfaces[0], { x: -2, y: 35 }));
  Assert.ok(Api.SurfaceContains(Surfaces[0], { x: -2, y: 35 }, 1.5));
});

Test('the app and the bridge read every case alike', () => {
  const Scratch = Path.join(Backend, 'TestArtifacts', 'DeclaredSurfaces');
  Fs.mkdirSync(Scratch, { recursive: true });
  const Files = Cases.map((Case, Index) => {
    const File = Path.join(Scratch, `Parity${Index}.json`);
    Fs.writeFileSync(File, JSON.stringify(Case));
    return File;
  });
  const Python = Fs.readFileSync(Path.join(Root, '../.backend-python.txt'), 'utf8').replace(/^﻿/, '').trim();
  const Bridge = JSON.parse(Execute(Python, ['DeclaredSurfaces.py', ...Files], { cwd: Backend, encoding: 'utf8' }));
  Assert.equal(Bridge.length, Cases.length);
  Cases.forEach((Case, Index) => Assert.deepEqual(Api.ValidateSurfaces(Case), Bridge[Index], `case ${Index}: ${JSON.stringify(Case)}`));
  // Every refusal case was refused (and its valid neighbour kept) on both sides.
  Assert.ok(Bridge.filter(Result => Result.Problems.length).length >= 20);
});

Test('the page says the declared surfaces\' problems once at load: the bridge\'s (GET /surfaces) and its own, each once', () => {
  Assert.equal(Api.SurfaceProblemsNote({ surfaces: [], problems: [], table_z_cm: -2.0 }, []), null);
  Assert.equal(Api.SurfaceProblemsNote(null, []), null, 'a bridge that did not answer, and a page without problems');
  Assert.equal(Api.SurfaceProblemsNote({ problems: 'not a list' }, []), null);
  const One = 'surface 2 (cabinet top): TopZCm 0.5 is within 1.5 cm of the desk (Z -2): that is the desk';
  Assert.equal(Api.SurfaceProblemsNote({ problems: [One] }, [One]),
    `back end/DeclaredSurfaces.json has a problem, and those surfaces are left out: ${One}. Fix the file, then reload the page.`);
  const Page = 'surface 3 (shelf): a shelf needs BoardAboveZCm, the underside of the board above (cm, display Z)';
  Assert.equal(Api.SurfaceProblemsNote({ problems: [One, '  '] }, [Page]),
    `back end/DeclaredSurfaces.json has 2 problems, and those surfaces are left out: ${One}; ${Page}. Fix the file, then reload the page.`);
  // With the file as committed the page has none of its own.
  Assert.equal(Api.SurfaceProblemsNote({ problems: [] }), null);
});
