import { build as Build } from 'esbuild';
import { readFileSync as ReadFile } from 'node:fs';
import { createRequire as CreateRequire } from 'node:module';
import { test as Test } from 'node:test';
import Assert from 'node:assert/strict';
import Path from 'node:path';
import { fileURLToPath as FilePath } from 'node:url';

// The app's model request, as App.tsx sends it: Ollama's native /api/chat with
// the num_ctx, num_batch and keep_alive in back end/OllamaOptions.json (the
// bridge's grounding sends the same, or Ollama reloads the model), thinking
// off, a fixed 1024-token answer budget, and each content part as its own
// message (the layout that reproduced the /v1 token counts). Also the page's
// warm-up request. Nothing here reaches Ollama: fetch is replaced.
const Root = Path.dirname(FilePath(import.meta.url));
const Options = JSON.parse(ReadFile(Path.join(Root, '../back end/OllamaOptions.json'), 'utf8'));
const Load = async (Contents, ResolveDir, Loader = 'ts') => {
  const Bundle = await Build({ stdin: { contents: Contents, resolveDir: ResolveDir, loader: Loader }, bundle: true, platform: 'node', format: 'cjs',
    write: false, loader: { '.css': 'empty' } });
  const Module = { exports: {} };
  new Function('require', 'module', 'exports', Bundle.outputFiles[0].text)(CreateRequire(import.meta.url), Module, Module.exports);
  return Module.exports;
};
globalThis.window ??= { location: { hostname: 'localhost' } };
const App = await Load(ReadFile(Path.join(Root, 'src/App.tsx'), 'utf8') + '\nexport { requestAiCompletion as RequestAiCompletion };', Path.join(Root, 'src'), 'tsx');

const Jpeg = Buffer.from([0xff, 0xd8, 0xff, 0xe0, 1, 2, 3]).toString('base64'), Png = Buffer.from([0x89, 0x50, 0x4e, 0x47, 4, 5, 6]).toString('base64');
// A check-in as runAiRequest builds it: system prompt, one earlier turn, then
// the prompt text with the colour frame and the depth colormap.
const Messages = [
  { role: 'system', content: 'You are controlling an SO-101 robotic arm.' },
  { role: 'user', content: 'Grab the eraser' },
  { role: 'assistant', content: '{"response_to_user":"Hovering first."}' },
  { role: 'user', content: [{ type: 'text', text: 'AUTO CHECK-IN: where is the tip?' },
    { type: 'image_url', image_url: { url: `data:image/jpeg;base64,${Jpeg}` } },
    { type: 'image_url', image_url: { url: `data:image/png;base64,${Png}` } }] },
];
const NativeReply = (Fields = {}) => ({ model: 'qwen', created_at: '2026-09-29T15:00:00Z', message: { role: 'assistant', content: '{"response_to_user":"ok"}' },
  done: true, done_reason: 'stop', prompt_eval_count: 5354, eval_count: 88, load_duration: 21000000, ...Fields });
async function Ask(Reply, MaxTokens = 253952) {
  const Calls = [], Usage = [];
  globalThis.fetch = async (Url, Init) => {
    Calls.push({ Url: String(Url), Body: JSON.parse(Init.body) });
    return { ok: true, json: async () => Reply, text: async () => JSON.stringify(Reply) };
  };
  let Result, Failure;
  try { Result = await App.RequestAiCompletion(Messages, 'qwen', MaxTokens, Item => Usage.push(Item)); } catch (Error_) { Failure = Error_; }
  return { Calls, Usage, Result, Failure };
}

Test('the model request goes to native /api/chat with the shared options, thinking off and a 1024-token budget', async () => {
  const { Calls, Result, Failure } = await Ask(NativeReply());
  Assert.equal(Calls.length, 1);
  const { Url, Body } = Calls[0];
  Assert.equal(Url, '/ollama/api/chat');
  Assert.equal(Body.model, 'qwen');
  Assert.equal(Body.stream, false);
  Assert.equal(Body.think, false, 'without think:false the model reasoned for 258 tokens first');
  Assert.equal(Body.keep_alive, Options.keep_alive);
  Assert.deepEqual(Body.options, { num_ctx: Options.num_ctx, num_batch: Options.num_batch, temperature: 0.2, num_predict: 1024 },
    'a budget of 253952 tokens (the old max_tokens at 262k) is capped at 1024');
  for (const Key of ['max_tokens', 'reasoning_effort', 'temperature']) Assert.ok(!(Key in Body), `${Key} is a /v1 field`);
  Assert.equal(Failure, undefined);
  Assert.equal(Result, '{"response_to_user":"ok"}');
});

Test('each content part is its own message: the text, then each image as bare base64', async () => {
  const { Calls } = await Ask(NativeReply());
  Assert.deepEqual(Calls[0]?.Body.messages, [
    { role: 'system', content: 'You are controlling an SO-101 robotic arm.' },
    { role: 'user', content: 'Grab the eraser' },
    { role: 'assistant', content: '{"response_to_user":"Hovering first."}' },
    { role: 'user', content: 'AUTO CHECK-IN: where is the tip?' },
    { role: 'user', content: '', images: [Jpeg] },
    { role: 'user', content: '', images: [Png] },
  ]);
});

Test('a smaller budget is kept; Ollama\'s token counts reach the statistics', async () => {
  const { Calls, Usage } = await Ask(NativeReply(), 300);
  Assert.equal(Calls[0]?.Body.options?.num_predict, 300);
  Assert.equal(Usage.length, 1);
  Assert.deepEqual([Usage[0].InputTokens, Usage[0].OutputTokens, Usage[0].TotalTokens], [5354, 88, 5442]);
});

Test('a reply cut off by the budget, an empty one and an unreadable one are errors, not replies', async () => {
  const Cut = await Ask(NativeReply({ done_reason: 'length', message: { role: 'assistant', content: '{"response_to_user":"Hover' } }));
  Assert.match(String(Cut.Failure?.message), /cut off at the 1024-token limit/);
  const ShortCut = await Ask(NativeReply({ done_reason: 'length' }), 512);
  Assert.match(String(ShortCut.Failure?.message), /cut off at the 512-token limit/);
  const Empty = await Ask(NativeReply({ message: { role: 'assistant', content: '  ' } }));
  Assert.match(String(Empty.Failure?.message), /empty text content \(stop\)/);
  const Unreadable = await Ask({ choices: [{ message: { content: 'a /v1 reply' } }] });
  Assert.match(String(Unreadable.Failure?.message), /unreadable response/);
});

Test('the warm-up request loads the model with the same num_ctx and num_batch and asks nothing', async () => {
  const Chat = await Load("export * from './OllamaChat';", Path.join(Root, 'src/lib'));
  Assert.deepEqual(Chat.BuildOllamaPreloadBody('qwen'), { model: 'qwen', messages: [], stream: false, keep_alive: Options.keep_alive,
    options: { num_ctx: Options.num_ctx, num_batch: Options.num_batch } });
  Assert.equal(Chat.OllamaChatUrl, '/ollama/api/chat');
  // When the page warms the model again: shortly before keep_alive runs out.
  Assert.equal(Chat.KeepAliveMs('30m'), 30 * 60_000);
  Assert.equal(Chat.KeepAliveMs('90s'), 90_000);
  Assert.equal(Chat.KeepAliveMs('1h'), 3_600_000);
  Assert.equal(Chat.KeepAliveMs(-1), Infinity);
  Assert.equal(Chat.KeepAliveMs('soon'), 5 * 60_000, "Ollama's own default");
  Assert.equal(Chat.RewarmAfterMs('30m'), 25 * 60_000);
  Assert.equal(Chat.RewarmAfterMs('2m'), 60_000);
});
