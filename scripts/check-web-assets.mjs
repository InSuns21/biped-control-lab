import { readFile, stat } from "node:fs/promises";
import vm from "node:vm";
import { chapters } from "../docs/js/routes.js";

async function requireFile(path) {
  const info = await stat(path);
  if (!info.isFile() || info.size === 0) throw new Error(`Missing or empty web asset: ${path}`);
}

await requireFile("docs/vendor/marked.umd.js");
await requireFile("docs/vendor/katex/katex.min.js");
await requireFile("docs/vendor/katex/contrib/auto-render.min.js");
await requireFile("docs/vendor/katex/katex.min.css");
await requireFile("docs/vendor/three.core.js");
await requireFile("docs/vendor/three.module.js");
await requireFile("docs/labs/x1-shower-tvc/index.html");
await requireFile("docs/labs/x1-shower-tvc/main.js");
await requireFile("docs/labs/x1-shower-tvc/scene.js");
await requireFile("docs/labs/x1-shower-tvc/phase0.js");
await requireFile("docs/labs/x1-shower-tvc/flexible-ui.js");
await requireFile("docs/labs/x1-shower-tvc/flexible-view.js");
await requireFile("docs/labs/x1-shower-tvc/nonlinear-ui.js");
await requireFile("docs/labs/x1-shower-tvc/nonlinear-view.js");
await requireFile("docs/js/shower/flexible/scenarios.js");
await requireFile("docs/js/shower/flexible/boundary.js");
await requireFile("docs/js/shower/flexible/calibration.js");
await requireFile("docs/js/shower/flexible/nonlinear-rod.js");
await requireFile("docs/js/shower/flexible/nonlinear-flow.js");
await requireFile("docs/js/shower/flexible/nonlinear-shower-head.js");
await requireFile("docs/js/shower/flexible/nonlinear-scenario.js");
for (const [, , filename] of chapters) await requireFile(`docs/theory/${filename}`);

const code = await readFile("docs/vendor/marked.umd.js", "utf8");
const sandbox = { console };
sandbox.globalThis = sandbox;
vm.createContext(sandbox);
vm.runInContext(code, sandbox, { filename: "marked.umd.js" });

if (!sandbox.marked || typeof sandbox.marked.parse !== "function") {
  throw new Error("Vendored Marked does not expose marked.parse()");
}
const rendered = sandbox.marked.parse("# Runtime smoke test");
if (!String(rendered).includes("<h1>Runtime smoke test</h1>")) {
  throw new Error("Vendored Marked failed to render Markdown");
}

const threeModule = await readFile("docs/vendor/three.module.js", "utf8");
if (threeModule.includes("./three.core.js")) {
  await requireFile("docs/vendor/three.core.js");
}

console.log(`Web runtime assets OK: Marked + KaTeX + Three.js + Shower Phase 0/1 assets + ${chapters.length} theory routes`);
