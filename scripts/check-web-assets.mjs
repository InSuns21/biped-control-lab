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

console.log(`Web runtime assets OK: Marked parse + KaTeX assets + ${chapters.length} theory routes`);
