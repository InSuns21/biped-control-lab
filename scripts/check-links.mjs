import { readdir, readFile, stat } from "node:fs/promises";
import { dirname, join, normalize, resolve } from "node:path";

const root = resolve(".");
async function walk(dir) {
  const out = [];
  for (const entry of await readdir(dir, { withFileTypes: true })) {
    if (["node_modules", ".git", "vendor"].includes(entry.name)) continue;
    const path = join(dir, entry.name);
    if (entry.isDirectory()) out.push(...await walk(path));
    else if (entry.isFile() && (path.endsWith(".md") || path.endsWith(".html"))) out.push(path);
  }
  return out;
}
async function exists(path) { try { await stat(path); return true; } catch { return false; } }
function refs(text, ext) {
  const out = [];
  if (ext === ".md") {
    for (const m of text.matchAll(/\[[^\]]*\]\(([^)]+)\)/g)) out.push(m[1].trim());
  } else {
    for (const m of text.matchAll(/(?:href|src)=["']([^"']+)["']/g)) out.push(m[1].trim());
  }
  return out;
}
function shouldSkip(ref) { return !ref || ref.startsWith("#") || /^(https?:|mailto:|data:|javascript:)/i.test(ref); }
let errors = 0, checked = 0;
for (const file of await walk(".")) {
  const text = await readFile(file, "utf8");
  const ext = file.endsWith(".md") ? ".md" : ".html";
  for (let ref of refs(text, ext)) {
    if (shouldSkip(ref)) continue;
    ref = ref.split("#")[0].split("?")[0];
    if (!ref) continue;
    const target = normalize(resolve(dirname(file), ref));
    if (!target.startsWith(root)) { console.error(`Link escapes repository: ${file} -> ${ref}`); errors++; continue; }
    checked++;
    let ok = await exists(target);
    if (ok && (await stat(target)).isDirectory()) ok = await exists(join(target, "index.html")) || await exists(join(target, "README.md"));
    if (!ok) { console.error(`Broken local link: ${file} -> ${ref}`); errors++; }
  }
}
if (errors) process.exit(1);
console.log(`Local links OK: ${checked} references`);
