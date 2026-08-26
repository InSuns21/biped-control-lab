import { readdir, readFile } from "node:fs/promises";
import { join } from "node:path";
import katex from "katex";

async function walk(dir) {
  const out = [];
  for (const entry of await readdir(dir, { withFileTypes: true })) {
    const path = join(dir, entry.name);
    if (entry.isDirectory()) out.push(...await walk(path));
    else if (entry.isFile() && path.endsWith(".md")) out.push(path);
  }
  return out;
}

function stripCode(text) {
  return text.replace(/```[\s\S]*?```/g, "").replace(/`[^`\n]*`/g, "");
}

function formulas(text) {
  const cleaned = stripCode(text);
  const found = [];
  const display = /\$\$([\s\S]*?)\$\$/g;
  let match;
  while ((match = display.exec(cleaned))) found.push(match[1]);
  const withoutDisplay = cleaned.replace(display, "");
  const inline = /(?<!\\)\$(?!\$)([^\n$]+?)(?<!\\)\$/g;
  while ((match = inline.exec(withoutDisplay))) found.push(match[1]);
  return found;
}

let errors = 0, count = 0;
for (const file of await walk("docs")) {
  const text = await readFile(file, "utf8");
  for (const formula of formulas(text)) {
    count++;
    try {
      katex.renderToString(formula, { throwOnError: true, strict: "error" });
    } catch (error) {
      errors++;
      console.error(`KaTeX error in ${file}: ${formula.trim()}\n${error.message}`);
    }
  }
}
if (errors) process.exit(1);
console.log(`KaTeX syntax OK: ${count} formulas`);
