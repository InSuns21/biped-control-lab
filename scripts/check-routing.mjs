import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import { chapters, resolveContentReference } from "../docs/js/routes.js";

const siteBase = "https://insuns21.github.io/biped-control-lab/";
let markdownChapterLinks = 0;

for (const [id, , filename] of chapters) {
  const baseUrl = new URL(`theory/${filename}`, siteBase);

  assert.equal(
    resolveContentReference(filename, baseUrl),
    `#${id}`,
    `${filename} should route to #${id}`
  );

  const markdown = await readFile(`docs/theory/${filename}`, "utf8");
  for (const match of markdown.matchAll(/(?<!!)\[[^\]]*\]\(([^)]+)\)/g)) {
    const ref = match[1].trim();
    if (!/\.md(?:#.*)?$/i.test(ref)) continue;
    markdownChapterLinks++;
    assert.match(
      resolveContentReference(ref, baseUrl),
      /^#\d{2}$/,
      `${filename}: ${ref} should stay inside the theory SPA`
    );
  }
}

assert.ok(markdownChapterLinks > 0, "No chapter-to-chapter Markdown links were found");
assert.equal(
  resolveContentReference(
    "../labs/02-pid-balance/",
    new URL("theory/03_pid_feedback.md", siteBase)
  ),
  `${siteBase}labs/02-pid-balance/`
);
assert.equal(
  resolveContentReference("https://example.com/reference", new URL(siteBase)),
  "https://example.com/reference"
);

console.log(`Theory routing OK: ${markdownChapterLinks} Markdown chapter links stay in SPA routes`);
