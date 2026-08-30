import assert from "node:assert/strict";
import { readdir, readFile } from "node:fs/promises";

const diagramDir = "docs/assets/diagrams";
const files = (await readdir(diagramDir)).filter((name) => name.endsWith(".svg")).sort();
let markerCount = 0;
let referenceCount = 0;

for (const file of files) {
  const path = `${diagramDir}/${file}`;
  const svg = await readFile(path, "utf8");

  assert.match(svg, /<svg\b[^>]*\bviewBox="[^"]+"/i, `${file}: root <svg> must define viewBox`);

  const markerIds = new Set();
  for (const match of svg.matchAll(/<marker\b([^>]*)>/gi)) {
    markerCount++;
    const attrs = match[1];
    const id = attrs.match(/\bid="([^"]+)"/i)?.[1];
    assert.ok(id, `${file}: every marker must have an id`);
    markerIds.add(id);

    assert.match(
      attrs,
      /\bmarkerUnits="userSpaceOnUse"/i,
      `${file}: marker #${id} must use markerUnits="userSpaceOnUse" so arrowheads do not scale with stroke width`
    );
    assert.match(
      attrs,
      /\bviewBox="[^"]+"/i,
      `${file}: marker #${id} must define its own viewBox`
    );
  }

  for (const match of svg.matchAll(/marker-(?:start|mid|end)="url\(#([^)]+)\)"/gi)) {
    referenceCount++;
    const id = match[1];
    assert.ok(markerIds.has(id), `${file}: marker reference #${id} has no matching <marker>`);
  }
}

assert.ok(files.length >= 1, "No SVG diagrams found");
console.log(`SVG diagrams OK: ${files.length} files, ${markerCount} markers, ${referenceCount} marker references`);
