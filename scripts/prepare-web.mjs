import { cp, copyFile, mkdir, rm } from "node:fs/promises";
import { resolve } from "node:path";

const vendor = resolve("docs/vendor");
await rm(vendor, { recursive: true, force: true });
await mkdir(vendor, { recursive: true });

await copyFile(
  resolve("node_modules/marked/lib/marked.umd.js"),
  resolve(vendor, "marked.umd.js")
);
await cp(
  resolve("node_modules/katex/dist"),
  resolve(vendor, "katex"),
  { recursive: true }
);

console.log("Web vendor assets prepared: Marked + KaTeX");
