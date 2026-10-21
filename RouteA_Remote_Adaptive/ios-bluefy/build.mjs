import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
const root = path.dirname(fileURLToPath(import.meta.url));
const template = fs.readFileSync(path.join(root, "template.html"), "utf8");
const css = fs.readFileSync(path.join(root, "src", "styles.css"), "utf8");
const core = fs.readFileSync(path.join(root, "src", "core.js"), "utf8");
const sequence = fs.readFileSync(path.join(root, "src", "sequence.js"), "utf8");
const app = fs.readFileSync(path.join(root, "src", "app.js"), "utf8");
const out = template
  .replace("/*__CSS__*/", () => css)
  .replace("/*__CORE__*/", () => core + "\n" + sequence)
  .replace("/*__APP__*/", () => app);
if (out.includes("/*__CSS__*/") || out.includes("/*__CORE__*/") || out.includes("/*__APP__*/")) throw new Error("build placeholder remains");
if (!out.includes("P3,BSEQ") || !out.includes("开始自动LLM调参")) throw new Error("built artifact lost v3 protocol or auto UI");
for (const name of ["routea_remote_adaptive_bluefy.html", "index.html"]) fs.writeFileSync(path.join(root, name), out);
console.log("Built routea_remote_adaptive_bluefy.html and index.html (" + Buffer.byteLength(out) + " bytes each)");
