import { chapters, resolveContentReference } from "./routes.js";

const nav = document.querySelector("#theory-nav");
const content = document.querySelector("#content");
for (const [id, title] of chapters) {
  const li = document.createElement("li");
  const a = document.createElement("a");
  a.href = `#${id}`;
  a.textContent = title;
  li.append(a);
  nav.append(li);
}

function homeMarkdown() {
  return `# 二足歩行制御を、倒立振子から理解する

二足歩行の難しさは、**立っている状態そのものが自然には安定しない**ことにあります。
重心が少しずれると、そのずれを増やす向きに重力が働きます。ロボットはセンサで状態を測り、未来の転倒を予測し、足首・股関節・着地点を絶えず調整します。

<div class="callout"><strong>この教材の方針</strong><br>数式を読むだけではなく、各モデルをブラウザで壊します。ゲインを変え、押し、支持脚を減らし、「なぜ倒れたか」を式へ戻って確認します。</div>

## 第1版の実験

<div class="lab-grid">
<div class="lab-card"><strong>Lab 1 — 無制御倒立振子</strong><p>直立が不安定平衡点であることを体験。</p><a href="./labs/01-open-loop/">実験を開く →</a></div>
<div class="lab-card"><strong>Lab 2 — PD制御</strong><p>フィードバックで不安定系を安定化。</p><a href="./labs/02-pid-balance/">実験を開く →</a></div>
<div class="lab-card"><strong>Lab 3 — 支持多角形</strong><p>両足・片足で安定領域がどう変わるか見る。</p><a href="./labs/03-support-polygon/">実験を開く →</a></div>
<div class="lab-card"><strong>Lab 4 — LIPM歩行</strong><p>ZMPとcapture pointで外乱回復。</p><a href="./labs/04-lipm-walking/">実験を開く →</a></div>
</div>

## 補講 — 必要になったところで読む

本編は大学1年程度の微積分・線形代数・基礎物理から追えるようにし、それを越える内容は補講へ分離しています。

- [S01 常微分方程式と指数モード](#S01) — 時定数、特性方程式、連続時間と離散時間
- [S02 固有値・固有ベクトルを制御の言葉で読む](#S02) — モード分解、閉ループ固有値
- [S03 Lyapunov安定性](#S03) — 解を全部求めずに安定性を調べる
- [S04 状態推定とKalman filter](#S04) — センサから測れない状態を推定する
- [S05 Lagrange法と多リンクロボット力学](#S05) — 倒立振子から全身運動方程式へ
- [S06 接触・摩擦・CoP制約](#S06) — ZMP条件だけでは足りない理由
- [S07 Preview ControlとMPC](#S07) — 未来の支持切替を見ながら制約付き最適化
- [S08 Centroidal Dynamics](#S08) — 全身を重心と運動量へ圧縮して多接触を扱う
- [S09 Whole-Body QP](#S09) — 全身力学と摩擦・トルク制約の中で関節トルクを決める

## 現代ヒューマノイド制御への接続

本編07までの LIPM / Capture Point から先は、

**Preview / MPC → Centroidal Dynamics → Whole-Body QP**

と進むと、「未来の歩行計画」「全身外力・運動量」「関節トルク実現」を順番に理解できます。

## どこまで単純化しているか

第1版は2次元・剛体近似です。実機には接触・摩擦・関節制約・3次元運動・状態推定・遅延などが加わります。それでも、倒立振子、ZMP、LIPM、capture point は「なぜ歩行制御が難しいか」を見る強力な入口です。
`;
}

function assertRuntimeDependencies() {
  if (!globalThis.marked || typeof globalThis.marked.parse !== "function") {
    throw new Error("Markdown renderer (Marked) failed to load");
  }
  if (typeof globalThis.renderMathInElement !== "function") {
    throw new Error("Math renderer (KaTeX auto-render) failed to load");
  }
}

async function loadChapter() {
  try {
    assertRuntimeDependencies();
    const key = location.hash.replace(/^#/, "");
    if (!key) {
      render(homeMarkdown());
      return;
    }
    const chapter = chapters.find(([id]) => id === key);
    if (!chapter) {
      render("# 404\n\n指定された章はありません。[トップへ](./)");
      return;
    }
    const response = await fetch(`./theory/${chapter[2]}`);
    if (!response.ok) throw new Error(`Chapter fetch failed: HTTP ${response.status}`);
    render(await response.text(), new URL(`./theory/${chapter[2]}`, location.href));
  } catch (error) {
    console.error(error);
    content.innerHTML = `<h1>読み込みエラー</h1><p>${escapeHtml(String(error))}</p><p>ページを再読み込みしても直らない場合は、公開アセットの生成・配信を確認してください。</p>`;
  }
}

function escapeHtml(value) {
  return value.replace(/[&<>"']/g, (char) => ({
    "&": "&amp;",
    "<": "&lt;",
    ">": "&gt;",
    '"': "&quot;",
    "'": "&#39;"
  })[char]);
}

function protectMath(md) {
  const formulas = [];
  const stash = (value) => {
    const token = `BIPEDMATHTOKEN${formulas.length}END`;
    formulas.push(value);
    return token;
  };

  let protectedMd = md.replace(/\$\$[\s\S]*?\$\$/g, (value) => stash(value));
  protectedMd = protectedMd.replace(
    /(^|[^\\])\$(?!\$)([^\n$]+?)\$/gm,
    (_match, prefix, formula) => `${prefix}${stash(`$${formula}$`)}`
  );
  return { protectedMd, formulas };
}

function restoreMath(html, formulas) {
  return html.replace(
    /BIPEDMATHTOKEN(\d+)END/g,
    (_match, index) => escapeHtml(formulas[Number(index)] ?? "")
  );
}

function render(md, baseUrl = null) {
  const { protectedMd, formulas } = protectMath(md);
  const renderedMarkdown = globalThis.marked.parse(protectedMd, { gfm: true });
  content.innerHTML = restoreMath(renderedMarkdown, formulas);
  if (baseUrl) {
    for (const node of content.querySelectorAll("a[href], img[src]")) {
      const attr = node.hasAttribute("href") ? "href" : "src";
      const value = node.getAttribute(attr);
      if (value) {
        node.setAttribute(attr, resolveContentReference(value, baseUrl));
      }
    }
  }
  globalThis.renderMathInElement(content, {
    delimiters: [
      { left: "$$", right: "$$", display: true },
      { left: "$", right: "$", display: false }
    ],
    throwOnError: false
  });
  content.focus({ preventScroll: true });
  window.scrollTo({ top: 0, behavior: "instant" });
}

window.addEventListener("hashchange", loadChapter);
loadChapter();
