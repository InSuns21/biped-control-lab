const chapters = [
  ["01", "なぜ二足立位は難しいのか", "01_why_biped_is_hard.md"],
  ["02", "倒立振子", "02_inverted_pendulum.md"],
  ["03", "PID / PDフィードバック", "03_pid_feedback.md"],
  ["04", "状態空間と状態フィードバック", "04_state_space.md"],
  ["05", "LQR", "05_lqr.md"],
  ["06", "支持多角形とZMP", "06_zmp_support_polygon.md"],
  ["07", "LIPMとステップ制御", "07_lipm_step_control.md"]
];

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
<div class="lab-card"><strong>Lab 3 — 支持多角形</strong><p>両足・片足で静的安定領域を比較。</p><a href="./labs/03-support-polygon/">実験を開く →</a></div>
<div class="lab-card"><strong>Lab 4 — LIPM歩行</strong><p>ZMPとcapture pointで外乱回復。</p><a href="./labs/04-lipm-walking/">実験を開く →</a></div>
</div>

## どこまで単純化しているか

第1版は2次元・剛体近似です。実機には接触・摩擦・関節制約・3次元運動・状態推定・遅延などが加わります。それでも、倒立振子、ZMP、LIPM、capture point は「なぜ歩行制御が難しいか」を見る強力な入口です。
`;
}

async function loadChapter() {
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
  try {
    const response = await fetch(`./theory/${chapter[2]}`);
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    render(await response.text(), new URL(`./theory/${chapter[2]}`, location.href));
  } catch (error) {
    content.innerHTML = `<h1>読み込みエラー</h1><p>${String(error)}</p>`;
  }
}

function render(md, baseUrl = null) {
  content.innerHTML = marked.parse(md, { gfm: true });
  if (baseUrl) {
    for (const node of content.querySelectorAll("a[href], img[src]")) {
      const attr = node.hasAttribute("href") ? "href" : "src";
      const value = node.getAttribute(attr);
      if (value && !value.startsWith("#") && !/^[a-z]+:/i.test(value)) {
        node.setAttribute(attr, new URL(value, baseUrl).href);
      }
    }
  }
  renderMathInElement(content, {
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
