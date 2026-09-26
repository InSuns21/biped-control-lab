import { mountFlexiblePhase } from "./flexible-ui.js";

const phase1Tab = document.querySelector("#phase1Tab");
const phase0Tab = document.querySelector("#phase0Tab");
const phase1Panel = document.querySelector("#phase1Panel");
const phase0Panel = document.querySelector("#phase0Panel");

const phase1App = mountFlexiblePhase(phase1Panel);
phase1App.setActive(true);

let phase0App = null;
let phase0Loading = false;

function updateTabState(activePhase) {
  const phase1Active = activePhase === "phase1";
  phase1Tab.setAttribute("aria-selected", String(phase1Active));
  phase0Tab.setAttribute("aria-selected", String(!phase1Active));
  phase1Tab.tabIndex = phase1Active ? 0 : -1;
  phase0Tab.tabIndex = phase1Active ? -1 : 0;
  phase1Panel.hidden = !phase1Active;
  phase0Panel.hidden = phase1Active;
  phase1App.setActive(phase1Active);
  phase0App?.setActive(!phase1Active);
}

async function ensurePhase0() {
  if (phase0App || phase0Loading) return;
  phase0Loading = true;

  try {
    const { mountPhase0 } = await import("./phase0.js");
    phase0App = mountPhase0(phase0Panel);
  } catch (error) {
    const message = document.createElement("div");
    message.className = "callout status-danger";
    message.setAttribute("role", "alert");
    const detail = error instanceof Error ? error.message : String(error);
    message.innerHTML = "<strong>Phase 0 の3D描画を初期化できませんでした。</strong>"
      + "<br><code></code>";
    message.querySelector("code").textContent = detail;
    phase0Panel.prepend(message);
    console.error("Phase 0 initialization failed", error);
  } finally {
    phase0Loading = false;
  }
}

async function activatePhase(activePhase) {
  if (activePhase === "phase0") {
    await ensurePhase0();
  }
  updateTabState(activePhase);
}

phase1Tab.addEventListener("click", () => activatePhase("phase1"));
phase0Tab.addEventListener("click", () => activatePhase("phase0"));

[phase1Tab, phase0Tab].forEach((tab, index, tabs) => {
  tab.addEventListener("keydown", (event) => {
    if (event.key !== "ArrowLeft" && event.key !== "ArrowRight") return;
    event.preventDefault();
    const delta = event.key === "ArrowRight" ? 1 : -1;
    const next = tabs[(index + delta + tabs.length) % tabs.length];
    next.focus();
    next.click();
  });
});

updateTabState("phase1");
