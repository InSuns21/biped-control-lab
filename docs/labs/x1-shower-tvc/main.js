import { mountFlexiblePhase } from "./flexible-ui.js";
import { mountNonlinearPhase } from "./nonlinear-ui.js";

const phase1Tab = document.querySelector("#phase1Tab");
const phase0Tab = document.querySelector("#phase0Tab");
const phase1Panel = document.querySelector("#phase1Panel");
const phase0Panel = document.querySelector("#phase0Panel");

const nonlinearModelTab = document.querySelector("#nonlinearModelTab");
const linearModelTab = document.querySelector("#linearModelTab");
const nonlinearModelPanel = document.querySelector("#nonlinearModelPanel");
const linearModelPanel = document.querySelector("#linearModelPanel");

const nonlinearApp = mountNonlinearPhase(nonlinearModelPanel);
const linearApp = mountFlexiblePhase(linearModelPanel);

let activeModel = "nonlinear";
let phase1Visible = true;
let phase0App = null;
let phase0Loading = false;

function updateModelState(model) {
  activeModel = model;
  const nonlinearActive = model === "nonlinear";

  nonlinearModelTab.setAttribute(
    "aria-selected",
    String(nonlinearActive),
  );
  linearModelTab.setAttribute(
    "aria-selected",
    String(!nonlinearActive),
  );
  nonlinearModelTab.tabIndex = nonlinearActive ? 0 : -1;
  linearModelTab.tabIndex = nonlinearActive ? -1 : 0;
  nonlinearModelPanel.hidden = !nonlinearActive;
  linearModelPanel.hidden = nonlinearActive;

  nonlinearApp.setActive(phase1Visible && nonlinearActive);
  linearApp.setActive(phase1Visible && !nonlinearActive);
}

function updateTabState(activePhase) {
  const phase1Active = activePhase === "phase1";
  phase1Visible = phase1Active;

  phase1Tab.setAttribute("aria-selected", String(phase1Active));
  phase0Tab.setAttribute("aria-selected", String(!phase1Active));
  phase1Tab.tabIndex = phase1Active ? 0 : -1;
  phase0Tab.tabIndex = phase1Active ? -1 : 0;
  phase1Panel.hidden = !phase1Active;
  phase0Panel.hidden = phase1Active;

  nonlinearApp.setActive(
    phase1Active && activeModel === "nonlinear",
  );
  linearApp.setActive(
    phase1Active && activeModel === "linear",
  );
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
nonlinearModelTab.addEventListener(
  "click",
  () => updateModelState("nonlinear"),
);
linearModelTab.addEventListener(
  "click",
  () => updateModelState("linear"),
);

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

[nonlinearModelTab, linearModelTab].forEach((tab, index, tabs) => {
  tab.addEventListener("keydown", (event) => {
    if (event.key !== "ArrowLeft" && event.key !== "ArrowRight") return;
    event.preventDefault();
    const delta = event.key === "ArrowRight" ? 1 : -1;
    const next = tabs[(index + delta + tabs.length) % tabs.length];
    next.focus();
    next.click();
  });
});

updateModelState("nonlinear");
updateTabState("phase1");
