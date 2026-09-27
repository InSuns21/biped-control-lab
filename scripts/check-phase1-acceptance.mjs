import assert from "node:assert/strict";
import { readFileSync } from "node:fs";

import {
  createDifficultyGameState,
  gameHudSnapshot,
} from "../docs/js/shower/flexible/game.js";

const html = readFileSync(
  new URL("../docs/labs/x1-shower-tvc/index.html", import.meta.url),
  "utf8",
);
const ui = readFileSync(
  new URL("../docs/labs/x1-shower-tvc/nonlinear-ui.js", import.meta.url),
  "utf8",
);
const css = readFileSync(
  new URL("../docs/css/site.css", import.meta.url),
  "utf8",
);

function indexOfOrFail(source, marker, label = marker) {
  const index = source.indexOf(marker);
  assert.ok(index >= 0, `missing ${label}`);
  return index;
}

// Phase 1 acceptance: normal play must remain setup -> 3D play -> HUD ->
// secondary history. This is the tablet flow verified during H1-8A.
const setupIndex = indexOfOrFail(
  html,
  'class="game-panel game-launch-panel"',
  "game setup panel",
);
const canvasIndex = indexOfOrFail(
  html,
  'id="nl3dCanvas"',
  "3D game canvas",
);
const hudIndex = indexOfOrFail(
  html,
  'class="game-metrics game-primary-metrics game-below-canvas-hud"',
  "primary game HUD",
);
const historyIndex = indexOfOrFail(
  html,
  'class="controller-comparison-panel game-secondary-details"',
  "comparison history",
);
const debugIndex = indexOfOrFail(
  html,
  'class="advanced-lab-panel"',
  "advanced/debug controls",
);
assert.ok(
  setupIndex < canvasIndex
    && canvasIndex < hudIndex
    && hudIndex < historyIndex
    && historyIndex < debugIndex,
  "normal play order must remain setup -> canvas -> HUD -> history -> debug",
);

// PC + tablet pointer contract. Pointer Events cover mouse, pen and touch.
// touch-action:none prevents the browser's pan gesture from stealing gameplay.
for (const marker of [
  'installPointerControl(game3dCanvas)',
  'addEventListener("pointerdown"',
  'addEventListener("pointermove"',
  'addEventListener("pointerup"',
  'addEventListener("pointercancel"',
  "setPointerCapture(event.pointerId)",
  "releasePointerCapture(pointerId)",
]) {
  assert.ok(ui.includes(marker), `missing pointer contract: ${marker}`);
}
assert.ok(
  css.includes("touch-action: none"),
  "game canvases must disable native touch panning during pointer control",
);
assert.ok(
  css.includes("@media (max-width: 1024px)"),
  "tablet breakpoint must remain defined",
);
assert.ok(
  css.includes("min-height: 44px"),
  "touch targets must retain a 44px minimum at narrow widths",
);

// Game stability and diagnostic RMS intentionally use different references.
// Make that distinction explicit in both implementation and labels.
for (const marker of [
  "function gameStabilityMetrics(",
  "referenceEquilibriumKinematics(",
  "lateralPositionM: currentBoundary.lateralPositionM",
  "angleRad: currentBoundary.angleRad",
  "rmsM: gameMetrics.rmsM",
  "tipAngleErrorRad: gameMetrics.tipAngleErrorRad",
]) {
  assert.ok(
    ui.includes(marker),
    `missing moving-hand stability marker: ${marker}`,
  );
}
assert.ok(
  html.includes("Mean flex RMS（moving-hand基準）")
    && html.includes("dynamic RMS（static equilibrium基準）"),
  "UI must name the moving-hand and static-equilibrium RMS references",
);
assert.ok(
  html.includes('id="nlGameRms"')
    && ui.includes('gameRmsMetric.textContent = hud.rmsLabel'),
  "moving-frame mean RMS must be wired into the game HUD",
);
assert.ok(
  ui.includes("moving-hand ref: RMS≤"),
  "game target text must identify the moving-hand reference",
);

// Verify that the centralized HUD formatter and the DOM-facing UI use the
// same physical units and percentages.
const sample = {
  ...createDifficultyGameState("normal"),
  elapsedS: 2.25,
  score: 731,
  insideFraction: 0.75,
  aimHitFraction: 0.625,
  rmsMeanM: 0.01234,
  effortJ: 0.42,
  status: "running",
};
const hud = gameHudSnapshot(sample);
assert.equal(hud.status, "RUNNING");
assert.equal(hud.timeLabel, "8.8 s");
assert.equal(hud.scoreLabel, "731");
assert.equal(hud.targetLabel, "75% / 60%");
assert.equal(hud.hitLabel, "63% / 50%");
assert.equal(hud.rmsLabel, "12.3 mm");
assert.ok(
  /^0\.42 \/ [0-9.]+ J$/.test(hud.effortLabel),
  "effort HUD must stay in joules",
);

for (const mapping of [
  "gameTimeMetric.textContent = hud.timeLabel",
  "gameScoreMetric.textContent = hud.scoreLabel",
  "gameInsideMetric.textContent = hud.targetLabel",
  "gameHitMetric.textContent = hud.hitLabel",
  "gameRmsMetric.textContent = hud.rmsLabel",
  "gameEffortMetric.textContent = hud.effortLabel",
]) {
  assert.ok(ui.includes(mapping), `missing HUD mapping: ${mapping}`);
}

// Start flow remains armed/countdown based; START itself must not resume
// physics before Human tap or Auto countdown completion.
const startIndex = indexOfOrFail(
  ui,
  "async function startGame(difficultyId)",
  "startGame",
);
const restartFeedbackIndex = indexOfOrFail(
  ui.slice(startIndex),
  "function flashRestartFeedback()",
  "flashRestartFeedback",
) + startIndex;
const startBody = ui.slice(startIndex, restartFeedbackIndex);
assert.ok(
  !startBody.includes("paused = false"),
  "START must not resume physics immediately",
);
assert.ok(
  ui.includes('"READY — TAP TO PLAY"')
    && ui.includes("for (const count of [3, 2, 1])"),
  "Human ready gate and Auto countdown must remain active",
);

console.log(
  "H1-9 Phase 1 automated acceptance OK:",
  JSON.stringify({
    layout: "setup->3D->HUD->history->debug",
    pointer: "mouse/pen/touch Pointer Events + capture",
    tabletBreakpointPx: 1024,
    hudReference: "moving-hand flexible RMS",
    debugReference: "static equilibrium RMS",
    startFlow: "armed Human / countdown Auto",
  }),
);
