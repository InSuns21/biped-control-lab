import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import {
  computeNonlinearWorldFit,
} from "../docs/labs/x1-shower-tvc/nonlinear-view.js";

const html = await readFile(
  "docs/labs/x1-shower-tvc/index.html",
  "utf8",
);
const nonlinearUi = await readFile(
  "docs/labs/x1-shower-tvc/nonlinear-ui.js",
  "utf8",
);

for (const marker of [
  'id="nonlinearModelTab"',
  'id="linearModelTab"',
  'id="nonlinearModelPanel"',
  'id="linearModelPanel"',
  'id="nlCanvas"',
  'data-nl-preset="baseline18"',
  'data-nl-preset="fast22"',
  'data-nl-preset="high30"',
  'id="nlRmsChart"',
  'id="nlTipChart"',
  'id="nlPulseDuration"',
  'data-hand-pulse="left"',
  'data-hand-pulse="right"',
  'data-hand-pulse="ccw"',
  'data-hand-pulse="cw"',
  'id="nlHandMetric"',
  'id="nlHandReactionMetric"',
  'id="nlHandPowerMetric"',
  'id="nlHandWorkMetric"',
  'id="nlHandTargetMetric"',
  'id="nlActuatorMetric"',
  'id="nlPointerMetric"',
  'id="nlCenterHand"',
  'id="nlHandXChart"',
  'id="nlHandAngleChart"',
  'id="nlGameStage"',
  'id="nlGameStatus"',
  'id="nlGameTime"',
  'id="nlGameScore"',
  'id="nlGameInside"',
  'id="nlGameHit"',
  'id="nlGameEffort"',
  'id="nlGameTargets"',
  'id="nlGameRestart"',
  'id="nlControlMode"',
  'id="nlControlSenseMetric"',
  'id="nlControlCommandMetric"',
  'id="nlLqrStateMetric"',
  'id="nlLqrDesignMetric"',
  '<option value="state">Auto: State FB — 自動プレイ</option>',
  '<option value="lqr">Auto: LQR — 自動プレイ</option>',
  'id="nlGameStart"',
  'id="nlBeginHuman"',
  'id="nlControlModeHelp"',
  'id="nlPlayInstruction"',
  'class="advanced-lab-panel"',
  'class="advanced-analysis-panel"',
  'id="nlComparisonDifficulty"',
  'id="nlComparisonBody"',
  'id="nlComparisonReset"',
  'data-game-difficulty="easy"',
  'data-game-difficulty="normal"',
  'data-game-difficulty="expert"',
  'data-game-difficulty="insane"',
  'id="nlGameOverlay"',
  'id="nlGameOverlayTitle"',
  'id="nlGameOverlayScore"',
  'id="nlResultRestart"',
]) {
  assert.ok(html.includes(marker), `missing H1-4B UI marker: ${marker}`);
}

assert.ok(
  !html.includes("ねとらぼ動画の実ホース値へフィット"),
  "redundant fit disclaimer must not be shown in the lab UI",
);
assert.ok(
  !html.includes("実製品同定ではありません"),
  "redundant identification disclaimer must not be shown in the lab UI",
);

for (const marker of [
  'addEventListener("pointerdown"',
  'addEventListener("pointermove"',
  'addEventListener("pointerup"',
  'addEventListener("pointercancel"',
  "setPointerCapture",
  "pointerDeltaToHandTarget",
  "stepHandActuator",
  "actuatorBoundaryTrajectory",
  "createDifficultyGameState",
  "difficultyAimOffsetM",
  "evaluateWaterAim",
  "updateGameState",
  "updateScheduledAimTarget",
  "setGameControlLock",
  "startGame",
  "setControlMode",
  "senseTipFeedback",
  "controllerHandTarget",
  "designFullStateLqr",
  "lqrHandTarget",
  "automaticGameHandTarget",
  "FIXED_STATE_FEEDBACK_GAIN_SCALE",
  "recordComparisonResult",
  "updateComparisonTable",
  "ensureLqrDesign",
  "lqrDesignCache",
  "restartGame",
  "configureResponsive",
  "solutionCache",
  "LOADING…",
  "historyDirty",
]) {
  assert.ok(
    nonlinearUi.includes(marker),
    `missing H1-5-1 Pointer wiring: ${marker}`,
  );
}


const lockStart = nonlinearUi.indexOf(
  "function setGameControlLock(locked)",
);
const viewStart = nonlinearUi.indexOf(
  "function setVisualMode(mode)",
  lockStart,
);
assert.ok(lockStart >= 0 && viewStart > lockStart);
const lockBody = nonlinearUi.slice(lockStart, viewStart);
assert.ok(
  !lockBody.includes("addEventListener"),
  "setGameControlLock must not register event listeners",
);

assert.equal(
  nonlinearUi.split('game3dViewTab.addEventListener(').length - 1,
  1,
  "3D view-tab click handler must be registered exactly once",
);
assert.equal(
  nonlinearUi.split('debug2dViewTab.addEventListener(').length - 1,
  1,
  "2D view-tab click handler must be registered exactly once",
);

assert.equal(
  nonlinearUi.split('controlModeSelect.addEventListener("change"').length - 1,
  1,
  "feedback mode change handler must be registered exactly once",
);

assert.ok(
  nonlinearUi.includes('gameStartButton.addEventListener("click"'),
  "game must have an explicit START action",
);
assert.ok(
  nonlinearUi.includes("await startGame(selectedDifficultyId)"),
  "START must launch the selected difficulty",
);
assert.ok(
  nonlinearUi.includes("selectDifficulty(button.dataset.gameDifficulty)"),
  "difficulty buttons must select without immediately starting",
);
assert.ok(
  !nonlinearUi.includes("await startGame(button.dataset.gameDifficulty)"),
  "difficulty selection must not immediately start the game",
);
assert.ok(
  nonlinearUi.includes('paused = true;\n      pauseButton.textContent = "再開";'),
  "pre-game nonlinear physics should remain paused until START",
);
assert.ok(
  html.includes("Auto: PD — 自動プレイ")
    && html.includes("START を押すまでゲームは始まりません"),
  "auto-play and explicit-start guidance must be visible",
);

const launcherIndex = html.indexOf(
  'class="game-panel game-launch-panel"',
);
const canvasIndex = html.indexOf('id="nl3dCanvas"');
const hudIndex = html.indexOf(
  'class="game-metrics game-primary-metrics game-below-canvas-hud"',
);
const comparisonIndex = html.indexOf(
  'class="controller-comparison-panel game-secondary-details"',
);
assert.ok(
  launcherIndex >= 0
    && canvasIndex > launcherIndex
    && hudIndex > canvasIndex
    && comparisonIndex > hudIndex,
  "tablet flow must be setup -> 3D canvas -> HUD -> comparison history",
);

for (const marker of [
  'gameLaunchPhase = "armed-human"',
  'function beginGameRun(',
  'function runAutomaticCountdown(',
  'for (const count of [3, 2, 1])',
  'focusGameplayView()',
  'beginHumanButton.addEventListener("click"',
  '"READY — TAP TO PLAY"',
]) {
  assert.ok(
    nonlinearUi.includes(marker),
    `missing H1-8A ready/countdown marker: ${marker}`,
  );
}

const startGameStart = nonlinearUi.indexOf(
  "async function startGame(difficultyId)",
);
const flashRestartStart = nonlinearUi.indexOf(
  "function flashRestartFeedback()",
  startGameStart,
);
assert.ok(startGameStart >= 0 && flashRestartStart > startGameStart);
const startGameBody = nonlinearUi.slice(
  startGameStart,
  flashRestartStart,
);
assert.ok(
  !startGameBody.includes("paused = false"),
  "START must not immediately advance physics",
);
assert.ok(
  startGameBody.includes("armPreparedGame("),
  "START must arm a prepared run rather than play immediately",
);

const beginRunStart = nonlinearUi.indexOf(
  "function beginGameRun(",
);
const countdownStart = nonlinearUi.indexOf(
  "async function runAutomaticCountdown(",
  beginRunStart,
);
assert.ok(beginRunStart >= 0 && countdownStart > beginRunStart);
const beginRunBody = nonlinearUi.slice(beginRunStart, countdownStart);
assert.ok(
  beginRunBody.includes('gameLaunchPhase = "running"')
    && beginRunBody.includes("paused = false"),
  "physics may resume only from the explicit begin transition",
);

const restartStart = nonlinearUi.indexOf("function restartGame()");
const recordStart = nonlinearUi.indexOf(
  "function recordHistory()",
  restartStart,
);
assert.ok(restartStart >= 0 && recordStart > restartStart);
const restartBody = nonlinearUi.slice(restartStart, recordStart);
assert.ok(
  restartBody.includes("resetSimulationState()"),
  "Restart must reset the current solved scenario directly",
);
assert.ok(
  !restartBody.includes("solveContinuation"),
  "Restart must not solve static equilibrium again",
);
assert.ok(
  !restartBody.includes("configureResponsive"),
  "Restart must not enter the heavy configure path",
);
assert.ok(
  nonlinearUi.includes('gameRestartButton.textContent = "Ready ✓"'),
  "Restart needs immediate ready-state feedback",
);
assert.ok(
  nonlinearUi.includes("void configureResponsive({ preserveFlow: false })"),
  "initial nonlinear solve must be deferred through the responsive path",
);
assert.ok(
  nonlinearUi.includes("!initializationBusy\n      && scenario\n      && state"),
  "physics stepping must wait until async initialization is ready",
);
assert.ok(
  nonlinearUi.includes(
    "setPointerTargetFromDelta(controlCanvas, dx, dy);\n      render();",
  ),
  "Pointer movement should render target feedback immediately",
);

assert.ok(
  !/id="nonlinearModelPanel"[^>]*hidden/.test(html),
  "H1-4B nonlinear panel must be visible by default",
);
assert.ok(
  /id="linearModelPanel"[^>]*hidden/.test(html),
  "H1-4 linear panel must be hidden by default",
);

const portraitPoints = [
  [0, 0],
  [-0.65, 0.45],
  [-0.20, 1.20],
  [0.35, 1.45],
  [0.55, 1.65],
];
const fit = computeNonlinearWorldFit(
  portraitPoints,
  360,
  540,
);
for (const point of portraitPoints) {
  const [x, y] = fit.map(point);
  assert.ok(x >= 0 && x <= 360, "world-fit x must remain inside portrait canvas");
  assert.ok(y >= 0 && y <= 540, "world-fit y must remain inside portrait canvas");
}

console.log(
  `H1-6-3 human-vs-controller visualization checks OK: portrait scale=${fit.scale.toFixed(2)} px/m, nonlinear default visible`,
);
