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
  'id="nlGameEffort"',
  'id="nlGameTargets"',
  'id="nlGameRestart"',
  'data-game-stage="low"',
  'data-game-stage="near"',
  'data-game-stage="flutter"',
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
  "createGameState",
  "updateGameState",
  "setGameControlLock",
  "startGame",
]) {
  assert.ok(
    nonlinearUi.includes(marker),
    `missing H1-5-1 Pointer wiring: ${marker}`,
  );
}

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
  `H1-5-2 visualization checks OK: portrait scale=${fit.scale.toFixed(2)} px/m, nonlinear default visible`,
);
