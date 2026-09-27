import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import {
  DEFAULT_GAME3D_LAYOUT,
  mapRodPointToGame3D,
  mapRodVectorToGame3D,
} from "../docs/labs/x1-shower-tvc/game-3d-view.js";

const nearly = (a, b, tol = 1e-12) => Math.abs(a - b) <= tol;

const base = mapRodPointToGame3D([0, 0]);
assert.ok(nearly(base[0], 0));
assert.ok(nearly(base[1], DEFAULT_GAME3D_LAYOUT.handHeightM));
assert.ok(nearly(base[2], DEFAULT_GAME3D_LAYOUT.physicsPlaneZM));

const point = mapRodPointToGame3D([0.25, 1.10]);
assert.ok(nearly(point[0], 0.25));
assert.ok(
  nearly(
    point[1],
    DEFAULT_GAME3D_LAYOUT.handHeightM - 1.10,
  ),
);
assert.ok(nearly(point[2], 0));

const down = mapRodVectorToGame3D([0, 1]);
assert.ok(nearly(down[0], 0));
assert.ok(nearly(down[1], -1));
assert.ok(nearly(down[2], 0));
const right = mapRodVectorToGame3D([1, 0]);
assert.ok(nearly(right[0], 1));
assert.ok(nearly(right[1], 0));
assert.ok(nearly(right[2], 0));

const html = await readFile(
  "docs/labs/x1-shower-tvc/index.html",
  "utf8",
);
const ui = await readFile(
  "docs/labs/x1-shower-tvc/nonlinear-ui.js",
  "utf8",
);
const view = await readFile(
  "docs/labs/x1-shower-tvc/game-3d-view.js",
  "utf8",
);

for (const marker of [
  'id="nl3dViewTab"',
  'id="nl2dViewTab"',
  'id="nl3dViewPanel"',
  'id="nl2dViewPanel"',
  'id="nl3dCanvas"',
  'id="nl3dCamera"',
  '3D Game',
  '2D Debug',
  'data-game-difficulty="easy"',
  'data-game-difficulty="normal"',
  'data-game-difficulty="expert"',
  'data-game-difficulty="insane"',
  'id="nlGameHit"',
  'id="nlGameOverlay"',
  'id="nlResultRestart"',
]) {
  assert.ok(
    html.includes(marker),
    `missing H1-5-3 HTML marker: ${marker}`,
  );
}

assert.ok(
  !/id="nl3dViewPanel"[^>]*hidden/.test(html),
  "3D Game view must be visible by default",
);
assert.ok(
  /id="nl2dViewPanel"[^>]*hidden/.test(html),
  "2D Debug view must be hidden by default",
);

for (const marker of [
  'createGame3DView',
  'setVisualMode("3d")',
  'installPointerControl(game3dCanvas)',
  'installPointerControl(canvas)',
  'game3dView.render',
  'game3dView.setCameraView',
  'createDifficultyGameState',
  'evaluateWaterAim',
  'showGameResult',
]) {
  assert.ok(
    ui.includes(marker),
    `missing H1-5-3 UI wiring: ${marker}`,
  );
}

for (const marker of [
  'import * as THREE from "../../vendor/three.module.js"',
  "CylinderGeometry",
  "PerspectiveCamera",
  "mapRodPointToGame3D",
  "updateRod",
  "updateHead",
  "updateAimTarget",
  "aimTargetGroup",
  "aimClosestMarker",
]) {
  assert.ok(
    view.includes(marker),
    `missing H1-5-3 renderer marker: ${marker}`,
  );
}

console.log(
  "H1-5-4 3D gameplay checks OK:",
  JSON.stringify({
    base,
    samplePoint: point,
    downVector: down,
    defaultView: "3d",
  }),
);
