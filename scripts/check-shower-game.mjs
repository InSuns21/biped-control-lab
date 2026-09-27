import assert from "node:assert/strict";
import {
  createGameState,
  gameHudSnapshot,
  gameStageById,
  scoreGameState,
  SHOWER_GAME_STAGES,
  updateGameState,
  wrapAngleRad,
} from "../docs/js/shower/flexible/game.js";

assert.equal(Object.keys(SHOWER_GAME_STAGES).length, 3);
assert.equal(gameStageById("low").scenario.flowLpm, 12);
assert.equal(gameStageById("near").scenario.flowLpm, 18);
assert.equal(gameStageById("flutter").scenario.flowLpm, 22);

assert.ok(Math.abs(wrapAngleRad(3 * Math.PI) - Math.PI) < 1e-12);
assert.ok(Math.abs(wrapAngleRad(-3 * Math.PI) + Math.PI) < 1e-12);

// Perfect control should succeed with a near-perfect score.
let success = createGameState("low");
const dt = 0.01;
while (success.status === "running") {
  success = updateGameState(
    success,
    {
      rmsM: 0,
      tipAngleErrorRad: 0,
      handPowerW: 0,
      actuatorSaturated: false,
    },
    dt,
  );
}
assert.equal(success.status, "success");
assert.ok(success.insideFraction > 0.999);
assert.equal(success.score, 1000);

// Failure envelope is intentionally hold-time based, not a one-sample spike.
let failure = createGameState("flutter");
for (let i = 0; i < 20; i += 1) {
  failure = updateGameState(
    failure,
    {
      rmsM: failure.stage.failRmsM * 1.2,
      tipAngleErrorRad: 0,
      handPowerW: 0,
      actuatorSaturated: false,
    },
    0.01,
  );
}
assert.equal(failure.status, "running");
for (let i = 0; i < 30; i += 1) {
  failure = updateGameState(
    failure,
    {
      rmsM: failure.stage.failRmsM * 1.2,
      tipAngleErrorRad: 0,
      handPowerW: 0,
      actuatorSaturated: false,
    },
    0.01,
  );
}
assert.equal(failure.status, "failed");
assert.equal(failure.failureReason, "failure envelope");

// Surviving outside the target can still fail at the time limit.
let ratioFailure = createGameState("near");
while (ratioFailure.status === "running") {
  ratioFailure = updateGameState(
    ratioFailure,
    {
      rmsM: 1.2 * ratioFailure.stage.targetRmsM,
      tipAngleErrorRad:
        1.2 * ratioFailure.stage.targetTipAngleErrorRad,
      handPowerW: 0,
      actuatorSaturated: false,
    },
    dt,
  );
}
assert.equal(ratioFailure.status, "failed");
assert.equal(ratioFailure.failureReason, "target ratio");

// Net hand work can cancel, but effort must not.
let efficient = createGameState("low");
let wasteful = createGameState("low");
for (let i = 0; i < 200; i += 1) {
  efficient = updateGameState(
    efficient,
    {
      rmsM: 0.010,
      tipAngleErrorRad: 0.02,
      handPowerW: 0,
      actuatorSaturated: false,
    },
    dt,
  );
  wasteful = updateGameState(
    wasteful,
    {
      rmsM: 0.010,
      tipAngleErrorRad: 0.02,
      handPowerW: i % 2 === 0 ? 4 : -4,
      actuatorSaturated: true,
    },
    dt,
  );
}
assert.ok(Math.abs(wasteful.netWorkJ) < 1e-9);
assert.ok(wasteful.effortJ > 7.9);
assert.ok(scoreGameState(wasteful) < scoreGameState(efficient));

// HUD snapshot should remain finite and readable.
const hud = gameHudSnapshot(efficient);
for (const value of Object.values(hud)) {
  assert.ok(!String(value).includes("NaN"));
  assert.ok(!String(value).includes("Infinity"));
}

console.log(
  "H1-5-2 game-rule checks OK:",
  JSON.stringify({
    lowScore: success.score,
    heldFailureAtS: failure.elapsedS,
    ratioFailureInside: ratioFailure.insideFraction,
    efficientScore: scoreGameState(efficient),
    wastefulScore: scoreGameState(wasteful),
    wastefulEffortJ: wasteful.effortJ,
  }),
);
