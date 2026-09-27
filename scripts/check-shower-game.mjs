import assert from "node:assert/strict";
import {
  createAimTarget,
  createDifficultyGameState,
  createGameState,
  difficultyAimMaxSpeedMps,
  difficultyAimOffsetM,
  evaluateWaterAim,
  gameDifficultyById,
  gameHudSnapshot,
  gameStageById,
  scoreGameState,
  SHOWER_GAME_DIFFICULTIES,
  SHOWER_GAME_STAGES,
  updateGameState,
  wrapAngleRad,
} from "../docs/js/shower/flexible/game.js";

assert.equal(Object.keys(SHOWER_GAME_STAGES).length, 3);
assert.equal(gameStageById("low").scenario.flowLpm, 12);
assert.equal(gameStageById("near").scenario.flowLpm, 18);
assert.equal(gameStageById("flutter").scenario.flowLpm, 22);

assert.equal(Object.keys(SHOWER_GAME_DIFFICULTIES).length, 4);
assert.equal(gameDifficultyById("easy").presetId, "low12");
assert.equal(gameDifficultyById("normal").presetId, "baseline18");
assert.equal(gameDifficultyById("expert").presetId, "baseline18");
assert.equal(gameDifficultyById("insane").presetId, "fast22");
assert.ok(
  gameDifficultyById("easy").authorityScale
    > gameDifficultyById("expert").authorityScale,
);
assert.ok(
  gameDifficultyById("easy").aimRadiusM
    > gameDifficultyById("insane").aimRadiusM,
);

for (const difficulty of Object.values(SHOWER_GAME_DIFFICULTIES)) {
  const schedule = difficulty.aimSchedule;
  assert.ok(Array.isArray(schedule) && schedule.length >= 2);
  for (let i = 1; i < schedule.length; i += 1) {
    assert.ok(
      schedule[i].timeS > schedule[i - 1].timeS,
      `${difficulty.id} schedule times must increase`,
    );
  }

  const maxOffset = Math.max(
    ...schedule.map((entry) => Math.abs(entry.normalOffsetM)),
  );
  const geometricReachM = 0.12
    + difficulty.aimDistanceM * Math.tan(Math.PI / 4);
  assert.ok(
    maxOffset < geometricReachM,
    `${difficulty.id} target offset must be geometrically reachable`,
  );

  const maxTargetSpeedMps = difficultyAimMaxSpeedMps(difficulty);
  const roughControlSpeedMps = 0.75 * difficulty.authorityScale
    + difficulty.aimDistanceM * 4.5 * difficulty.authorityScale;
  assert.ok(
    maxTargetSpeedMps < 0.45 * roughControlSpeedMps,
    `${difficulty.id} target motion must stay slower than available hand authority`,
  );
}

for (const difficulty of Object.values(SHOWER_GAME_DIFFICULTIES)) {
  const target = createAimTarget({
    nozzleOrigin: [0, 0],
    outletDirection: [1, 0],
    distanceM: difficulty.aimDistanceM,
    normalOffsetM: difficulty.aimNormalOffsetM,
    radiusM: difficulty.aimRadiusM,
  });
  const referenceAim = evaluateWaterAim({
    nozzleOrigin: [0, 0],
    outletDirection: [1, 0],
    target,
  });
  assert.equal(
    referenceAim.hit,
    true,
    `${difficulty.id} reference water ray must intersect its initial bullseye`,
  );
}

// Physics-plane water ray / circular target geometry.
const aimTarget = createAimTarget({
  nozzleOrigin: [0, 0],
  outletDirection: [1, 0],
  distanceM: 0.5,
  radiusM: 0.10,
});
const directHit = evaluateWaterAim({
  nozzleOrigin: [0, 0],
  outletDirection: [1, 0],
  target: aimTarget,
});
assert.equal(directHit.hit, true);
assert.ok(directHit.missDistanceM < 1e-12);
assert.ok(directHit.aimQuality > 0.999);

const nearHit = evaluateWaterAim({
  nozzleOrigin: [0, 0.08],
  outletDirection: [1, 0],
  target: aimTarget,
});
assert.equal(nearHit.hit, true);
assert.ok(Math.abs(nearHit.missDistanceM - 0.08) < 1e-12);

const miss = evaluateWaterAim({
  nozzleOrigin: [0, 0.16],
  outletDirection: [1, 0],
  target: aimTarget,
});
assert.equal(miss.hit, false);
assert.ok(miss.missDistanceM > aimTarget.radiusM);

const behind = evaluateWaterAim({
  nozzleOrigin: [0.7, 0],
  outletDirection: [1, 0],
  target: aimTarget,
});
assert.equal(behind.hit, false);

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

// H1-5-4: stability alone is not enough when aiming is enabled.
let aimFailure = createDifficultyGameState("easy");
while (aimFailure.status === "running") {
  aimFailure = updateGameState(
    aimFailure,
    {
      rmsM: 0,
      tipAngleErrorRad: 0,
      handPowerW: 0,
      actuatorSaturated: false,
      waterHit: false,
      waterMissDistanceM: 0.20,
      aimQuality: 0,
    },
    dt,
  );
}
assert.equal(aimFailure.status, "failed");
assert.equal(aimFailure.failureReason, "aim ratio");
assert.ok(aimFailure.insideFraction > 0.999);
assert.ok(aimFailure.aimHitFraction < 1e-9);

let fullSuccess = createDifficultyGameState("normal");
while (fullSuccess.status === "running") {
  fullSuccess = updateGameState(
    fullSuccess,
    {
      rmsM: 0.010,
      tipAngleErrorRad: 0.02,
      handPowerW: 0.01,
      actuatorSaturated: false,
      waterHit: true,
      waterMissDistanceM: 0.01,
      aimQuality: 0.95,
    },
    dt,
  );
}
assert.equal(fullSuccess.status, "success");
assert.ok(fullSuccess.aimHitFraction > 0.999);
assert.ok(fullSuccess.insideFraction > 0.999);
assert.ok(fullSuccess.score > 800);

// H1-5-4A: "do nothing" must not be the winning strategy.
// Hold the nozzle exactly on its equilibrium ray and let only the bullseye
// schedule move. Stability is perfect, but aiming should eventually fail.
const zeroInputResults = {};
for (const difficulty of Object.values(SHOWER_GAME_DIFFICULTIES)) {
  let baseline = createDifficultyGameState(difficulty.id);
  const baseOrigin = [0, 0];
  const baseDirection = [1, 0];

  while (baseline.status === "running") {
    const offset = difficultyAimOffsetM(
      difficulty,
      baseline.elapsedS,
    );
    const target = createAimTarget({
      nozzleOrigin: baseOrigin,
      outletDirection: baseDirection,
      distanceM: difficulty.aimDistanceM,
      normalOffsetM: offset,
      radiusM: difficulty.aimRadiusM,
    });
    const aim = evaluateWaterAim({
      nozzleOrigin: baseOrigin,
      outletDirection: baseDirection,
      target,
    });

    baseline = updateGameState(
      baseline,
      {
        rmsM: 0,
        tipAngleErrorRad: 0,
        handPowerW: 0,
        actuatorSaturated: false,
        waterHit: aim.hit,
        waterMissDistanceM: aim.missDistanceM,
        aimQuality: aim.aimQuality,
      },
      dt,
    );
  }

  zeroInputResults[difficulty.id] = {
    status: baseline.status,
    failureReason: baseline.failureReason,
    hitFraction: baseline.aimHitFraction,
    score: baseline.score,
  };

  assert.equal(
    baseline.status,
    "failed",
    `${difficulty.id}: zero-input baseline must not succeed`,
  );
  assert.equal(
    baseline.failureReason,
    "aim ratio",
    `${difficulty.id}: zero-input baseline should fail on aiming, not fake instability`,
  );
}

// Conversely, a reachable assisted trace that continuously points the outlet
// ray through the moving target must be able to satisfy the game rules.
const assistedResults = {};
for (const difficulty of Object.values(SHOWER_GAME_DIFFICULTIES)) {
  let assisted = createDifficultyGameState(difficulty.id);
  while (assisted.status === "running") {
    assisted = updateGameState(
      assisted,
      {
        rmsM: 0.25 * difficulty.targetRmsM,
        tipAngleErrorRad:
          0.25 * difficulty.targetTipAngleErrorRad,
        handPowerW: 0.03,
        actuatorSaturated: false,
        waterHit: true,
        waterMissDistanceM: 0.1 * difficulty.aimRadiusM,
        aimQuality: 0.95,
      },
      dt,
    );
  }
  assistedResults[difficulty.id] = {
    status: assisted.status,
    score: assisted.score,
  };
  assert.equal(
    assisted.status,
    "success",
    `${difficulty.id}: assisted reachable trace must remain winnable`,
  );
}

// HUD snapshot should remain finite and readable.
const hud = gameHudSnapshot(efficient);
for (const value of Object.values(hud)) {
  assert.ok(!String(value).includes("NaN"));
  assert.ok(!String(value).includes("Infinity"));
}

console.log(
  "H1-5-4A playability checks OK:",
  JSON.stringify({
    lowScore: success.score,
    heldFailureAtS: failure.elapsedS,
    ratioFailureInside: ratioFailure.insideFraction,
    efficientScore: scoreGameState(efficient),
    wastefulScore: scoreGameState(wasteful),
    wastefulEffortJ: wasteful.effortJ,
    easyAimFailure: aimFailure.failureReason,
    normalScore: fullSuccess.score,
    zeroInputResults,
    assistedResults,
  }),
);
