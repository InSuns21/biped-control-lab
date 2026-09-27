const degToRad = (deg) => deg * Math.PI / 180;

export const SHOWER_GAME_STAGES = Object.freeze({
  low: Object.freeze({
    id: "low",
    title: "F2 Low Flow",
    description: "安定域。操作とスコアの練習。",
    presetId: "low12",
    durationS: 8,
    targetRmsM: 0.035,
    targetTipAngleErrorRad: degToRad(10),
    failRmsM: 0.15,
    failTipAngleErrorRad: degToRad(50),
    failHoldS: 0.50,
    minInsideFraction: 0.80,
    effortBudgetJ: 0.40,
    scenario: Object.freeze({
      flowLpm: 12,
      lengthM: 1.2,
      flexuralRigidityNm2: 0.7,
      rayleighMassPerS: 0.08,
      rayleighStiffnessS: 0.0002,
    }),
  }),
  near: Object.freeze({
    id: "near",
    title: "F3 Near Critical",
    description: "振動が残りやすい領域を一定時間抑える。",
    presetId: "baseline18",
    durationS: 10,
    targetRmsM: 0.050,
    targetTipAngleErrorRad: degToRad(15),
    failRmsM: 0.22,
    failTipAngleErrorRad: degToRad(65),
    failHoldS: 0.50,
    minInsideFraction: 0.70,
    effortBudgetJ: 0.70,
    scenario: Object.freeze({
      flowLpm: 18,
      lengthM: 1.2,
      flexuralRigidityNm2: 0.7,
      rayleighMassPerS: 0.08,
      rayleighStiffnessS: 0.0002,
    }),
  }),
  flutter: Object.freeze({
    id: "flutter",
    title: "F4/F5 Flutter",
    description: "自励成長するホースを手元境界で抑える。",
    presetId: "fast22",
    durationS: 12,
    targetRmsM: 0.070,
    targetTipAngleErrorRad: degToRad(18),
    failRmsM: 0.30,
    failTipAngleErrorRad: degToRad(80),
    failHoldS: 0.45,
    minInsideFraction: 0.55,
    effortBudgetJ: 1.20,
    scenario: Object.freeze({
      flowLpm: 22,
      lengthM: 1.5,
      flexuralRigidityNm2: 0.25,
      rayleighMassPerS: 0.02,
      rayleighStiffnessS: 0.0002,
    }),
  }),
});

export const SHOWER_GAME_DIFFICULTIES = Object.freeze({
  easy: Object.freeze({
    id: "easy",
    stageId: "low",
    title: "Easy",
    subtitle: "Low Flow / moving target",
    description: "2.5秒の猶予後、広い照準がゆっくり移動する入門。",
    presetId: "low12",
    durationS: 10,
    targetRmsM: 0.060,
    targetTipAngleErrorRad: degToRad(20),
    failRmsM: 0.24,
    failTipAngleErrorRad: degToRad(75),
    failHoldS: 0.70,
    minInsideFraction: 0.60,
    effortBudgetJ: 1.00,
    aimDistanceM: 0.48,
    aimRadiusM: 0.15,
    minHitFraction: 0.45,
    authorityScale: 1.25,
    aimSchedule: Object.freeze([
      Object.freeze({ timeS: 0, normalOffsetM: 0 }),
      Object.freeze({ timeS: 2.5, normalOffsetM: 0 }),
      Object.freeze({ timeS: 3.7, normalOffsetM: 0.20 }),
      Object.freeze({ timeS: 10, normalOffsetM: 0.20 }),
    ]),
  }),
  normal: Object.freeze({
    id: "normal",
    stageId: "near",
    title: "Normal",
    subtitle: "18 L/min / tracking",
    description: "開始時から照準が外れ、中盤で反対側へ移動する標準。",
    presetId: "baseline18",
    durationS: 11,
    targetRmsM: 0.065,
    targetTipAngleErrorRad: degToRad(20),
    failRmsM: 0.25,
    failTipAngleErrorRad: degToRad(72),
    failHoldS: 0.60,
    minInsideFraction: 0.60,
    effortBudgetJ: 1.10,
    aimDistanceM: 0.50,
    aimRadiusM: 0.12,
    minHitFraction: 0.50,
    authorityScale: 1.12,
    aimSchedule: Object.freeze([
      Object.freeze({ timeS: 0, normalOffsetM: 0.16 }),
      Object.freeze({ timeS: 4.5, normalOffsetM: 0.16 }),
      Object.freeze({ timeS: 5.7, normalOffsetM: -0.16 }),
      Object.freeze({ timeS: 11, normalOffsetM: -0.16 }),
    ]),
  }),
  expert: Object.freeze({
    id: "expert",
    stageId: "near",
    title: "Expert",
    subtitle: "18 L/min / multi-target",
    description: "安定化しながら複数回移動する小照準を追う。",
    presetId: "baseline18",
    durationS: 12,
    targetRmsM: 0.050,
    targetTipAngleErrorRad: degToRad(15),
    failRmsM: 0.22,
    failTipAngleErrorRad: degToRad(65),
    failHoldS: 0.50,
    minInsideFraction: 0.70,
    effortBudgetJ: 0.90,
    aimDistanceM: 0.50,
    aimRadiusM: 0.090,
    minHitFraction: 0.55,
    authorityScale: 1.00,
    aimSchedule: Object.freeze([
      Object.freeze({ timeS: 0, normalOffsetM: 0.13 }),
      Object.freeze({ timeS: 2.3, normalOffsetM: 0.13 }),
      Object.freeze({ timeS: 3.1, normalOffsetM: -0.14 }),
      Object.freeze({ timeS: 5.4, normalOffsetM: -0.14 }),
      Object.freeze({ timeS: 6.2, normalOffsetM: 0.16 }),
      Object.freeze({ timeS: 8.6, normalOffsetM: 0.16 }),
      Object.freeze({ timeS: 9.4, normalOffsetM: -0.12 }),
      Object.freeze({ timeS: 12, normalOffsetM: -0.12 }),
    ]),
  }),
  insane: Object.freeze({
    id: "insane",
    stageId: "flutter",
    title: "Insane",
    subtitle: "Fast 22 / controlled chaos",
    description: "Fast 22は維持。即死より追従制御を主難度にする。",
    presetId: "fast22",
    durationS: 12,
    targetRmsM: 0.110,
    targetTipAngleErrorRad: degToRad(28),
    failRmsM: 0.55,
    failTipAngleErrorRad: degToRad(125),
    failHoldS: 0.90,
    minInsideFraction: 0.40,
    effortBudgetJ: 2.00,
    aimDistanceM: 0.52,
    aimRadiusM: 0.13,
    minHitFraction: 0.32,
    authorityScale: 1.18,
    aimSchedule: Object.freeze([
      Object.freeze({ timeS: 0, normalOffsetM: 0.16 }),
      Object.freeze({ timeS: 5.0, normalOffsetM: 0.16 }),
      Object.freeze({ timeS: 6.6, normalOffsetM: -0.16 }),
      Object.freeze({ timeS: 12, normalOffsetM: -0.16 }),
    ]),
  }),
});

export function gameStageById(id) {
  const stage = SHOWER_GAME_STAGES[id];
  if (!stage) throw new RangeError(`unknown shower game stage: ${id}`);
  return stage;
}

export function gameDifficultyById(id) {
  const difficulty = SHOWER_GAME_DIFFICULTIES[id];
  if (!difficulty) {
    throw new RangeError(`unknown shower game difficulty: ${id}`);
  }
  return difficulty;
}

export function wrapAngleRad(angleRad) {
  return Math.atan2(Math.sin(angleRad), Math.cos(angleRad));
}

function clamp01(value) {
  return Math.min(1, Math.max(0, value));
}

function normalizedDirection2(direction) {
  if (!Array.isArray(direction) || direction.length !== 2) {
    throw new RangeError("direction must be a 2D vector");
  }
  const norm = Math.hypot(direction[0], direction[1]);
  if (!(norm > 1e-12)) {
    throw new RangeError("direction must be non-zero");
  }
  return [direction[0] / norm, direction[1] / norm];
}

function smoothstep01(value) {
  const x = clamp01(value);
  return x * x * (3 - 2 * x);
}

export function difficultyAimOffsetM(
  difficultyInput,
  timeS,
) {
  const difficulty = typeof difficultyInput === "string"
    ? gameDifficultyById(difficultyInput)
    : difficultyInput;
  const schedule = difficulty?.aimSchedule;
  if (!Array.isArray(schedule) || schedule.length === 0) {
    return difficulty?.aimNormalOffsetM ?? 0;
  }
  if (timeS <= schedule[0].timeS) {
    return schedule[0].normalOffsetM;
  }

  for (let i = 1; i < schedule.length; i += 1) {
    const next = schedule[i];
    const previous = schedule[i - 1];
    if (timeS <= next.timeS) {
      const duration = Math.max(
        1e-9,
        next.timeS - previous.timeS,
      );
      const ratio = (timeS - previous.timeS) / duration;
      const blend = smoothstep01(ratio);
      return previous.normalOffsetM
        + blend * (
          next.normalOffsetM - previous.normalOffsetM
        );
    }
  }

  return schedule.at(-1).normalOffsetM;
}

export function difficultyAimMaxSpeedMps(difficultyInput) {
  const difficulty = typeof difficultyInput === "string"
    ? gameDifficultyById(difficultyInput)
    : difficultyInput;
  const schedule = difficulty?.aimSchedule;
  if (!Array.isArray(schedule) || schedule.length < 2) {
    return 0;
  }
  let maxSpeed = 0;
  for (let i = 1; i < schedule.length; i += 1) {
    const previous = schedule[i - 1];
    const next = schedule[i];
    const duration = next.timeS - previous.timeS;
    if (!(duration > 0)) {
      throw new RangeError(
        "aimSchedule times must be strictly increasing",
      );
    }
    // smoothstep derivative max is 1.5 at t=0.5.
    maxSpeed = Math.max(
      maxSpeed,
      1.5 * Math.abs(
        next.normalOffsetM - previous.normalOffsetM,
      ) / duration,
    );
  }
  return maxSpeed;
}

export function createAimTarget({
  nozzleOrigin,
  outletDirection,
  distanceM,
  normalOffsetM = 0,
  radiusM,
}) {
  if (!Array.isArray(nozzleOrigin) || nozzleOrigin.length !== 2) {
    throw new RangeError("nozzleOrigin must be a 2D point");
  }
  if (!(distanceM > 0) || !(radiusM > 0)) {
    throw new RangeError("aim distance and radius must be positive");
  }
  const direction = normalizedDirection2(outletDirection);
  const normal = [-direction[1], direction[0]];
  return {
    center: [
      nozzleOrigin[0]
        + distanceM * direction[0]
        + normalOffsetM * normal[0],
      nozzleOrigin[1]
        + distanceM * direction[1]
        + normalOffsetM * normal[1],
    ],
    radiusM,
    referenceDistanceM: distanceM,
    normalOffsetM,
    maxRayDistanceM: distanceM + 0.08,
  };
}

export function evaluateWaterAim({
  nozzleOrigin,
  outletDirection,
  target,
}) {
  if (!target?.center || !(target.radiusM > 0)) {
    return {
      hit: false,
      forwardDistanceM: 0,
      missDistanceM: Infinity,
      aimQuality: 0,
      closestPoint: [...nozzleOrigin],
    };
  }

  const direction = normalizedDirection2(outletDirection);
  const dx = target.center[0] - nozzleOrigin[0];
  const dy = target.center[1] - nozzleOrigin[1];
  const forwardDistanceM = dx * direction[0] + dy * direction[1];
  const maxRayDistanceM = target.maxRayDistanceM ?? Infinity;
  const clampedForward = Math.min(
    maxRayDistanceM,
    Math.max(0, forwardDistanceM),
  );
  const closestPoint = [
    nozzleOrigin[0] + clampedForward * direction[0],
    nozzleOrigin[1] + clampedForward * direction[1],
  ];
  const missDistanceM = Math.hypot(
    target.center[0] - closestPoint[0],
    target.center[1] - closestPoint[1],
  );
  const inFront = forwardDistanceM > 0;
  const inRange = forwardDistanceM <= maxRayDistanceM;
  const hit = inFront
    && inRange
    && missDistanceM <= target.radiusM;

  return {
    hit,
    forwardDistanceM,
    missDistanceM,
    closestPoint,
    aimQuality: clamp01(
      1 - missDistanceM / (2 * target.radiusM),
    ),
  };
}

function createState(config, {
  stageId = config.id,
  difficultyId = null,
} = {}) {
  return {
    stageId,
    difficultyId,
    status: "running",
    elapsedS: 0,
    insideTargetS: 0,
    dangerHoldS: 0,
    qualityIntegral: 0,
    effortJ: 0,
    netWorkJ: 0,
    saturationS: 0,
    aimHitS: 0,
    aimHitFraction: 0,
    aimQualityIntegral: 0,
    missDistanceIntegralM: 0,
    meanMissDistanceM: 0,
    score: 0,
    insideFraction: 0,
    failureReason: null,
    stage: config,
  };
}

export function createGameState(stageId) {
  return createState(gameStageById(stageId), { stageId });
}

export function createDifficultyGameState(difficultyId) {
  const difficulty = gameDifficultyById(difficultyId);
  return createState(difficulty, {
    stageId: difficulty.stageId,
    difficultyId,
  });
}

export function evaluateGameSample(stage, sample) {
  const angleError = Math.abs(wrapAngleRad(
    sample.tipAngleErrorRad,
  ));
  const rmsRatio = sample.rmsM / stage.targetRmsM;
  const angleRatio = angleError / stage.targetTipAngleErrorRad;

  const rmsQuality = clamp01(1 - 0.5 * rmsRatio);
  const angleQuality = clamp01(1 - 0.5 * angleRatio);
  const trackingQuality = 0.6 * rmsQuality + 0.4 * angleQuality;

  return {
    angleErrorRad: angleError,
    insideTarget: sample.rmsM <= stage.targetRmsM
      && angleError <= stage.targetTipAngleErrorRad,
    outsideFailureEnvelope: sample.rmsM > stage.failRmsM
      || angleError > stage.failTipAngleErrorRad,
    trackingQuality,
  };
}

export function scoreGameState(state) {
  const elapsed = Math.max(state.elapsedS, 1e-9);
  const qualityMean = state.qualityIntegral / elapsed;
  const saturationFraction = state.saturationS / elapsed;
  const stage = state.stage;

  const effortOverBudget = Math.max(
    0,
    state.effortJ / stage.effortBudgetJ - 1,
  );

  if (Number.isFinite(stage.minHitFraction)) {
    const aimQualityMean = state.aimQualityIntegral / elapsed;
    return Math.round(1000 * clamp01(
      0.55 * qualityMean
        + 0.25 * aimQualityMean
        + 0.20 * state.aimHitFraction
        - 0.08 * saturationFraction
        - 0.08 * effortOverBudget,
    ));
  }

  return Math.round(1000 * clamp01(
    qualityMean
      - 0.12 * saturationFraction
      - 0.10 * effortOverBudget,
  ));
}

export function updateGameState(stateInput, sample, dt) {
  if (!(dt > 0)) throw new RangeError("dt must be positive");
  if (stateInput.status !== "running") return stateInput;

  const state = {
    ...stateInput,
    stage: stateInput.stage,
  };
  const evaluation = evaluateGameSample(state.stage, sample);

  state.elapsedS = Math.min(
    state.stage.durationS,
    state.elapsedS + dt,
  );
  if (evaluation.insideTarget) {
    state.insideTargetS += dt;
  }

  state.dangerHoldS = evaluation.outsideFailureEnvelope
    ? state.dangerHoldS + dt
    : 0;

  state.qualityIntegral += evaluation.trackingQuality * dt;
  state.effortJ += Math.abs(sample.handPowerW ?? 0) * dt;
  state.netWorkJ += (sample.handPowerW ?? 0) * dt;
  if (sample.actuatorSaturated) {
    state.saturationS += dt;
  }

  if (Number.isFinite(state.stage.minHitFraction)) {
    if (sample.waterHit) {
      state.aimHitS += dt;
    }
    state.aimQualityIntegral += clamp01(
      sample.aimQuality ?? (sample.waterHit ? 1 : 0),
    ) * dt;
    if (Number.isFinite(sample.waterMissDistanceM)) {
      state.missDistanceIntegralM += (
        sample.waterMissDistanceM * dt
      );
    }
  }

  state.insideFraction = state.insideTargetS
    / Math.max(state.elapsedS, 1e-9);
  state.aimHitFraction = state.aimHitS
    / Math.max(state.elapsedS, 1e-9);
  state.meanMissDistanceM = state.missDistanceIntegralM
    / Math.max(state.elapsedS, 1e-9);
  state.score = scoreGameState(state);

  if (state.dangerHoldS >= state.stage.failHoldS) {
    state.status = "failed";
    state.failureReason = "failure envelope";
    return state;
  }

  if (state.elapsedS >= state.stage.durationS - 1e-12) {
    if (state.insideFraction < state.stage.minInsideFraction) {
      state.status = "failed";
      state.failureReason = "target ratio";
    } else if (
      Number.isFinite(state.stage.minHitFraction)
      && state.aimHitFraction < state.stage.minHitFraction
    ) {
      state.status = "failed";
      state.failureReason = "aim ratio";
    } else {
      state.status = "success";
    }
  }

  return state;
}

export function gameHudSnapshot(state) {
  if (!state) {
    return {
      status: "READY",
      timeLabel: "-",
      scoreLabel: "-",
      targetLabel: "-",
      hitLabel: "-",
      effortLabel: "-",
    };
  }

  const remainingS = Math.max(
    0,
    state.stage.durationS - state.elapsedS,
  );
  const hitLabel = Number.isFinite(state.stage.minHitFraction)
    ? `${(100 * state.aimHitFraction).toFixed(0)}% / ${(100 * state.stage.minHitFraction).toFixed(0)}%`
    : "-";
  return {
    status: state.status.toUpperCase(),
    timeLabel: `${remainingS.toFixed(1)} s`,
    scoreLabel: String(state.score),
    targetLabel:
      `${(100 * state.insideFraction).toFixed(0)}% / ${(100 * state.stage.minInsideFraction).toFixed(0)}%`,
    hitLabel,
    effortLabel:
      `${state.effortJ.toFixed(2)} / ${state.stage.effortBudgetJ.toFixed(2)} J`,
  };
}
