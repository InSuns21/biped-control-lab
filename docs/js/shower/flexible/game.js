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

export function gameStageById(id) {
  const stage = SHOWER_GAME_STAGES[id];
  if (!stage) throw new RangeError(`unknown shower game stage: ${id}`);
  return stage;
}

export function wrapAngleRad(angleRad) {
  return Math.atan2(Math.sin(angleRad), Math.cos(angleRad));
}

export function createGameState(stageId) {
  const stage = gameStageById(stageId);
  return {
    stageId,
    status: "running",
    elapsedS: 0,
    insideTargetS: 0,
    dangerHoldS: 0,
    qualityIntegral: 0,
    effortJ: 0,
    netWorkJ: 0,
    saturationS: 0,
    score: 0,
    insideFraction: 0,
    failureReason: null,
    stage,
  };
}

function clamp01(value) {
  return Math.min(1, Math.max(0, value));
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

  state.insideFraction = state.insideTargetS
    / Math.max(state.elapsedS, 1e-9);
  state.score = scoreGameState(state);

  if (state.dangerHoldS >= state.stage.failHoldS) {
    state.status = "failed";
    state.failureReason = "failure envelope";
    return state;
  }

  if (state.elapsedS >= state.stage.durationS - 1e-12) {
    if (state.insideFraction >= state.stage.minInsideFraction) {
      state.status = "success";
    } else {
      state.status = "failed";
      state.failureReason = "target ratio";
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
      effortLabel: "-",
    };
  }

  const remainingS = Math.max(
    0,
    state.stage.durationS - state.elapsedS,
  );
  return {
    status: state.status.toUpperCase(),
    timeLabel: `${remainingS.toFixed(1)} s`,
    scoreLabel: String(state.score),
    targetLabel:
      `${(100 * state.insideFraction).toFixed(0)}% / ${(100 * state.stage.minInsideFraction).toFixed(0)}%`,
    effortLabel:
      `${state.effortJ.toFixed(2)} / ${state.stage.effortBudgetJ.toFixed(2)} J`,
  };
}
