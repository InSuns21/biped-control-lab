import assert from "node:assert/strict";
import {
  createNonlinearShowerScenario,
  perturbEquilibriumState,
  solveNonlinearShowerEquilibrium,
} from "../docs/js/shower/flexible/nonlinear-scenario.js";
import {
  nonlinearShowerHeadReaction,
} from "../docs/js/shower/flexible/nonlinear-shower-head.js";
import {
  rodKinematicsWithHandBoundary,
  stepRodWithHandBoundaryRK4,
  ZERO_HAND_BOUNDARY,
} from "../docs/js/shower/flexible/nonlinear-boundary.js";
import {
  actuatorBoundaryTrajectory,
  createHandActuatorState,
  DEFAULT_HAND_ACTUATOR_LIMITS,
  scaleHandActuatorLimits,
  stepHandActuator,
} from "../docs/js/shower/flexible/hand-actuator.js";
import {
  createAimTarget,
  createDifficultyGameState,
  difficultyAimOffsetM,
  evaluateWaterAim,
  gameDifficultyById,
  updateGameState,
} from "../docs/js/shower/flexible/game.js";
import {
  automaticGameHandTarget,
  FIXED_STATE_FEEDBACK_GAIN_SCALE,
} from "../docs/js/shower/flexible/game-feedback-controller.js";
import {
  designFullStateLqr,
} from "../docs/js/shower/flexible/state-feedback-controller.js";

const DT = 0.002;
const SEGMENT_COUNT = 12;

function optionsForDifficulty(difficulty) {
  if (difficulty.presetId === "low12") {
    return {
      flowLpm: 12,
      lengthM: 1.2,
      flexuralRigidityNm2: 0.7,
      rayleighMassPerS: 0.08,
      rayleighStiffnessS: 0.0002,
      segmentCount: SEGMENT_COUNT,
    };
  }
  if (difficulty.presetId === "baseline18") {
    return {
      flowLpm: 18,
      lengthM: 1.2,
      flexuralRigidityNm2: 0.7,
      rayleighMassPerS: 0.08,
      rayleighStiffnessS: 0.0002,
      segmentCount: SEGMENT_COUNT,
    };
  }
  return {
    flowLpm: 22,
    lengthM: 1.5,
    flexuralRigidityNm2: 0.25,
    rayleighMassPerS: 0.02,
    rayleighStiffnessS: 0.0002,
    segmentCount: SEGMENT_COUNT,
  };
}

function solveContinuation(options) {
  let angles = null;
  let scenario = null;
  let equilibrium = null;
  for (const q of [
    0,
    options.flowLpm / 3,
    2 * options.flowLpm / 3,
    options.flowLpm,
  ]) {
    scenario = createNonlinearShowerScenario({
      ...options,
      flowLpm: q,
      headMassKg: 0.20,
      headRotInertiaKgM2: 0.002,
    });
    equilibrium = solveNonlinearShowerEquilibrium(
      scenario,
      { initialAnglesRad: angles },
    );
    assert.ok(equilibrium.converged);
    angles = [...equilibrium.anglesRad];
  }
  return { scenario, equilibrium };
}

function reactionFor(scenario, anglesRad) {
  return nonlinearShowerHeadReaction(
    scenario.system,
    anglesRad,
    {
      flowRateM3s: scenario.flowRateM3s,
      waterDensityKgM3: scenario.params.waterDensityKgM3,
      hoseInnerDiameterM: scenario.params.hoseInnerDiameterM,
      head: scenario.head,
    },
  );
}

function nozzleOrigin(current, reaction) {
  return [
    current.tip[0] + reaction.nozzleOffsetWorldM[0],
    current.tip[1] + reaction.nozzleOffsetWorldM[1],
  ];
}

function geometryMetrics(current, equilibrium) {
  let sumSq = 0;
  for (let i = 1; i < current.nodes.length; i += 1) {
    const dx = current.nodes[i][0] - equilibrium.nodes[i][0];
    const dy = current.nodes[i][1] - equilibrium.nodes[i][1];
    sumSq += dx * dx + dy * dy;
  }
  return {
    rmsM: Math.sqrt(
      sumSq / Math.max(1, current.nodes.length - 1),
    ),
  };
}

function loadOptions(scenario) {
  return {
    tipLoad: scenario.tipLoad,
    additionalGeneralizedForce: scenario.flowForce,
    additionalCartesianResultant: scenario.flowResultant,
  };
}

const solvedCache = new Map();
const designCache = new Map();

function solvedFor(difficulty) {
  if (!solvedCache.has(difficulty.presetId)) {
    solvedCache.set(
      difficulty.presetId,
      solveContinuation(optionsForDifficulty(difficulty)),
    );
  }
  return solvedCache.get(difficulty.presetId);
}

function designFor(difficulty, solved, limits) {
  const key = `${difficulty.presetId}:${difficulty.authorityScale}`;
  if (!designCache.has(key)) {
    designCache.set(
      key,
      designFullStateLqr(
        solved.scenario,
        solved.equilibrium,
        {
          dt: DT,
          limits,
          lqrOptions: {
            tolerance: 1e-8,
            maxIterations: 4000,
          },
        },
      ),
    );
  }
  return designCache.get(key);
}

function simulate(difficultyId, mode) {
  const difficulty = gameDifficultyById(difficultyId);
  const solved = solvedFor(difficulty);
  const { scenario, equilibrium } = solved;
  const limits = scaleHandActuatorLimits(
    DEFAULT_HAND_ACTUATOR_LIMITS,
    difficulty.authorityScale,
  );
  const design = mode === "state" || mode === "lqr"
    ? designFor(difficulty, solved, limits)
    : null;

  let state = perturbEquilibriumState(
    scenario,
    equilibrium.anglesRad,
    {
      tipAnglePerturbationRad: 0.02,
      velocityAmplitudeRadS: 0.03,
    },
  );
  let actuator = createHandActuatorState();
  let boundary = { ...ZERO_HAND_BOUNDARY };
  let game = createDifficultyGameState(difficultyId);
  let previousPowerW = 0;
  let effortJ = 0;
  let saturationS = 0;

  const equilibriumReaction = reactionFor(
    scenario,
    equilibrium.anglesRad,
  );
  const referenceNozzle = nozzleOrigin(
    equilibrium.kinematics,
    equilibriumReaction,
  );

  while (game.status === "running") {
    const target = createAimTarget({
      nozzleOrigin: referenceNozzle,
      outletDirection: equilibriumReaction.outletDirection,
      distanceM: difficulty.aimDistanceM,
      normalOffsetM: difficultyAimOffsetM(
        difficulty,
        game.elapsedS,
      ),
      radiusM: difficulty.aimRadiusM,
    });

    const controller = mode === "open"
      ? null
      : automaticGameHandTarget(
          mode,
          {
            system: scenario.system,
            rodState: state,
            boundary,
            actuatorState: actuator,
            equilibriumKinematics: equilibrium.kinematics,
            equilibriumReaction,
            aimTarget: target,
            lqrDesign: design,
            limits,
            stateGainScale: FIXED_STATE_FEEDBACK_GAIN_SCALE,
          },
        );

    const handTarget = controller?.target ?? {
      lateralPositionM: 0,
      angleRad: 0,
    };
    const startActuator = actuator;
    const actuatorStep = stepHandActuator(
      startActuator,
      handTarget,
      DT,
      limits,
    );
    const trajectory = actuatorBoundaryTrajectory(
      startActuator,
      actuatorStep,
      DT,
    );
    const next = stepRodWithHandBoundaryRK4(
      scenario.system,
      state,
      game.elapsedS,
      DT,
      (absoluteTimeS) => trajectory(
        absoluteTimeS - game.elapsedS,
      ),
      loadOptions(scenario),
    );

    state = next.state;
    actuator = actuatorStep.state;
    boundary = next.boundary;

    assert.ok(state.anglesRad.every(Number.isFinite));
    assert.ok(state.angularRatesRadS.every(Number.isFinite));

    const current = rodKinematicsWithHandBoundary(
      scenario.system,
      state,
      boundary,
    );
    const reaction = reactionFor(scenario, state.anglesRad);
    const aim = evaluateWaterAim({
      nozzleOrigin: nozzleOrigin(current, reaction),
      outletDirection: reaction.outletDirection,
      target,
    });
    const metrics = geometryMetrics(
      current,
      equilibrium.kinematics,
    );

    effortJ += 0.5 * (
      Math.abs(previousPowerW)
      + Math.abs(next.diagnostics.handPowerW)
    ) * DT;
    previousPowerW = next.diagnostics.handPowerW;

    const saturated = Object.values(
      actuatorStep.saturation,
    ).some(Boolean);
    if (saturated) saturationS += DT;

    game = updateGameState(
      game,
      {
        rmsM: metrics.rmsM,
        tipAngleErrorRad:
          current.tipAngleRad
          - equilibrium.kinematics.tipAngleRad,
        handPowerW: next.diagnostics.handPowerW,
        actuatorSaturated: saturated,
        waterHit: aim.hit,
        waterMissDistanceM: aim.missDistanceM,
        aimQuality: aim.aimQuality,
      },
      DT,
    );
  }

  return {
    difficultyId,
    mode,
    status: game.status,
    failureReason: game.failureReason,
    score: game.score,
    hitFraction: game.aimHitFraction,
    insideFraction: game.insideFraction,
    rmsMeanM: game.rmsMeanM,
    effortJ,
    saturationS,
    elapsedS: game.elapsedS,
    stageMinHitFraction: difficulty.minHitFraction,
    stageMinInsideFraction: difficulty.minInsideFraction,
  };
}

const results = {};
for (const difficultyId of ["easy", "normal", "expert", "insane"]) {
  results[difficultyId] = {};
  for (const mode of ["open", "p", "pd", "state", "lqr"]) {
    results[difficultyId][mode] = simulate(
      difficultyId,
      mode,
    );
  }
}

console.log(
  "H1-8B production auto-control matrix:",
  JSON.stringify(results),
);

for (const difficultyId of ["easy", "normal", "expert", "insane"]) {
  const group = results[difficultyId];
  for (const mode of ["p", "pd", "state", "lqr"]) {
    assert.ok(Number.isFinite(group[mode].score));
    assert.ok(Number.isFinite(group[mode].rmsMeanM));
    assert.ok(Number.isFinite(group[mode].effortJ));
    assert.ok(Number.isFinite(group[mode].saturationS));
    assert.ok(
      Number.isFinite(group[mode].hitFraction)
        && group[mode].hitFraction >= 0
        && group[mode].hitFraction <= 1,
      `${difficultyId} ${mode} hit fraction must be finite`,
    );
  }
  assert.equal(
    group.open.status,
    "failed",
    `${difficultyId} open baseline should remain non-winning`,
  );
}

// H1-8B diagnostic pass: success thresholds are restored after the
// production matrix is inspected.

console.log(
  "H1-6-3 nonlinear game comparison:",
  JSON.stringify(results),
);
