import assert from "node:assert/strict";
import {
  createNonlinearShowerScenario,
  perturbEquilibriumState,
  solveNonlinearShowerEquilibrium,
} from "../docs/js/shower/flexible/nonlinear-scenario.js";
import {
  rodKinematics,
  stepNonlinearRodRK4,
} from "../docs/js/shower/flexible/nonlinear-rod.js";
import {
  nonlinearShowerHeadReaction,
} from "../docs/js/shower/flexible/nonlinear-shower-head.js";
import {
  createAimTarget,
  createDifficultyGameState,
  difficultyAimOffsetM,
  evaluateWaterAim,
  gameDifficultyById,
  updateGameState,
} from "../docs/js/shower/flexible/game.js";

const DT = 0.004;

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

function solveContinuation({
  flowLpm,
  lengthM,
  flexuralRigidityNm2,
  rayleighMassPerS,
  rayleighStiffnessS,
}) {
  let angles = null;
  let scenario = null;
  let equilibrium = null;
  for (const q of [0, flowLpm / 3, 2 * flowLpm / 3, flowLpm]) {
    scenario = createNonlinearShowerScenario({
      segmentCount: 10,
      flowLpm: q,
      lengthM,
      flexuralRigidityNm2,
      rayleighMassPerS,
      rayleighStiffnessS,
      headMassKg: 0.20,
      headRotInertiaKgM2: 0.002,
    });
    equilibrium = solveNonlinearShowerEquilibrium(
      scenario,
      { initialAnglesRad: angles },
    );
    assert.ok(
      equilibrium.converged,
      `equilibrium must converge at Q=${q}`,
    );
    angles = [...equilibrium.anglesRad];
  }
  return { scenario, equilibrium };
}

function scenarioOptionsForDifficulty(difficulty) {
  if (difficulty.presetId === "low12") {
    return {
      flowLpm: 12,
      lengthM: 1.2,
      flexuralRigidityNm2: 0.7,
      rayleighMassPerS: 0.08,
      rayleighStiffnessS: 0.0002,
    };
  }
  if (difficulty.presetId === "baseline18") {
    return {
      flowLpm: 18,
      lengthM: 1.2,
      flexuralRigidityNm2: 0.7,
      rayleighMassPerS: 0.08,
      rayleighStiffnessS: 0.0002,
    };
  }
  return {
    flowLpm: 22,
    lengthM: 1.5,
    flexuralRigidityNm2: 0.25,
    rayleighMassPerS: 0.02,
    rayleighStiffnessS: 0.0002,
  };
}

function headReaction(scenario, anglesRad) {
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

function nozzleOrigin(kinematics, reaction) {
  return [
    kinematics.tip[0] + reaction.nozzleOffsetWorldM[0],
    kinematics.tip[1] + reaction.nozzleOffsetWorldM[1],
  ];
}

const solvedByPreset = new Map();

function getSolved(difficulty) {
  if (!solvedByPreset.has(difficulty.presetId)) {
    solvedByPreset.set(
      difficulty.presetId,
      solveContinuation(
        scenarioOptionsForDifficulty(difficulty),
      ),
    );
  }
  return solvedByPreset.get(difficulty.presetId);
}

const results = {};

for (const id of ["easy", "normal", "expert", "insane"]) {
  const difficulty = gameDifficultyById(id);
  const { scenario, equilibrium } = getSolved(difficulty);
  const equilibriumReaction = headReaction(
    scenario,
    equilibrium.anglesRad,
  );
  const referenceOrigin = nozzleOrigin(
    equilibrium.kinematics,
    equilibriumReaction,
  );
  const referenceDirection = equilibriumReaction.outletDirection;

  let state = perturbEquilibriumState(
    scenario,
    equilibrium.anglesRad,
    {
      tipAnglePerturbationRad: 0.02,
      velocityAmplitudeRadS: 0.03,
    },
  );
  let game = createDifficultyGameState(id);

  while (game.status === "running") {
    state = stepNonlinearRodRK4(
      scenario.system,
      state,
      DT,
      scenario.tipLoad,
      scenario.flowForce,
    );

    const current = rodKinematics(
      scenario.system,
      state.anglesRad,
    );
    const reaction = headReaction(
      scenario,
      state.anglesRad,
    );
    const target = createAimTarget({
      nozzleOrigin: referenceOrigin,
      outletDirection: referenceDirection,
      distanceM: difficulty.aimDistanceM,
      normalOffsetM: difficultyAimOffsetM(
        difficulty,
        game.elapsedS,
      ),
      radiusM: difficulty.aimRadiusM,
    });
    const aim = evaluateWaterAim({
      nozzleOrigin: nozzleOrigin(current, reaction),
      outletDirection: reaction.outletDirection,
      target,
    });
    const metrics = geometryMetrics(
      current,
      equilibrium.kinematics,
    );

    game = updateGameState(
      game,
      {
        rmsM: metrics.rmsM,
        tipAngleErrorRad:
          current.tipAngleRad
          - equilibrium.kinematics.tipAngleRad,
        handPowerW: 0,
        actuatorSaturated: false,
        waterHit: aim.hit,
        waterMissDistanceM: aim.missDistanceM,
        aimQuality: aim.aimQuality,
      },
      DT,
    );
  }

  results[id] = {
    status: game.status,
    failureReason: game.failureReason,
    hitFraction: game.aimHitFraction,
    insideFraction: game.insideFraction,
    score: game.score,
  };

  assert.equal(
    game.status,
    "failed",
    `${id}: real nonlinear zero-input run must not succeed`,
  );
  assert.equal(
    game.failureReason,
    "aim ratio",
    `${id}: calibrated zero-input run should survive stability but fail aiming`,
  );
}

console.log(
  "H1-5-4A nonlinear zero-input playability OK:",
  JSON.stringify(results),
);
