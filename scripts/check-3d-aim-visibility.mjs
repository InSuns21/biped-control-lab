import assert from "node:assert/strict";
import * as THREE from "../docs/vendor/three.module.js";
import {
  createNonlinearShowerScenario,
  solveNonlinearShowerEquilibrium,
} from "../docs/js/shower/flexible/nonlinear-scenario.js";
import {
  nonlinearShowerHeadReaction,
} from "../docs/js/shower/flexible/nonlinear-shower-head.js";
import {
  createAimTarget,
  difficultyAimOffsetM,
  gameDifficultyById,
} from "../docs/js/shower/flexible/game.js";
import {
  DEFAULT_GAME3D_LAYOUT,
  GAME3D_CAMERA_VIEWS,
  mapRodPointToGame3D,
} from "../docs/labs/x1-shower-tvc/game-3d-view.js";

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
      segmentCount: 10,
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

function reactionAtEquilibrium(scenario, equilibrium) {
  return nonlinearShowerHeadReaction(
    scenario.system,
    equilibrium.anglesRad,
    {
      flowRateM3s: scenario.flowRateM3s,
      waterDensityKgM3: scenario.params.waterDensityKgM3,
      hoseInnerDiameterM: scenario.params.hoseInnerDiameterM,
      head: scenario.head,
    },
  );
}

function makeCamera(config, aspect) {
  const camera = new THREE.PerspectiveCamera(
    43,
    aspect,
    0.02,
    12,
  );
  camera.position.set(...config.position);
  camera.up.set(0, 1, 0);
  camera.lookAt(new THREE.Vector3(...config.target));
  camera.updateMatrixWorld(true);
  camera.updateProjectionMatrix();
  return camera;
}

function ndcFor(worldPoint, config, aspect) {
  const camera = makeCamera(config, aspect);
  return new THREE.Vector3(...worldPoint).project(camera);
}

const solvedByPreset = new Map();
const results = {};
const aspects = {
  desktop: 900 / 640,
  tablet: 620 / 440,
};

for (const id of ["easy", "normal", "expert", "insane"]) {
  const difficulty = gameDifficultyById(id);
  if (!solvedByPreset.has(difficulty.presetId)) {
    solvedByPreset.set(
      difficulty.presetId,
      solveContinuation(scenarioOptionsForDifficulty(difficulty)),
    );
  }
  const { scenario, equilibrium } = solvedByPreset.get(
    difficulty.presetId,
  );
  const reaction = reactionAtEquilibrium(scenario, equilibrium);
  const nozzleOrigin = [
    equilibrium.kinematics.tip[0]
      + reaction.nozzleOffsetWorldM[0],
    equilibrium.kinematics.tip[1]
      + reaction.nozzleOffsetWorldM[1],
  ];

  const sampleTimes = [
    0,
    ...difficulty.aimSchedule.map((entry) => entry.timeS),
    difficulty.durationS,
  ];
  const uniqueTimes = [...new Set(sampleTimes)];
  results[id] = [];

  for (const timeS of uniqueTimes) {
    const target = createAimTarget({
      nozzleOrigin,
      outletDirection: reaction.outletDirection,
      distanceM: difficulty.aimDistanceM,
      normalOffsetM: difficultyAimOffsetM(difficulty, timeS),
      radiusM: difficulty.aimRadiusM,
    });
    const worldPoint = mapRodPointToGame3D(
      target.center,
      DEFAULT_GAME3D_LAYOUT,
    );
    const projections = {};

    for (const [viewName, config] of Object.entries(
      GAME3D_CAMERA_VIEWS,
    )) {
      projections[viewName] = {};
      for (const [aspectName, aspect] of Object.entries(aspects)) {
        const ndc = ndcFor(worldPoint, config, aspect);
        projections[viewName][aspectName] = {
          x: ndc.x,
          y: ndc.y,
          z: ndc.z,
        };
        assert.ok(
          Math.abs(ndc.x) <= 0.92
            && Math.abs(ndc.y) <= 0.92
            && ndc.z >= -1
            && ndc.z <= 1,
          `${id} t=${timeS}s bullseye must stay visible in ${viewName}/${aspectName}`,
        );
      }
    }

    results[id].push({
      timeS,
      targetCenter: target.center,
      worldPoint,
      projections,
    });
  }
}

console.log(
  "H1-5-4A 3D aim visibility OK:",
  JSON.stringify(results),
);
