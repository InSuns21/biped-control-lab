import assert from "node:assert/strict";
import {
  createNonlinearShowerScenario,
  perturbEquilibriumState,
  solveNonlinearShowerEquilibrium,
} from "../docs/js/shower/flexible/nonlinear-scenario.js";
import {
  rodKinematicsWithHandBoundary,
  stepRodWithHandBoundaryRK4,
  ZERO_HAND_BOUNDARY,
} from "../docs/js/shower/flexible/nonlinear-boundary.js";
import {
  actuatorBoundaryTrajectory,
  createHandActuatorState,
  DEFAULT_HAND_ACTUATOR_LIMITS,
  stepHandActuator,
} from "../docs/js/shower/flexible/hand-actuator.js";
import {
  pdHandTarget,
  senseTipFeedback,
} from "../docs/js/shower/flexible/feedback-controller.js";

const DT = 0.002;

function optionsFor(kind) {
  return kind === "fast22"
    ? {
        flowLpm: 22,
        lengthM: 1.5,
        flexuralRigidityNm2: 0.25,
        rayleighMassPerS: 0.02,
        rayleighStiffnessS: 0.0002,
      }
    : {
        flowLpm: 18,
        lengthM: 1.2,
        flexuralRigidityNm2: 0.7,
        rayleighMassPerS: 0.08,
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

function rms(current, equilibrium) {
  let sumSq = 0;
  for (let i = 1; i < current.nodes.length; i += 1) {
    const dx = current.nodes[i][0] - equilibrium.nodes[i][0];
    const dy = current.nodes[i][1] - equilibrium.nodes[i][1];
    sumSq += dx * dx + dy * dy;
  }
  return Math.sqrt(sumSq / (current.nodes.length - 1));
}

function loadOptions(scenario) {
  return {
    tipLoad: scenario.tipLoad,
    additionalGeneralizedForce: scenario.flowForce,
    additionalCartesianResultant: scenario.flowResultant,
  };
}

function simulate(solved, gains, durationS) {
  const { scenario, equilibrium } = solved;
  let state = perturbEquilibriumState(
    scenario,
    equilibrium.anglesRad,
    {
      tipAnglePerturbationRad: 0.035,
      velocityAmplitudeRadS: 0.05,
    },
  );
  let actuator = createHandActuatorState();
  let boundary = { ...ZERO_HAND_BOUNDARY };
  let integral = 0;
  let maxRms = 0;
  let saturationS = 0;

  for (
    let step = 0;
    step < Math.round(durationS / DT);
    step += 1
  ) {
    const timeS = step * DT;
    const sensing = senseTipFeedback(
      scenario.system,
      state,
      boundary,
      equilibrium.kinematics,
    );
    const target = gains
      ? pdHandTarget(
          sensing,
          gains,
          DEFAULT_HAND_ACTUATOR_LIMITS,
        )
      : { lateralPositionM: 0, angleRad: 0 };

    const start = actuator;
    const actuatorStep = stepHandActuator(
      start,
      target,
      DT,
      DEFAULT_HAND_ACTUATOR_LIMITS,
    );
    const trajectory = actuatorBoundaryTrajectory(
      start,
      actuatorStep,
      DT,
    );
    const result = stepRodWithHandBoundaryRK4(
      scenario.system,
      state,
      timeS,
      DT,
      (absoluteTimeS) => trajectory(
        absoluteTimeS - timeS,
      ),
      loadOptions(scenario),
    );
    actuator = actuatorStep.state;
    boundary = result.boundary;
    state = result.state;

    if (!state.anglesRad.every(Number.isFinite)) {
      return { finite: false, integral: Infinity, maxRms: Infinity };
    }

    const value = rms(
      rodKinematicsWithHandBoundary(
        scenario.system,
        state,
        boundary,
      ),
      equilibrium.kinematics,
    );
    integral += value * DT;
    maxRms = Math.max(maxRms, value);
    if (Object.values(actuatorStep.saturation).some(Boolean)) {
      saturationS += DT;
    }
  }

  return {
    finite: true,
    integral,
    mean: integral / durationS,
    maxRms,
    saturationS,
  };
}

const baselineSolved = solveContinuation(optionsFor("baseline18"));
const fastSolved = solveContinuation(optionsFor("fast22"));

const openBaseline = simulate(baselineSolved, null, 3);
const openFast = simulate(fastSolved, null, 3);

const positionGains = [-0.15, -0.35];
const angleGains = [-0.15, -0.35];
const velocityGains = [-0.03, -0.08];
const angularRateGains = [-0.03, -0.08];

const candidates = [];
for (const lateralPositionGain of positionGains) {
  for (const angleGain of angleGains) {
    for (const lateralVelocityGainS of velocityGains) {
      for (const angularRateGainS of angularRateGains) {
        candidates.push({
          lateralPositionGain,
          lateralVelocityGainS,
          angleGain,
          angularRateGainS,
        });
      }
    }
  }
}

const results = candidates.map((gains) => {
  const baseline = simulate(baselineSolved, gains, 3);
  const fast = simulate(fastSolved, gains, 3);
  const score =
    baseline.integral / openBaseline.integral
    + fast.integral / openFast.integral
    + 0.05 * (
      baseline.saturationS / 3
      + fast.saturationS / 3
    );
  return { gains, baseline, fast, score };
});

results.sort((a, b) => a.score - b.score);
const best = results.slice(0, 12);

console.log(
  "H1-6 gain sweep:",
  JSON.stringify({
    openBaseline,
    openFast,
    best,
  }),
);
