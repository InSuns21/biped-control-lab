import assert from "node:assert/strict";
import {
  createNonlinearShowerScenario,
  perturbEquilibriumState,
  solveNonlinearShowerEquilibrium,
} from "../docs/js/shower/flexible/nonlinear-scenario.js";
import {
  actuatorBoundaryTrajectory,
  createHandActuatorState,
  DEFAULT_HAND_ACTUATOR_LIMITS,
  stepHandActuator,
} from "../docs/js/shower/flexible/hand-actuator.js";
import {
  rodKinematicsWithHandBoundary,
  stepRodWithHandBoundaryRK4,
  ZERO_HAND_BOUNDARY,
} from "../docs/js/shower/flexible/nonlinear-boundary.js";
import {
  controllabilityDiagnostics,
  createFullStateDescriptor,
  decodeFullStateDeviation,
  designFullStateLqr,
  encodeFullStateDeviation,
  fullStateOneStep,
  linearizeFullStateOneStep,
  linearOneStepPrediction,
  lqrHandTarget,
  realizationDifferenceMetrics,
} from "../docs/js/shower/flexible/state-feedback-controller.js";

const DT = 0.002;

function optionsFor(kind, segmentCount = 8) {
  return kind === "fast22"
    ? {
        flowLpm: 22,
        lengthM: 1.5,
        flexuralRigidityNm2: 0.25,
        rayleighMassPerS: 0.02,
        rayleighStiffnessS: 0.0002,
        segmentCount,
      }
    : {
        flowLpm: 18,
        lengthM: 1.2,
        flexuralRigidityNm2: 0.7,
        rayleighMassPerS: 0.08,
        rayleighStiffnessS: 0.0002,
        segmentCount,
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

function maxAbs(values) {
  return Math.max(...values.map(Math.abs));
}

function vectorDiff(a, b) {
  return a.map((value, i) => value - b[i]);
}

function vectorNorm(values) {
  return Math.sqrt(values.reduce(
    (sum, value) => sum + value * value,
    0,
  ));
}

function rmsFromEquilibrium(current, equilibrium) {
  let sumSq = 0;
  for (let i = 1; i < current.nodes.length; i += 1) {
    const dx = current.nodes[i][0] - equilibrium.nodes[i][0];
    const dy = current.nodes[i][1] - equilibrium.nodes[i][1];
    sumSq += dx * dx + dy * dy;
  }
  return Math.sqrt(
    sumSq / Math.max(1, current.nodes.length - 1),
  );
}

function loadOptions(scenario) {
  return {
    tipLoad: scenario.tipLoad,
    additionalGeneralizedForce: scenario.flowForce,
    additionalCartesianResultant: scenario.flowResultant,
  };
}

const baselineSolved = solveContinuation(
  optionsFor("baseline18", 6),
);

// H1-6-2A: state representation round-trip.
{
  const { scenario, equilibrium } = baselineSolved;
  const descriptor = createFullStateDescriptor(
    scenario.system,
    equilibrium.anglesRad,
  );
  assert.equal(
    descriptor.dimension,
    2 * (scenario.system.params.segmentCount - 1) + 4,
  );

  const rodState = perturbEquilibriumState(
    scenario,
    equilibrium.anglesRad,
    {
      tipAnglePerturbationRad: 0.012,
      velocityAmplitudeRadS: 0.025,
    },
  );
  const actuator = createHandActuatorState({
    lateralPositionM: 0.014,
    lateralVelocityMps: -0.032,
    angleRad: 0.055,
    angularRateRadS: 0.18,
  });
  rodState.anglesRad[0] = actuator.angleRad;
  rodState.angularRatesRadS[0] = actuator.angularRateRadS;

  const encoded = encodeFullStateDeviation(
    descriptor,
    rodState,
    actuator,
  );
  const decoded = decodeFullStateDeviation(
    descriptor,
    encoded,
  );
  const encodedAgain = encodeFullStateDeviation(
    descriptor,
    decoded.rodState,
    decoded.actuatorState,
  );

  assert.ok(
    maxAbs(vectorDiff(encodedAgain, encoded)) < 1e-14,
    "state encode/decode must round-trip",
  );
}

// H1-6-2A/B/C: production-shaped realization and LQR.
const baselineRealization = linearizeFullStateOneStep(
  baselineSolved.scenario,
  baselineSolved.equilibrium,
  { dt: DT },
);
assert.ok(
  baselineRealization.residualNorm < 5e-7,
  `equilibrium one-step residual too large: ${baselineRealization.residualNorm}`,
);

const bNorm = Math.sqrt(
  baselineRealization.B.flat().reduce(
    (sum, value) => sum + value * value,
    0,
  ),
);
assert.ok(bNorm > 1e-5, "boundary input Jacobian must be non-zero");

const realizationHalf = linearizeFullStateOneStep(
  baselineSolved.scenario,
  baselineSolved.equilibrium,
  {
    dt: DT,
    stateScaleMultiplier: 0.5,
    inputScaleMultiplier: 0.5,
  },
);
const realizationDouble = linearizeFullStateOneStep(
  baselineSolved.scenario,
  baselineSolved.equilibrium,
  {
    dt: DT,
    stateScaleMultiplier: 2,
    inputScaleMultiplier: 2,
  },
);
const halfDifference = realizationDifferenceMetrics(
  baselineRealization,
  realizationHalf,
);
const doubleDifference = realizationDifferenceMetrics(
  baselineRealization,
  realizationDouble,
);
assert.ok(
  halfDifference.aRelative < 2e-3
    && doubleDifference.aRelative < 2e-3,
  "A linearization should be insensitive to finite-difference epsilon",
);
assert.ok(
  halfDifference.bRelative < 2e-3
    && doubleDifference.bRelative < 2e-3,
  "B linearization should be insensitive to finite-difference epsilon",
);

const controllability = controllabilityDiagnostics(
  baselineRealization,
);
assert.ok(
  controllability.rank >= Math.min(
    controllability.dimension,
    0.70 * controllability.dimension,
  ),
  "augmented realization should have a substantial controllable subspace",
);
assert.ok(
  controllability.rodRank >= Math.min(
    controllability.rodDimension,
    0.70 * controllability.rodDimension,
  ),
  "rod state should have a substantial controllable subspace",
);

// Small one-step nonlinear/linear agreement.
{
  const descriptor = baselineRealization.descriptor;
  const x = Array(descriptor.dimension).fill(0);
  for (let i = 0; i < descriptor.freeCount; i += 1) {
    x[i] = 2e-5 * (i + 1) / descriptor.freeCount;
    x[descriptor.freeCount + i] =
      -4e-4 * (i + 1) / descriptor.freeCount;
  }
  const offset = 2 * descriptor.freeCount;
  x[offset] = 5e-6;
  x[offset + 1] = -2e-4;
  x[offset + 2] = 2e-5;
  x[offset + 3] = 5e-4;
  const u = [1e-4, -2e-4];

  const nonlinearBase = fullStateOneStep(
    baselineSolved.scenario,
    descriptor,
    Array(descriptor.dimension).fill(0),
    [0, 0],
    { dt: DT },
  ).stateVector;
  const nonlinear = fullStateOneStep(
    baselineSolved.scenario,
    descriptor,
    x,
    u,
    { dt: DT },
  ).stateVector;
  const nonlinearDelta = vectorDiff(
    nonlinear,
    nonlinearBase,
  );
  const predicted = linearOneStepPrediction(
    baselineRealization,
    x,
    u,
  );
  const error = vectorNorm(
    vectorDiff(nonlinearDelta, predicted),
  );
  const reference = Math.max(
    1e-12,
    vectorNorm(nonlinearDelta),
  );
  assert.ok(
    error / reference < 0.02,
    `linear one-step prediction error too large: ${error / reference}`,
  );
}

const baselineDesign = designFullStateLqr(
  baselineSolved.scenario,
  baselineSolved.equilibrium,
  { dt: DT },
);
assert.ok(baselineDesign.lqr.iterations < 5000);
assert.ok(Number.isFinite(baselineDesign.lqr.maxAbsGain));

function simulate(kind, mode, {
  durationS = 3.5,
  segmentCount = 8,
} = {}) {
  const solved = solveContinuation(
    optionsFor(kind, segmentCount),
  );
  const { scenario, equilibrium } = solved;
  const design = mode === "lqr"
    ? designFullStateLqr(
        scenario,
        equilibrium,
        { dt: DT },
      )
    : null;

  let state = perturbEquilibriumState(
    scenario,
    equilibrium.anglesRad,
    {
      tipAnglePerturbationRad: 0.025,
      velocityAmplitudeRadS: 0.04,
    },
  );
  let actuator = createHandActuatorState();
  let boundary = { ...ZERO_HAND_BOUNDARY };
  let rmsIntegral = 0;
  let maxRms = 0;
  let saturationTimeS = 0;

  const steps = Math.round(durationS / DT);
  for (let step = 0; step < steps; step += 1) {
    const timeS = step * DT;
    const target = mode === "lqr"
      ? lqrHandTarget(
          design,
          state,
          actuator,
          DEFAULT_HAND_ACTUATOR_LIMITS,
        ).target
      : {
          lateralPositionM: 0,
          angleRad: 0,
        };

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

    assert.ok(state.anglesRad.every(Number.isFinite));
    assert.ok(state.angularRatesRadS.every(Number.isFinite));

    const current = rodKinematicsWithHandBoundary(
      scenario.system,
      state,
      boundary,
    );
    const rms = rmsFromEquilibrium(
      current,
      equilibrium.kinematics,
    );
    rmsIntegral += rms * DT;
    maxRms = Math.max(maxRms, rms);
    if (Object.values(actuatorStep.saturation).some(Boolean)) {
      saturationTimeS += DT;
    }
  }

  return {
    kind,
    mode,
    durationS,
    rmsIntegral,
    rmsMean: rmsIntegral / durationS,
    maxRms,
    saturationTimeS,
    controllability: design?.controllability ?? null,
    lqrIterations: design?.lqr.iterations ?? null,
    maxAbsGain: design?.lqr.maxAbsGain ?? null,
  };
}

const baselineOpen = simulate("baseline18", "open", {
  durationS: 4,
});
const baselineLqr = simulate("baseline18", "lqr", {
  durationS: 4,
});
const fastOpen = simulate("fast22", "open", {
  durationS: 3.5,
});
const fastLqr = simulate("fast22", "lqr", {
  durationS: 3.5,
});

assert.ok(
  baselineLqr.maxRms <= 1.20 * baselineOpen.maxRms,
  "LQR should not greatly amplify the stable baseline peak RMS",
);
assert.ok(
  fastLqr.rmsIntegral < fastOpen.rmsIntegral,
  "LQR must reduce Fast 22 RMS integral",
);
assert.ok(
  fastLqr.maxRms < fastOpen.maxRms,
  "LQR must reduce Fast 22 peak RMS",
);
assert.ok(
  fastLqr.saturationTimeS < 0.35 * fastLqr.durationS,
  "LQR should not spend most of its time saturated",
);

console.log(
  "H1-6-2 full-state/LQR checks OK:",
  JSON.stringify({
    realization: {
      dimension: baselineRealization.descriptor.dimension,
      residualNorm: baselineRealization.residualNorm,
      bNorm,
      halfDifference,
      doubleDifference,
      controllability,
      lqrIterations: baselineDesign.lqr.iterations,
      maxAbsGain: baselineDesign.lqr.maxAbsGain,
    },
    baseline: {
      open: baselineOpen,
      lqr: baselineLqr,
      ratio:
        baselineLqr.rmsIntegral / baselineOpen.rmsIntegral,
    },
    fast: {
      open: fastOpen,
      lqr: fastLqr,
      ratio: fastLqr.rmsIntegral / fastOpen.rmsIntegral,
    },
  }),
);
