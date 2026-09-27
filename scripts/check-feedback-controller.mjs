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
  controllerHandTarget,
  DEFAULT_P_GAINS,
  DEFAULT_PD_GAINS,
  pHandTarget,
  pdHandTarget,
  senseTipFeedback,
} from "../docs/js/shower/flexible/feedback-controller.js";

const DT = 0.002;

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

function optionsFor(kind) {
  if (kind === "fast22") {
    return {
      flowLpm: 22,
      lengthM: 1.5,
      flexuralRigidityNm2: 0.25,
      rayleighMassPerS: 0.02,
      rayleighStiffnessS: 0.0002,
    };
  }
  return {
    flowLpm: 18,
    lengthM: 1.2,
    flexuralRigidityNm2: 0.7,
    rayleighMassPerS: 0.08,
    rayleighStiffnessS: 0.0002,
  };
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

function simulate(kind, mode, {
  durationS = 5,
  pGains = DEFAULT_P_GAINS,
  pdGains = DEFAULT_PD_GAINS,
} = {}) {
  const { scenario, equilibrium } = solveContinuation(
    optionsFor(kind),
  );
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

  let rmsIntegral = 0;
  let maxRms = 0;
  let effortJ = 0;
  let previousPowerW = 0;
  let saturationTimeS = 0;

  const steps = Math.round(durationS / DT);
  for (let step = 0; step < steps; step += 1) {
    const timeS = step * DT;
    const sensing = senseTipFeedback(
      scenario.system,
      state,
      boundary,
      equilibrium.kinematics,
    );
    const target = controllerHandTarget(
      mode,
      sensing,
      {
        pGains,
        pdGains,
        limits: DEFAULT_HAND_ACTUATOR_LIMITS,
      },
    ) ?? {
      lateralPositionM: 0,
      angleRad: 0,
    };

    const actuatorStart = actuator;
    const actuatorStep = stepHandActuator(
      actuatorStart,
      target,
      DT,
      DEFAULT_HAND_ACTUATOR_LIMITS,
    );
    const trajectory = actuatorBoundaryTrajectory(
      actuatorStart,
      actuatorStep,
      DT,
    );
    const boundaryAtTime = (absoluteTimeS) => trajectory(
      absoluteTimeS - timeS,
    );

    const next = stepRodWithHandBoundaryRK4(
      scenario.system,
      state,
      timeS,
      DT,
      boundaryAtTime,
      loadOptions(scenario),
    );

    actuator = actuatorStep.state;
    boundary = next.boundary;
    state = next.state;

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

    effortJ += 0.5
      * (
        Math.abs(previousPowerW)
        + Math.abs(next.diagnostics.handPowerW)
      )
      * DT;
    previousPowerW = next.diagnostics.handPowerW;

    if (
      Object.values(actuatorStep.saturation).some(Boolean)
    ) {
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
    effortJ,
    saturationTimeS,
    finalActuator: actuator,
  };
}

// H1-6-0 sensor algebra.
{
  const { scenario, equilibrium } = solveContinuation(
    optionsFor("baseline18"),
  );
  const equilibriumState = {
    anglesRad: [...equilibrium.anglesRad],
    angularRatesRadS: Array(
      equilibrium.anglesRad.length,
    ).fill(0),
  };
  const sensed = senseTipFeedback(
    scenario.system,
    equilibriumState,
    ZERO_HAND_BOUNDARY,
    equilibrium.kinematics,
  );
  assert.ok(Math.abs(sensed.tipLateralErrorM) < 1e-12);
  assert.ok(Math.abs(sensed.tipAngleErrorRad) < 1e-12);
  assert.ok(Math.abs(sensed.tipLateralVelocityMps) < 1e-12);
  assert.ok(Math.abs(sensed.tipAngularRateRadS) < 1e-12);

  const pZero = pHandTarget(sensed);
  const pdZero = pdHandTarget(sensed);
  assert.ok(Math.abs(pZero.lateralPositionM) < 1e-12);
  assert.ok(Math.abs(pZero.angleRad) < 1e-12);
  assert.ok(Math.abs(pdZero.lateralPositionM) < 1e-12);
  assert.ok(Math.abs(pdZero.angleRad) < 1e-12);

  const positiveError = {
    ...sensed,
    tipLateralErrorM: 0.05,
    tipAngleErrorRad: 0.10,
    tipLateralVelocityMps: 0.20,
    tipAngularRateRadS: 0.30,
  };
  const p = pHandTarget(positiveError);
  const pd = pdHandTarget(positiveError);
  assert.ok(p.lateralPositionM < 0);
  assert.ok(p.angleRad < 0);
  assert.ok(pd.lateralPositionM < p.lateralPositionM);
  assert.ok(pd.angleRad < p.angleRad);

  const huge = pdHandTarget({
    ...positiveError,
    tipLateralErrorM: 100,
    tipAngleErrorRad: 100,
    tipLateralVelocityMps: 100,
    tipAngularRateRadS: 100,
  });
  assert.equal(
    huge.lateralPositionM,
    DEFAULT_HAND_ACTUATOR_LIMITS.lateralMinM,
  );
  assert.equal(
    huge.angleRad,
    DEFAULT_HAND_ACTUATOR_LIMITS.angleMinRad,
  );
}

const baseline = {
  open: simulate("baseline18", "off"),
  p: simulate("baseline18", "p"),
  pd: simulate("baseline18", "pd"),
};
const fast = {
  open: simulate("fast22", "off", { durationS: 3.5 }),
  p: simulate("fast22", "p", { durationS: 3.5 }),
  pd: simulate("fast22", "pd", { durationS: 3.5 }),
};

console.log(
  "H1-6 P/PD calibration:",
  JSON.stringify({ baseline, fast }),
);
