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
  stepRodWithHandBoundaryRK4,
} from "../docs/js/shower/flexible/nonlinear-boundary.js";

function maxAbsDiff(a, b) {
  return Math.max(
    ...a.map((value, index) => Math.abs(value - b[index])),
  );
}

function solveFastScenario() {
  const options = {
    segmentCount: 10,
    flexuralRigidityNm2: 0.25,
    lengthM: 1.5,
    rayleighMassPerS: 0.02,
    rayleighStiffnessS: 0.0002,
  };
  let angles = null;
  let scenario = null;
  let equilibrium = null;
  for (const flowLpm of [0, 7.33, 14.74, 22]) {
    scenario = createNonlinearShowerScenario({
      ...options,
      flowLpm,
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

const { scenario, equilibrium } = solveFastScenario();
const initial = perturbEquilibriumState(
  scenario,
  equilibrium.anglesRad,
  {
    tipAnglePerturbationRad: 0.01,
    velocityAmplitudeRadS: 0.015,
  },
);
let fixedState = {
  anglesRad: [...initial.anglesRad],
  angularRatesRadS: [...initial.angularRatesRadS],
};
let controlledState = {
  anglesRad: [...initial.anglesRad],
  angularRatesRadS: [...initial.angularRatesRadS],
};

let actuator = createHandActuatorState();
const limits = DEFAULT_HAND_ACTUATOR_LIMITS;
const dt = 0.001;
let maxBoundaryEndpointError = 0;
let maxHandX = 0;
let maxHandAngle = 0;
let accumulatedAbsPowerJ = 0;
let previousPower = 0;

const loadOptions = {
  tipLoad: scenario.tipLoad,
  additionalGeneralizedForce: scenario.flowForce,
  additionalCartesianResultant: scenario.flowResultant,
};

const zeroBoundary = () => ({
  lateralPositionM: 0,
  lateralVelocityMps: 0,
  lateralAccelerationMps2: 0,
  angleRad: 0,
  angularRateRadS: 0,
  angularAccelerationRadS2: 0,
});

for (let stepIndex = 0; stepIndex < 1800; stepIndex += 1) {
  const timeS = stepIndex * dt;
  const target = timeS < 0.55
    ? {
        lateralPositionM: 0.055,
        angleRad: 14 * Math.PI / 180,
      }
    : timeS < 1.10
      ? {
          lateralPositionM: -0.040,
          angleRad: -10 * Math.PI / 180,
        }
      : {
          lateralPositionM: 0,
          angleRad: 0,
        };

  const actuatorStart = actuator;
  const actuatorStep = stepHandActuator(
    actuatorStart,
    target,
    dt,
    limits,
  );
  const trajectory = actuatorBoundaryTrajectory(
    actuatorStart,
    actuatorStep,
    dt,
  );
  const boundaryAtTime = (absoluteTimeS) => trajectory(
    absoluteTimeS - timeS,
  );

  const controlledNext = stepRodWithHandBoundaryRK4(
    scenario.system,
    controlledState,
    timeS,
    dt,
    boundaryAtTime,
    loadOptions,
  );
  const fixedNext = stepRodWithHandBoundaryRK4(
    scenario.system,
    fixedState,
    timeS,
    dt,
    zeroBoundary,
    loadOptions,
  );

  const endpoint = boundaryAtTime(timeS + dt);
  maxBoundaryEndpointError = Math.max(
    maxBoundaryEndpointError,
    Math.abs(
      endpoint.lateralPositionM
        - actuatorStep.state.lateralPositionM,
    ),
    Math.abs(
      endpoint.lateralVelocityMps
        - actuatorStep.state.lateralVelocityMps,
    ),
    Math.abs(endpoint.angleRad - actuatorStep.state.angleRad),
    Math.abs(
      endpoint.angularRateRadS
        - actuatorStep.state.angularRateRadS,
    ),
  );

  assert.ok(
    Math.abs(actuatorStep.state.lateralVelocityMps)
      <= limits.lateralMaxSpeedMps + 1e-10,
  );
  assert.ok(
    Math.abs(actuatorStep.acceleration.lateralAccelerationMps2)
      <= limits.lateralMaxAccelerationMps2 + 1e-10,
  );
  assert.ok(
    Math.abs(actuatorStep.state.angularRateRadS)
      <= limits.angularMaxSpeedRadS + 1e-10,
  );
  assert.ok(
    Math.abs(actuatorStep.acceleration.angularAccelerationRadS2)
      <= limits.angularMaxAccelerationRadS2 + 1e-10,
  );

  accumulatedAbsPowerJ += 0.5
    * (
      Math.abs(previousPower)
      + Math.abs(controlledNext.diagnostics.handPowerW)
    )
    * dt;
  previousPower = controlledNext.diagnostics.handPowerW;

  actuator = actuatorStep.state;
  controlledState = controlledNext.state;
  fixedState = fixedNext.state;
  maxHandX = Math.max(
    maxHandX,
    Math.abs(controlledNext.boundary.lateralPositionM),
  );
  maxHandAngle = Math.max(
    maxHandAngle,
    Math.abs(controlledNext.boundary.angleRad),
  );

  assert.ok(controlledState.anglesRad.every(Number.isFinite));
  assert.ok(
    controlledState.angularRatesRadS.every(Number.isFinite),
  );
}

const stateDifference = Math.max(
  maxAbsDiff(
    controlledState.anglesRad,
    fixedState.anglesRad,
  ),
  maxAbsDiff(
    controlledState.angularRatesRadS,
    fixedState.angularRatesRadS,
  ),
);

assert.ok(
  maxBoundaryEndpointError < 1e-10,
  "actuator polynomial endpoint must equal the prescribed boundary endpoint",
);
assert.ok(
  maxHandX > 0.025,
  "direct actuator control must create visible lateral hand motion",
);
assert.ok(
  maxHandAngle > 5 * Math.PI / 180,
  "direct actuator control must create visible angular hand motion",
);
assert.ok(
  accumulatedAbsPowerJ > 1e-3,
  "direct actuator control must exchange measurable boundary energy",
);
assert.ok(
  stateDifference > 0.02,
  "direct actuator control must materially alter flowing hose dynamics",
);
assert.ok(
  Math.abs(actuator.lateralPositionM) < 0.01,
  "neutral target should bring lateral hand state back toward center",
);
assert.ok(
  Math.abs(actuator.angleRad) < 0.06,
  "neutral target should bring angular hand state back toward center",
);

console.log(
  "H1-5-1 direct control integration OK:",
  JSON.stringify({
    maxBoundaryEndpointError,
    maxHandX,
    maxHandAngleDeg: maxHandAngle * 180 / Math.PI,
    accumulatedAbsPowerJ,
    stateDifference,
    finalActuator: actuator,
  }),
);
