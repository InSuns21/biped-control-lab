import assert from "node:assert/strict";
import {
  createNonlinearRod,
  stepNonlinearRodRK4,
  straightRodState,
} from "../docs/js/shower/flexible/nonlinear-rod.js";
import {
  createNonlinearShowerScenario,
  perturbEquilibriumState,
  solveNonlinearShowerEquilibrium,
} from "../docs/js/shower/flexible/nonlinear-scenario.js";
import {
  handBoundaryDynamics,
  mechanicalEnergyWithHandBoundary,
  smoothHandPulse,
  stepRodWithHandBoundaryRK4,
  ZERO_HAND_BOUNDARY,
} from "../docs/js/shower/flexible/nonlinear-boundary.js";

const degToRad = (deg) => deg * Math.PI / 180;
const maxAbsDiff = (a, b) => Math.max(
  ...a.map((value, i) => Math.abs(value - b[i])),
);

function loadOptionsForScenario(scenario) {
  return {
    tipLoad: scenario.tipLoad,
    additionalGeneralizedForce: scenario.flowForce,
    additionalCartesianResultant: scenario.flowResultant,
  };
}

// 1) Fixed-boundary parity: the H1-5-0 solver must reduce to H1-4B when the
// hand is not moving.
const scenario = createNonlinearShowerScenario({
  flowLpm: 18,
  segmentCount: 10,
});
const equilibrium = solveNonlinearShowerEquilibrium(scenario);
assert.ok(equilibrium.converged);

let fixed = perturbEquilibriumState(
  scenario,
  equilibrium.anglesRad,
  {
    tipAnglePerturbationRad: 0.01,
    velocityAmplitudeRadS: 0.02,
  },
);
let boundary = {
  anglesRad: [...fixed.anglesRad],
  angularRatesRadS: [...fixed.angularRatesRadS],
};
const dtParity = 0.001;
const boundaryAtRest = () => ({ ...ZERO_HAND_BOUNDARY });

for (let step = 0; step < 250; step += 1) {
  fixed = stepNonlinearRodRK4(
    scenario.system,
    fixed,
    dtParity,
    scenario.tipLoad,
    scenario.flowForce,
  );
  boundary = stepRodWithHandBoundaryRK4(
    scenario.system,
    boundary,
    step * dtParity,
    dtParity,
    boundaryAtRest,
    loadOptionsForScenario(scenario),
  ).state;
}

assert.ok(
  maxAbsDiff(fixed.anglesRad, boundary.anglesRad) < 2e-10,
  "stationary H1-5-0 boundary must reproduce H1-4B angles",
);
assert.ok(
  maxAbsDiff(
    fixed.angularRatesRadS,
    boundary.angularRatesRadS,
  ) < 2e-9,
  "stationary H1-5-0 boundary must reproduce H1-4B rates",
);

// 2) Smooth pulse endpoints are kinematically quiet.
const pulseOptions = {
  startTimeS: 0.1,
  durationS: 0.40,
  lateralAmplitudeM: 0.010,
  angleAmplitudeRad: degToRad(5),
};
for (const t of [0, 0.1, 0.5, 0.8]) {
  const b = smoothHandPulse(t, pulseOptions);
  if (t <= 0.1 || t >= 0.5) {
    for (const value of Object.values(b)) {
      assert.ok(
        Math.abs(value) < 1e-14,
        "pulse must be at rest outside its active interval",
      );
    }
  }
}

// 3) Conservative boundary-work balance.
// With no gravity, damping, flow or external tip load, hand work should become
// rod kinetic + bending energy.
const conservative = createNonlinearRod({
  segmentCount: 8,
  lengthM: 1.2,
  flexuralRigidityNm2: 0.7,
  structuralMassPerM: 0.25,
  fluidMassPerM: 0,
  gravityMps2: 0,
  headMassKg: 0.20,
  headRotInertiaKgM2: 0.002,
  rayleighMassPerS: 0,
  rayleighStiffnessS: 0,
});
let state = straightRodState(conservative);
const boundaryAtTime = (timeS) => smoothHandPulse(
  timeS,
  pulseOptions,
);
const dt = 0.0005;
const durationS = 1.2;
const initialBoundary = boundaryAtTime(0);
const energy0 = mechanicalEnergyWithHandBoundary(
  conservative,
  state,
  initialBoundary,
).totalJ;
let diag0 = handBoundaryDynamics(
  conservative,
  state,
  initialBoundary,
);
let workJ = 0;

for (let step = 0; step < Math.round(durationS / dt); step += 1) {
  const timeS = step * dt;
  const next = stepRodWithHandBoundaryRK4(
    conservative,
    state,
    timeS,
    dt,
    boundaryAtTime,
  );
  workJ += 0.5
    * (diag0.handPowerW + next.diagnostics.handPowerW)
    * dt;
  state = next.state;
  diag0 = next.diagnostics;
}

const finalBoundary = boundaryAtTime(durationS);
const energy1 = mechanicalEnergyWithHandBoundary(
  conservative,
  state,
  finalBoundary,
).totalJ;
const deltaEnergyJ = energy1 - energy0;
const balanceScale = Math.max(
  1e-6,
  Math.abs(workJ),
  Math.abs(deltaEnergyJ),
);
const balanceRelativeError = Math.abs(
  workJ - deltaEnergyJ,
) / balanceScale;

assert.ok(Number.isFinite(workJ));
assert.ok(
  Math.abs(workJ) > 1e-4,
  "nontrivial hand pulse should exchange measurable energy",
);
assert.ok(
  balanceRelativeError < 0.025,
  "boundary work should match conservative mechanical-energy change within 2.5%",
);

// 4) A pulse during flowing dynamics must materially change the state while
// staying finite; this is the H1-5-0 "boundary input actually matters" guard.
const fast = createNonlinearShowerScenario({
  flowLpm: 22,
  segmentCount: 10,
  flexuralRigidityNm2: 0.25,
  lengthM: 1.5,
  rayleighMassPerS: 0.02,
  rayleighStiffnessS: 0.0002,
});
let angles = null;
let eq = null;
for (const flowLpm of [0, 7.33, 14.74, 22]) {
  const stage = createNonlinearShowerScenario({
    flowLpm,
    segmentCount: 10,
    flexuralRigidityNm2: 0.25,
    lengthM: 1.5,
    rayleighMassPerS: 0.02,
    rayleighStiffnessS: 0.0002,
  });
  eq = solveNonlinearShowerEquilibrium(
    stage,
    { initialAnglesRad: angles },
  );
  assert.ok(eq.converged);
  angles = [...eq.anglesRad];
}
let noHand = perturbEquilibriumState(
  fast,
  eq.anglesRad,
  {
    tipAnglePerturbationRad: 0.02,
    velocityAmplitudeRadS: 0.03,
  },
);
let withHand = {
  anglesRad: [...noHand.anglesRad],
  angularRatesRadS: [...noHand.angularRatesRadS],
};
const activePulse = (timeS) => smoothHandPulse(
  timeS,
  {
    startTimeS: 0.45,
    durationS: 0.35,
    lateralAmplitudeM: 0.012,
    angleAmplitudeRad: degToRad(6),
  },
);
const fastOptions = loadOptionsForScenario(fast);
const fastDt = 0.001;

for (let step = 0; step < 1400; step += 1) {
  const timeS = step * fastDt;
  noHand = stepNonlinearRodRK4(
    fast.system,
    noHand,
    fastDt,
    fast.tipLoad,
    fast.flowForce,
  );
  withHand = stepRodWithHandBoundaryRK4(
    fast.system,
    withHand,
    timeS,
    fastDt,
    activePulse,
    fastOptions,
  ).state;
}

const stateDifference = Math.max(
  maxAbsDiff(noHand.anglesRad, withHand.anglesRad),
  maxAbsDiff(
    noHand.angularRatesRadS,
    withHand.angularRatesRadS,
  ),
);
assert.ok(
  stateDifference > 0.02,
  "moving hand boundary must materially change nonlinear flowing dynamics",
);
assert.ok(withHand.anglesRad.every(Number.isFinite));
assert.ok(withHand.angularRatesRadS.every(Number.isFinite));

console.log(
  "H1-5-0 moving-hand checks OK:",
  JSON.stringify({
    fixedParityAngleError: maxAbsDiff(
      fixed.anglesRad,
      boundary.anglesRad,
    ),
    workJ,
    deltaEnergyJ,
    balanceRelativeError,
    flowingStateDifference: stateDifference,
  }),
);
