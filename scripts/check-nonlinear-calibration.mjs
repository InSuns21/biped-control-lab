import assert from "node:assert/strict";
import {
  EigenvalueDecomposition,
  Matrix,
} from "ml-matrix";
import {
  createNonlinearShowerScenario,
  linearizeNonlinearShowerScenario,
  simulateNonlinearShowerOnset,
  solveNonlinearShowerEquilibrium,
} from "../docs/js/shower/flexible/nonlinear-scenario.js";

const radToDeg = (rad) => rad * 180 / Math.PI;
const targetT3MinS = 0.8;
const targetT3MaxS = 4.0;

function eigenSummary(scenario, equilibrium) {
  const evd = new EigenvalueDecomposition(
    new Matrix(
      linearizeNonlinearShowerScenario(scenario, equilibrium),
    ),
  );
  const real = evd.realEigenvalues;
  const imag = evd.imaginaryEigenvalues;
  let index = 0;
  for (let i = 1; i < real.length; i += 1) {
    if (real[i] > real[index]) index = i;
  }
  const sigma = real[index];
  return {
    sigma,
    omega: imag[index],
    predicted3xS: sigma > 0 ? Math.log(3) / sigma : Infinity,
  };
}

function continuationCase(label, overrides) {
  const targetFlow = overrides.flowLpm ?? 18;
  const steps = [0, targetFlow / 3, 2 * targetFlow / 3, targetFlow];
  let angles = null;
  let finalScenario = null;
  let finalEquilibrium = null;

  for (const flowLpm of steps) {
    const scenario = createNonlinearShowerScenario({
      segmentCount: 10,
      ...overrides,
      flowLpm,
    });
    const equilibrium = solveNonlinearShowerEquilibrium(
      scenario,
      { initialAnglesRad: angles },
    );
    if (!equilibrium.converged) {
      return {
        label,
        overrides,
        converged: false,
      };
    }
    angles = [...equilibrium.anglesRad];
    finalScenario = scenario;
    finalEquilibrium = equilibrium;
  }

  const eigen = eigenSummary(finalScenario, finalEquilibrium);
  return {
    label,
    overrides,
    converged: true,
    scenario: finalScenario,
    equilibrium: finalEquilibrium,
    ...eigen,
    tipX: finalEquilibrium.kinematics.tip[0],
    tipY: finalEquilibrium.kinematics.tip[1],
    tipAngleDeg: radToDeg(finalEquilibrium.kinematics.tipAngleRad),
    maxAbsAngleDeg: Math.max(
      ...finalEquilibrium.anglesRad.map(
        (angle) => Math.abs(radToDeg(angle)),
      ),
    ),
  };
}

const cases = [];

for (const value of [0.30, 0.40, 0.55, 0.70, 0.90, 1.20]) {
  cases.push(continuationCase(`EI=${value}`, {
    flexuralRigidityNm2: value,
  }));
}
for (const value of [0.8, 1.0, 1.2, 1.4, 1.6, 1.8]) {
  cases.push(continuationCase(`L=${value}`, {
    lengthM: value,
  }));
}
for (const value of [0, 0.02, 0.05, 0.08, 0.12, 0.15]) {
  cases.push(continuationCase(`alphaM=${value}`, {
    rayleighMassPerS: value,
  }));
}
for (const value of [0, 0.00005, 0.0001, 0.0002, 0.00035, 0.0005]) {
  cases.push(continuationCase(`betaK=${value}`, {
    rayleighStiffnessS: value,
  }));
}
for (const value of [0.10, 0.15, 0.20, 0.25, 0.30, 0.35]) {
  cases.push(continuationCase(`headMass=${value}`, {
    headMassKg: value,
    headRotInertiaKgM2: 0.01 * value,
  }));
}
for (const value of [14, 16, 18, 20, 22, 24]) {
  cases.push(continuationCase(`Q=${value}`, {
    flowLpm: value,
  }));
}

assert.ok(cases.every((x) => x.converged), "all nonlinear OAT cases must converge");

console.log("\n### H1-4B nonlinear one-at-a-time sensitivity at/near 18 L/min");
for (const x of cases) {
  console.log(JSON.stringify({
    label: x.label,
    sigma: x.sigma,
    predicted3xS: Number.isFinite(x.predicted3xS)
      ? x.predicted3xS
      : null,
    omega: x.omega,
    tipX: x.tipX,
    tipY: x.tipY,
    tipAngleDeg: x.tipAngleDeg,
    maxAbsAngleDeg: x.maxAbsAngleDeg,
  }));
}

const shortlisted = cases
  .filter((x) => (
    x.sigma > 0
    && x.predicted3xS >= targetT3MinS
    && x.predicted3xS <= targetT3MaxS
    && x.maxAbsAngleDeg < 85
    && x.tipY > 0.20
  ))
  .sort((a, b) => a.predicted3xS - b.predicted3xS);

console.log("\n### H1-4B nonlinear OAT time-domain shortlist");
const timeResults = [];
for (const x of shortlisted.slice(0, 8)) {
  const result = simulateNonlinearShowerOnset(
    x.scenario,
    x.equilibrium,
    {
      durationS: 6,
      dt: 0.001,
    },
  );
  assert.equal(result.numericalFailure, false);
  const row = {
    label: x.label,
    flowLpm: x.scenario.params.flowLpm,
    predicted3xS: x.predicted3xS,
    onsetS: result.onsetTimeS,
    maxRmsMm: 1000 * result.maxRmsM,
    maxTipMm: 1000 * result.maxTipDisplacementM,
    equilibriumTipAngleDeg: x.tipAngleDeg,
  };
  timeResults.push(row);
  console.log(JSON.stringify(row));
}

const fast = timeResults.filter(
  (x) => x.onsetS !== null && x.onsetS >= 1 && x.onsetS <= 3,
);

console.log("\nH1-4B_SENSITIVITY_RESULT", JSON.stringify({
  fast,
  shortlistedCount: shortlisted.length,
}));

console.log(
  "H1-4B nonlinear sensitivity sweep complete; parameters remain educational, not product identification.",
);
