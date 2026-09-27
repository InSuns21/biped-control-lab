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

function solveAtFlow(overrides, segmentCount) {
  const targetFlow = overrides.flowLpm;
  const flowSteps = [
    0,
    0.33 * targetFlow,
    0.67 * targetFlow,
    targetFlow,
  ];
  let angles = null;
  let scenario = null;
  let equilibrium = null;

  for (const flowLpm of flowSteps) {
    scenario = createNonlinearShowerScenario({
      ...overrides,
      segmentCount,
      flowLpm,
    });
    equilibrium = solveNonlinearShowerEquilibrium(
      scenario,
      { initialAnglesRad: angles },
    );
    if (!equilibrium.converged) {
      return { converged: false };
    }
    angles = [...equilibrium.anglesRad];
  }

  return {
    converged: true,
    scenario,
    equilibrium,
    ...eigenSummary(scenario, equilibrium),
    maxAbsAngleDeg: Math.max(
      ...equilibrium.anglesRad.map(
        (angle) => Math.abs(radToDeg(angle)),
      ),
    ),
    tipY: equilibrium.kinematics.tip[1],
  };
}

const coarse = [];
for (const flowLpm of [18, 20, 22, 24]) {
  for (const flexuralRigidityNm2 of [0.25, 0.35, 0.50, 0.70]) {
    for (const lengthM of [1.2, 1.5, 1.8]) {
      for (const rayleighMassPerS of [0.02, 0.08]) {
        const overrides = {
          flowLpm,
          flexuralRigidityNm2,
          lengthM,
          rayleighMassPerS,
          rayleighStiffnessS: 0.0002,
          headMassKg: 0.20,
          headRotInertiaKgM2: 0.002,
        };
        coarse.push({
          overrides,
          ...solveAtFlow(overrides, 8),
        });
      }
    }
  }
}

const candidates = coarse
  .filter((x) => (
    x.converged
    && x.sigma > 0
    && x.predicted3xS >= 0.15
    && x.predicted3xS <= 4.0
    && x.maxAbsAngleDeg < 100
    && x.tipY > 0.10
  ))
  .sort((a, b) => {
    if (a.overrides.flowLpm !== b.overrides.flowLpm) {
      return a.overrides.flowLpm - b.overrides.flowLpm;
    }
    return a.predicted3xS - b.predicted3xS;
  });

console.log("\n### H1-4B nonlinear combination candidates");
for (const x of candidates.slice(0, 20)) {
  console.log(JSON.stringify({
    ...x.overrides,
    sigma: x.sigma,
    predicted3xS: x.predicted3xS,
    omega: x.omega,
    maxAbsAngleDeg: x.maxAbsAngleDeg,
    tipY: x.tipY,
  }));
}

assert.ok(
  candidates.length > 0,
  "nonlinear combination sweep should find at least one finite fast-growth candidate",
);

// Refine representatives from each flow bucket rather than allowing the
// lowest-flow cases to consume the whole budget. Within each bucket choose the
// candidates whose predicted 3x time is closest to 1.5 s.
const unique = [];
for (const flowLpm of [18, 20, 22, 24]) {
  const bucket = candidates
    .filter((candidate) => candidate.overrides.flowLpm === flowLpm);
  if (bucket.length === 0) continue;

  const fastest = [...bucket]
    .sort((a, b) => b.sigma - a.sigma)[0];
  const nearTarget = [...bucket]
    .sort((a, b) => (
      Math.abs(a.predicted3xS - 1.5)
      - Math.abs(b.predicted3xS - 1.5)
    ))[0];

  unique.push(fastest);
  if (nearTarget !== fastest) unique.push(nearTarget);
}

const refined = [];
console.log("\n### H1-4B refined nonlinear combination time histories");
for (const coarseCase of unique) {
  const solved = solveAtFlow(coarseCase.overrides, 12);
  assert.ok(solved.converged, "refined nonlinear combination must converge");

  const result = simulateNonlinearShowerOnset(
    solved.scenario,
    solved.equilibrium,
    {
      durationS: 6,
      dt: 0.001,
    },
  );
  assert.equal(result.numericalFailure, false);

  const row = {
    ...coarseCase.overrides,
    sigma: solved.sigma,
    predicted3xS: solved.predicted3xS,
    onsetS: result.onsetTimeS,
    maxRmsMm: 1000 * result.maxRmsM,
    maxTipMm: 1000 * result.maxTipDisplacementM,
    maxAbsAngleDeg: solved.maxAbsAngleDeg,
  };
  refined.push(row);
  console.log(JSON.stringify(row));
}

const fast = refined.filter(
  (x) => x.onsetS !== null && x.onsetS >= 1 && x.onsetS <= 3,
);

console.log("\nH1-4B_COMBINATION_RESULT", JSON.stringify({
  fast,
  candidateCount: candidates.length,
}));

console.log(
  "H1-4B nonlinear combination sweep complete; combinations are educational sensitivity cases, not fitted real-hose parameters.",
);
