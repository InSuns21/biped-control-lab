import assert from "node:assert/strict";
import {
  EigenvalueDecomposition,
  Matrix,
} from "ml-matrix";
import {
  createNonlinearShowerScenario,
  geometricRmsFromEquilibrium,
  linearizeNonlinearShowerScenario,
  simulateNonlinearShowerOnset,
  solveNonlinearShowerEquilibrium,
} from "../docs/js/shower/flexible/nonlinear-scenario.js";

const radToDeg = (rad) => rad * 180 / Math.PI;

function equilibriumEigenSummary(scenario, equilibrium) {
  const evd = new EigenvalueDecomposition(
    new Matrix(
      linearizeNonlinearShowerScenario(
        scenario,
        equilibrium,
      ),
    ),
  );
  const real = evd.realEigenvalues;
  const imag = evd.imaginaryEigenvalues;
  let index = 0;
  for (let i = 1; i < real.length; i += 1) {
    if (real[i] > real[index]) index = i;
  }
  return {
    maxReal: real[index],
    imagAtMax: imag[index],
  };
}

const staticFlowsLpm = [0, 5, 8, 12, 14, 16, 18, 20, 22, 24];
const equilibria = new Map();
let continuationAngles = null;

console.log("\n### H1-4B nonlinear static flow continuation");
for (const flowLpm of staticFlowsLpm) {
  const scenario = createNonlinearShowerScenario({
    flowLpm,
    segmentCount: 12,
  });
  const equilibrium = solveNonlinearShowerEquilibrium(
    scenario,
    {
      initialAnglesRad: continuationAngles,
      tolerance: 1e-8,
    },
  );

  assert.ok(
    equilibrium.converged,
    `nonlinear equilibrium must converge at Q=${flowLpm} L/min`,
  );
  assert.ok(
    equilibrium.anglesRad.every(Number.isFinite),
    `nonlinear equilibrium must stay finite at Q=${flowLpm} L/min`,
  );

  const eigen = equilibriumEigenSummary(
    scenario,
    equilibrium,
  );
  continuationAngles = [...equilibrium.anglesRad];
  equilibria.set(flowLpm, {
    scenario,
    equilibrium,
    eigen,
  });

  console.log(JSON.stringify({
    flowLpm,
    flowSpeedMps: scenario.flowSpeedMps,
    residual: equilibrium.residualNorm,
    tipX: equilibrium.kinematics.tip[0],
    tipY: equilibrium.kinematics.tip[1],
    tipAngleDeg: radToDeg(equilibrium.kinematics.tipAngleRad),
    maxAbsAngleDeg: Math.max(
      ...equilibrium.anglesRad.map((angle) => Math.abs(radToDeg(angle))),
    ),
    maxReal: eigen.maxReal,
    imagAtMax: eigen.imagAtMax,
  }));
}

// The nonlinear equilibrium should remain geometrically finite rather than
// reproducing the H1-4 small-angle extrapolation of about 98 deg at 18 L/min.
const eq18 = equilibria.get(18).equilibrium;
const eig18 = equilibria.get(18).eigen;
assert.ok(
  Math.abs(radToDeg(eq18.kinematics.tipAngleRad)) < 60,
  "18 L/min nonlinear equilibrium should remain a finite-rotation configuration",
);
assert.ok(
  Math.abs(eq18.kinematics.tip[0]) < 0.9,
  "18 L/min nonlinear equilibrium should not reproduce the 1.08 m linear extrapolation",
);

const dynamicFlowsLpm = [5, 12, 16, 18, 20, 22, 24];
const dynamicResults = [];

console.log("\n### H1-4B nonlinear onset sweep");
for (const flowLpm of dynamicFlowsLpm) {
  const { scenario, equilibrium } = equilibria.get(flowLpm);
  const result = simulateNonlinearShowerOnset(
    scenario,
    equilibrium,
    {
      durationS: 6,
      dt: 0.001,
      onsetFactor: 3,
      absoluteOnsetM: 0.020,
      tipAnglePerturbationRad: 0.02,
      velocityAmplitudeRadS: 0.03,
    },
  );

  assert.equal(
    result.numericalFailure,
    false,
    `nonlinear onset simulation must stay finite at Q=${flowLpm} L/min`,
  );

  const initialGeometry = geometricRmsFromEquilibrium(
    scenario,
    result.finalState,
    equilibrium.kinematics,
  );
  assert.ok(
    Number.isFinite(initialGeometry.rmsM),
    "final nonlinear geometry must remain finite",
  );

  const row = {
    flowLpm,
    flowSpeedMps: scenario.flowSpeedMps,
    initialRmsMm: 1000 * result.initialRmsM,
    thresholdMm: 1000 * result.onsetThresholdM,
    onsetS: result.onsetTimeS,
    maxRmsMm: 1000 * result.maxRmsM,
    maxTipMm: 1000 * result.maxTipDisplacementM,
    largeRotationS: result.largeRotationTimeS,
  };
  dynamicResults.push(row);
  console.log(JSON.stringify(row));
}

const low = dynamicResults.find((x) => x.flowLpm === 5);
assert.equal(
  low.onsetS,
  null,
  "5 L/min nonlinear scenario should not reach the visible-onset threshold in 6 s",
);

const fastCases = dynamicResults.filter(
  (x) => x.onsetS !== null && x.onsetS >= 1 && x.onsetS <= 3,
);
const veryFastCases = dynamicResults.filter(
  (x) => x.onsetS !== null && x.onsetS < 1,
);

console.log("\nH1-4B_ONSET_RESULT", JSON.stringify({
  targetWindowS: [1, 3],
  fastCases,
  veryFastCases,
  static18: {
    tipX: eq18.kinematics.tip[0],
    tipY: eq18.kinematics.tip[1],
    tipAngleDeg: radToDeg(eq18.kinematics.tipAngleRad),
    maxReal: eig18.maxReal,
    imagAtMax: eig18.imagAtMax,
  },
  equilibriumStability: staticFlowsLpm.map((flowLpm) => ({
    flowLpm,
    maxReal: equilibria.get(flowLpm).eigen.maxReal,
    imagAtMax: equilibria.get(flowLpm).eigen.imagAtMax,
  })),
}));

console.log(
  "H1-4B full nonlinear scenario sweep complete; no real-product identification claimed.",
);
