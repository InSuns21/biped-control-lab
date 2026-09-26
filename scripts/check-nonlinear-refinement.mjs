import assert from "node:assert/strict";
import {
  createNonlinearShowerScenario,
  simulateNonlinearShowerOnset,
  solveNonlinearShowerEquilibrium,
} from "../docs/js/shower/flexible/nonlinear-scenario.js";

const FAST_CASE = Object.freeze({
  flowLpm: 22,
  flexuralRigidityNm2: 0.25,
  lengthM: 1.5,
  rayleighMassPerS: 0.02,
  rayleighStiffnessS: 0.0002,
  headMassKg: 0.20,
  headRotInertiaKgM2: 0.002,
});

function solveContinuation(segmentCount) {
  let angles = null;
  let scenario = null;
  let equilibrium = null;

  for (const flowLpm of [0, 7.33, 14.74, 22]) {
    scenario = createNonlinearShowerScenario({
      ...FAST_CASE,
      segmentCount,
      flowLpm,
    });
    equilibrium = solveNonlinearShowerEquilibrium(
      scenario,
      { initialAnglesRad: angles },
    );
    assert.ok(
      equilibrium.converged,
      `fast-case equilibrium must converge at N=${segmentCount}, Q=${flowLpm}`,
    );
    angles = [...equilibrium.anglesRad];
  }

  return { scenario, equilibrium };
}

function run(segmentCount, dt) {
  const { scenario, equilibrium } = solveContinuation(segmentCount);
  const result = simulateNonlinearShowerOnset(
    scenario,
    equilibrium,
    {
      durationS: 3.5,
      dt,
    },
  );
  assert.equal(result.numericalFailure, false);
  assert.ok(
    result.onsetTimeS !== null,
    `fast case should reach onset for N=${segmentCount}, dt=${dt}`,
  );
  return {
    segmentCount,
    dt,
    onsetS: result.onsetTimeS,
    maxRmsMm: 1000 * result.maxRmsM,
    maxTipMm: 1000 * result.maxTipDisplacementM,
    tipAngleDeg: equilibrium.kinematics.tipAngleRad * 180 / Math.PI,
  };
}

const n10 = run(10, 0.001);
const n12 = run(12, 0.001);
const n16 = run(16, 0.001);
const dt2 = run(12, 0.002);

assert.ok(
  Math.abs(n10.onsetS - n12.onsetS) < 0.20,
  "10 -> 12 segment onset should agree within 0.20 s",
);
assert.ok(
  Math.abs(n12.onsetS - n16.onsetS) < 0.20,
  "12 -> 16 segment onset should agree within 0.20 s",
);
assert.ok(
  Math.abs(n12.onsetS - dt2.onsetS) < 0.08,
  "dt=0.001 -> 0.002 onset should agree within 0.08 s",
);

console.log(
  "H1-4B nonlinear refinement OK:",
  JSON.stringify({ n10, n12, n16, dt2 }),
);
