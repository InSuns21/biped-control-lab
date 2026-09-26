import assert from "node:assert/strict";
import {
  assembleDryBeam,
  dryStaticTipShapeReduced,
} from "../docs/js/shower/flexible/assemble.js";
import {
  cantileverAnalyticFirstFrequencyHz,
  naturalFrequenciesHz,
} from "../docs/js/shower/flexible/eigen.js";
import {
  createNewmarkAverageAcceleration,
  mechanicalEnergy,
} from "../docs/js/shower/flexible/integrator.js";
import {
  hermiteBeamElementMatrices,
} from "../docs/js/shower/flexible/beam-element.js";

const nearly = (a, b, tol = 1e-12) => Math.abs(a - b) <= tol;

function assertSymmetric(matrix, message) {
  for (let i = 0; i < matrix.length; i += 1) {
    for (let j = i + 1; j < matrix.length; j += 1) {
      assert.ok(nearly(matrix[i][j], matrix[j][i]), message);
    }
  }
}

const element = hermiteBeamElementMatrices({
  lengthM: 0.3,
  flexuralRigidityNm2: 0.7,
  massPerLengthKgM: 0.25,
});
assertSymmetric(element.mass, "consistent element mass must be symmetric");
assertSymmetric(
  element.stiffness,
  "Euler-Bernoulli element stiffness must be symmetric",
);

// H1-1 frequency convergence against the analytical clamped-free beam.
const referenceParams = {
  lengthM: 1.2,
  flexuralRigidityNm2: 0.7,
  structuralMassPerM: 0.25,
  tipMassKg: 0,
  tipRotInertiaKgM2: 0,
  rayleighMassPerS: 0,
  rayleighStiffnessS: 0,
};
const analyticHz = cantileverAnalyticFirstFrequencyHz(referenceParams);
const meshes = [2, 4, 8, 16];
const frequencyErrors = meshes.map((elementCount) => {
  const system = assembleDryBeam({ ...referenceParams, elementCount });
  const firstHz = naturalFrequenciesHz(system, 1)[0];
  return Math.abs(firstHz - analyticHz) / analyticHz;
});

for (let i = 1; i < frequencyErrors.length; i += 1) {
  assert.ok(
    frequencyErrors[i] < frequencyErrors[i - 1],
    "first-mode frequency error should decrease under mesh refinement",
  );
}
assert.ok(
  frequencyErrors.at(-1) < 1e-4,
  "16-element dry beam should converge tightly to the analytical first mode",
);

// A positive shower-head tip mass must lower the first dry natural frequency.
const noTipSystem = assembleDryBeam({
  ...referenceParams,
  elementCount: 8,
});
const withTipSystem = assembleDryBeam({
  ...referenceParams,
  elementCount: 8,
  tipMassKg: 0.20,
  tipRotInertiaKgM2: 0.002,
});
const noTipHz = naturalFrequenciesHz(noTipSystem, 1)[0];
const withTipHz = naturalFrequenciesHz(withTipSystem, 1)[0];
assert.ok(
  withTipHz < noTipHz,
  "adding shower-head tip inertia must lower the first natural frequency",
);

// Average-acceleration Newmark should conserve mechanical energy for the
// undamped linear dry beam to numerical precision.
const conservativeSystem = assembleDryBeam({
  ...referenceParams,
  elementCount: 8,
  tipMassKg: 0.20,
  tipRotInertiaKgM2: 0.002,
});
const conservativeIntegrator = createNewmarkAverageAcceleration(
  conservativeSystem.reduced,
  0.002,
);
const q0 = dryStaticTipShapeReduced(conservativeSystem, 0.03);
let conservativeState = conservativeIntegrator.initialize({
  q: q0,
  v: Array(q0.length).fill(0),
});
const conservativeEnergy0 = mechanicalEnergy(
  conservativeSystem.reduced,
  conservativeState,
);
let minEnergy = conservativeEnergy0;
let maxEnergy = conservativeEnergy0;
for (let i = 0; i < 2500; i += 1) {
  conservativeState = conservativeIntegrator.step(conservativeState);
  const energy = mechanicalEnergy(
    conservativeSystem.reduced,
    conservativeState,
  );
  assert.ok(Number.isFinite(energy), "dry integration energy must stay finite");
  minEnergy = Math.min(minEnergy, energy);
  maxEnergy = Math.max(maxEnergy, energy);
}
assert.ok(
  (maxEnergy - minEnergy) / conservativeEnergy0 < 1e-5,
  "undamped Newmark average-acceleration run should conserve energy",
);

// Structural damping must remove energy instead of requiring fake animation
// damping in the renderer.
const dampedSystem = assembleDryBeam({
  ...referenceParams,
  elementCount: 8,
  tipMassKg: 0.20,
  tipRotInertiaKgM2: 0.002,
  rayleighMassPerS: 0.25,
  rayleighStiffnessS: 0.0002,
});
const dampedIntegrator = createNewmarkAverageAcceleration(
  dampedSystem.reduced,
  0.002,
);
const dampedQ0 = dryStaticTipShapeReduced(dampedSystem, 0.03);
let dampedState = dampedIntegrator.initialize({
  q: dampedQ0,
  v: Array(dampedQ0.length).fill(0),
});
const dampedEnergy0 = mechanicalEnergy(dampedSystem.reduced, dampedState);
for (let i = 0; i < 4000; i += 1) {
  dampedState = dampedIntegrator.step(dampedState);
}
const dampedEnergyFinal = mechanicalEnergy(dampedSystem.reduced, dampedState);
assert.ok(
  dampedEnergyFinal < 0.7 * dampedEnergy0,
  "Rayleigh-damped dry hose should lose substantial mechanical energy",
);

console.log(
  [
    "Flexible hose H1-1 checks OK:",
    `analytic f1=${analyticHz.toFixed(6)} Hz`,
    `8-element no-tip f1=${noTipHz.toFixed(6)} Hz`,
    `8-element with-tip f1=${withTipHz.toFixed(6)} Hz`,
    `16-element relative error=${frequencyErrors.at(-1).toExponential(3)}`,
    `damped energy ratio=${(dampedEnergyFinal / dampedEnergy0).toFixed(4)}`,
  ].join(" "),
);
