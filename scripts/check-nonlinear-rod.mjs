import assert from "node:assert/strict";
import {
  bendingEnergy,
  createNonlinearRod,
  linearizedAngleSystemAtStraight,
  rodKinematics,
  solveStaticRodEquilibrium,
  stepNonlinearRodRK4,
  straightRodState,
  totalMechanicalEnergy,
} from "../docs/js/shower/flexible/nonlinear-rod.js";
import {
  cantileverAnalyticFirstFrequencyHz,
  naturalFrequenciesHz,
} from "../docs/js/shower/flexible/eigen.js";

const nearly = (a, b, tol = 1e-10) => Math.abs(a - b) <= tol;
const distance = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1]);

// H1-4B-0 objectivity: a pure rigid rotation has zero bending energy and
// preserves every inextensible segment length exactly.
const rotated = createNonlinearRod({
  segmentCount: 12,
  baseAngleRad: 1.1,
  gravityMps2: 0,
  headMassKg: 0,
  headRotInertiaKgM2: 0,
});
const rigidAngles = Array(rotated.params.segmentCount).fill(1.1);
assert.ok(
  Math.abs(bendingEnergy(rotated, rigidAngles)) < 1e-14,
  "rigid rotation must not create artificial bending energy",
);
const rotatedGeometry = rodKinematics(rotated, rigidAngles);
for (let i = 0; i < rotated.params.segmentCount; i += 1) {
  assert.ok(
    nearly(
      distance(rotatedGeometry.nodes[i], rotatedGeometry.nodes[i + 1]),
      rotated.segmentLengthM,
      1e-12,
    ),
    "geometrically exact rod must preserve each segment length",
  );
}

// Small-amplitude limit: the angle-coordinate rod should converge toward the
// analytical Euler-Bernoulli cantilever first frequency as the mesh is refined.
const reference = {
  lengthM: 1.2,
  flexuralRigidityNm2: 0.7,
  structuralMassPerM: 0.25,
  fluidMassPerM: 0,
  gravityMps2: 0,
  headMassKg: 0,
  headRotInertiaKgM2: 0,
  headComAxialOffsetM: 0,
  rayleighMassPerS: 0,
  rayleighStiffnessS: 0,
  baseAngleRad: 0,
};
const analyticHz = cantileverAnalyticFirstFrequencyHz({
  lengthM: reference.lengthM,
  flexuralRigidityNm2: reference.flexuralRigidityNm2,
  structuralMassPerM: reference.structuralMassPerM,
});
const meshCounts = [8, 16, 32];
const frequencyResults = meshCounts.map((segmentCount) => {
  const system = createNonlinearRod({ ...reference, segmentCount });
  const firstHz = naturalFrequenciesHz(
    linearizedAngleSystemAtStraight(system),
    1,
  )[0];
  return {
    segmentCount,
    firstHz,
    relativeError: Math.abs(firstHz - analyticHz) / analyticHz,
  };
});
for (let i = 1; i < frequencyResults.length; i += 1) {
  assert.ok(
    frequencyResults[i].relativeError < frequencyResults[i - 1].relativeError,
    "nonlinear rod small-angle first frequency should converge with mesh refinement",
  );
}
assert.ok(
  frequencyResults.at(-1).relativeError < 0.04,
  "32-segment nonlinear rod should be within 4% of the small-angle analytical first mode",
);

// Large-rotation static solve under gravity + a horizontal tip load.
// The result should converge without invoking a small-angle geometry.
const staticResults = [8, 16, 24].map((segmentCount) => {
  const system = createNonlinearRod({
    segmentCount,
    gravityMps2: 9.81,
    headMassKg: 0.20,
    headRotInertiaKgM2: 0.002,
    headComAxialOffsetM: 0.055,
  });
  const result = solveStaticRodEquilibrium(system, {
    tipLoad: {
      forceXYN: [2.0, 0],
      momentNm: 0,
    },
  });
  assert.ok(result.converged, "finite-rotation static solve must converge");
  assert.ok(
    result.residualNorm < 1e-8,
    "finite-rotation equilibrium residual must be small",
  );
  return {
    segmentCount,
    tip: result.kinematics.tip,
    tipAngleRad: result.kinematics.tipAngleRad,
  };
});

const static24 = staticResults.at(-1);
assert.ok(
  static24.tip[0] > 0.35,
  "2 N lateral load should create a clearly finite transverse displacement",
);
assert.ok(
  Math.abs(static24.tipAngleRad) > 0.45,
  "static case should exceed the old small-angle regime",
);
assert.ok(
  Math.abs(staticResults[1].tip[0] - static24.tip[0])
    / Math.abs(static24.tip[0]) < 0.03,
  "16 -> 24 segment nonlinear static tip displacement should be converged within 3%",
);
assert.ok(
  Math.abs(staticResults[1].tipAngleRad - static24.tipAngleRad)
    / Math.abs(static24.tipAngleRad) < 0.02,
  "16 -> 24 segment nonlinear static tip angle should be converged within 2%",
);

// With gravity and no lateral load the hanging state remains exactly straight.
const hanging = createNonlinearRod({
  segmentCount: 12,
  gravityMps2: 9.81,
});
const hangingEquilibrium = solveStaticRodEquilibrium(hanging);
assert.ok(hangingEquilibrium.converged);
assert.ok(
  hangingEquilibrium.anglesRad.every((angle) => Math.abs(angle) < 1e-12),
  "straight hanging rod should remain an equilibrium under gravity",
);

// Dry conservative transient: RK4 should keep mechanical energy nearly
// constant for a finite-rotation perturbation.
const conservative = createNonlinearRod({
  segmentCount: 8,
  gravityMps2: 0,
  rayleighMassPerS: 0,
  rayleighStiffnessS: 0,
  headMassKg: 0.20,
  headRotInertiaKgM2: 0.002,
});
let state = straightRodState(conservative);
state = {
  anglesRad: state.anglesRad.map(
    (value, i) => (
      i === 0
        ? value
        : 0.22 * i / (conservative.params.segmentCount - 1)
    ),
  ),
  angularRatesRadS: [...state.angularRatesRadS],
};
const energy0 = totalMechanicalEnergy(conservative, state);
let minEnergy = energy0;
let maxEnergy = energy0;
const dt = 0.001;
for (let i = 0; i < 1500; i += 1) {
  state = stepNonlinearRodRK4(conservative, state, dt);
  const energy = totalMechanicalEnergy(conservative, state);
  assert.ok(Number.isFinite(energy), "nonlinear dry energy must stay finite");
  minEnergy = Math.min(minEnergy, energy);
  maxEnergy = Math.max(maxEnergy, energy);
}
assert.ok(
  (maxEnergy - minEnergy) / energy0 < 0.006,
  "finite-rotation dry transient should conserve energy to the RK4 regression tolerance",
);

console.log(
  [
    "H1-4B nonlinear rod checks OK:",
    `analytic f1=${analyticHz.toFixed(5)} Hz`,
    `32-seg f1=${frequencyResults.at(-1).firstHz.toFixed(5)} Hz`,
    `error=${(100 * frequencyResults.at(-1).relativeError).toFixed(2)}%`,
    `large tip x=${static24.tip[0].toFixed(3)} m`,
    `large tip angle=${(static24.tipAngleRad * 180 / Math.PI).toFixed(1)} deg`,
    `energy band=${((maxEnergy - minEnergy) / energy0).toExponential(2)}`,
  ].join(" "),
);
