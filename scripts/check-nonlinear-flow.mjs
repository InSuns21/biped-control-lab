import assert from "node:assert/strict";
import {
  EigenvalueDecomposition,
  Matrix,
} from "ml-matrix";
import {
  createNonlinearRod,
  rodAcceleration,
} from "../docs/js/shower/flexible/nonlinear-rod.js";
import {
  createConveyingFlowGeneralizedForce,
  generalizedCentrifugalFlowForce,
  generalizedCoriolisFlowForce,
} from "../docs/js/shower/flexible/nonlinear-flow.js";

const rho = 997;
const diameterM = 0.006;
const areaM2 = Math.PI * diameterM * diameterM / 4;
const fluidMassPerM = rho * areaM2;
const nearly = (a, b, tol = 1e-10) => Math.abs(a - b) <= tol;

function baseSystem(segmentCount) {
  return createNonlinearRod({
    segmentCount,
    lengthM: 1.2,
    flexuralRigidityNm2: 0.7,
    structuralMassPerM: 0.25,
    fluidMassPerM,
    gravityMps2: 0,
    headMassKg: 0.20,
    headRotInertiaKgM2: 0.002,
    headComAxialOffsetM: 0,
    rayleighMassPerS: 0.08,
    rayleighStiffnessS: 0.0002,
  });
}

function linearizedStateMatrix(system, flowSpeedMps) {
  const count = system.params.segmentCount;
  const free = system.freeAngleIndices;
  const n = free.length;
  const angleStep = 1e-6;
  const rateStep = 1e-6;
  const flowForce = createConveyingFlowGeneralizedForce({
    flowSpeedMps,
    fluidMassPerM,
  });

  const accelerationAt = (anglesRad, angularRatesRadS) => rodAcceleration(
    system,
    { anglesRad, angularRatesRadS },
    null,
    flowForce,
  );

  const aqq = Array.from({ length: n }, () => Array(n).fill(0));
  const avv = Array.from({ length: n }, () => Array(n).fill(0));

  for (let column = 0; column < n; column += 1) {
    const dof = free[column];

    const plusQ = Array(count).fill(0);
    const minusQ = Array(count).fill(0);
    plusQ[dof] = angleStep;
    minusQ[dof] = -angleStep;
    const plusQA = accelerationAt(plusQ, Array(count).fill(0));
    const minusQA = accelerationAt(minusQ, Array(count).fill(0));

    const plusV = Array(count).fill(0);
    const minusV = Array(count).fill(0);
    plusV[dof] = rateStep;
    minusV[dof] = -rateStep;
    const plusVA = accelerationAt(Array(count).fill(0), plusV);
    const minusVA = accelerationAt(Array(count).fill(0), minusV);

    for (let row = 0; row < n; row += 1) {
      const rowDof = free[row];
      aqq[row][column] = (
        plusQA[rowDof] - minusQA[rowDof]
      ) / (2 * angleStep);
      avv[row][column] = (
        plusVA[rowDof] - minusVA[rowDof]
      ) / (2 * rateStep);
    }
  }

  const state = Array.from(
    { length: 2 * n },
    () => Array(2 * n).fill(0),
  );
  for (let i = 0; i < n; i += 1) {
    state[i][n + i] = 1;
    for (let j = 0; j < n; j += 1) {
      state[n + i][j] = aqq[i][j];
      state[n + i][n + j] = avv[i][j];
    }
  }
  return state;
}

function eigenSummary(system, flowSpeedMps) {
  const evd = new EigenvalueDecomposition(
    new Matrix(linearizedStateMatrix(system, flowSpeedMps)),
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

function criticalSpeed(segmentCount) {
  const system = baseSystem(segmentCount);
  let previousU = 0;
  let previous = eigenSummary(system, 0);
  assert.ok(previous.maxReal < 0, "zero-flow nonlinear rod should be damped");

  let bracket = null;
  for (let u = 0.5; u <= 16; u += 0.5) {
    const current = eigenSummary(system, u);
    if (previous.maxReal <= 0 && current.maxReal > 0) {
      bracket = [previousU, u];
      break;
    }
    previousU = u;
    previous = current;
  }
  assert.ok(bracket, "nonlinear rod linearization should cross into flutter");

  let [low, high] = bracket;
  for (let i = 0; i < 22; i += 1) {
    const mid = 0.5 * (low + high);
    if (eigenSummary(system, mid).maxReal > 0) high = mid;
    else low = mid;
  }
  const speed = 0.5 * (low + high);
  return {
    speed,
    ...eigenSummary(system, speed),
  };
}

// Finite-angle parity checks.
const paritySystem = baseSystem(8);
const state = {
  anglesRad: [0, 0.05, -0.08, 0.11, -0.03, 0.16, 0.09, -0.04],
  angularRatesRadS: [0, 0.2, -0.4, 0.1, 0.3, -0.2, 0.5, -0.1],
};
const cPlus = generalizedCoriolisFlowForce(paritySystem, state, {
  flowSpeedMps: 4,
  fluidMassPerM,
});
const cMinus = generalizedCoriolisFlowForce(paritySystem, state, {
  flowSpeedMps: -4,
  fluidMassPerM,
});
const kPlus = generalizedCentrifugalFlowForce(paritySystem, state, {
  flowSpeedMps: 4,
  fluidMassPerM,
});
const kMinus = generalizedCentrifugalFlowForce(paritySystem, state, {
  flowSpeedMps: -4,
  fluidMassPerM,
});

for (let i = 0; i < cPlus.length; i += 1) {
  assert.ok(
    nearly(cPlus[i], -cMinus[i], 1e-12),
    "finite-angle Coriolis force must be odd in U",
  );
  assert.ok(
    nearly(kPlus[i], kMinus[i], 1e-12),
    "finite-angle centrifugal force must be even in U",
  );
}

const straightState = {
  anglesRad: Array(8).fill(0),
  angularRatesRadS: Array(8).fill(0),
};
assert.ok(
  generalizedCentrifugalFlowForce(paritySystem, straightState, {
    flowSpeedMps: 10,
    fluidMassPerM,
  }).every((value) => Math.abs(value) < 1e-14),
  "straight rod must have zero finite-angle U^2 curvature load",
);

// The small-angle linearization must retain the H1-2 qualitative stability
// structure and converge under rod refinement.
const critical8 = criticalSpeed(8);
const critical12 = criticalSpeed(12);
const critical16 = criticalSpeed(16);

assert.ok(
  Math.abs(critical12.speed - critical16.speed) / critical16.speed < 0.04,
  "12 -> 16 segment nonlinear-flow critical speed should converge within 4%",
);
assert.ok(
  Math.abs(critical16.speed - 9.4808) / 9.4808 < 0.20,
  "nonlinear rod small-angle critical speed should stay within 20% of the H1-2 Hermite reference",
);
assert.ok(
  Math.abs(critical16.imagAtMax) > 1,
  "nonlinear rod instability should remain oscillatory at the crossing",
);

const low = eigenSummary(baseSystem(16), 5);
const high = eigenSummary(baseSystem(16), 12);
assert.ok(low.maxReal < 0, "5 m/s nonlinear-flow linearization should be stable");
assert.ok(high.maxReal > 0, "12 m/s nonlinear-flow linearization should be unstable");

console.log(
  [
    "H1-4B nonlinear flow checks OK:",
    `Ucr8=${critical8.speed.toFixed(3)} m/s`,
    `Ucr12=${critical12.speed.toFixed(3)} m/s`,
    `Ucr16=${critical16.speed.toFixed(3)} m/s`,
    `omega=${Math.abs(critical16.imagAtMax).toFixed(3)} rad/s`,
    `low Re=${low.maxReal.toFixed(4)} 1/s`,
    `high Re=${high.maxReal.toFixed(4)} 1/s`,
  ].join(" "),
);
