import assert from "node:assert/strict";
import {
  EigenvalueDecomposition,
  Matrix,
} from "ml-matrix";
import {
  assembleConveyingFluidBeam,
} from "../docs/js/shower/flexible/conveying-flow.js";
import {
  assembleShowerHeadConveyingBeam,
  linearizeShowerHeadMomentumBoundary,
  showerHeadMomentumReaction2D,
  showerHeadTipMassMatrix,
} from "../docs/js/shower/flexible/shower-head.js";
import {
  secondOrderStateMatrix,
} from "../docs/js/shower/flexible/stability.js";

const nearly = (a, b, tol = 1e-10) => Math.abs(a - b) <= tol;

function maxMatrixDifference(a, b) {
  let max = 0;
  for (let i = 0; i < a.length; i += 1) {
    for (let j = 0; j < a[i].length; j += 1) {
      max = Math.max(max, Math.abs(a[i][j] - b[i][j]));
    }
  }
  return max;
}

function maxAbsMatrix(matrix) {
  let max = 0;
  for (const row of matrix) {
    for (const value of row) max = Math.max(max, Math.abs(value));
  }
  return max;
}

function eigenSummary(system) {
  const evd = new EigenvalueDecomposition(
    new Matrix(secondOrderStateMatrix(system.reduced)),
  );
  const real = evd.realEigenvalues;
  const imag = evd.imaginaryEigenvalues;
  let index = 0;
  for (let i = 1; i < real.length; i += 1) {
    if (real[i] > real[index]) index = i;
  }
  return {
    maxReal: real[index],
    imagAtMaxReal: imag[index],
  };
}

function summaryAtSpeed(flowSpeedMps, elementCount = 8) {
  const system = assembleShowerHeadConveyingBeam({
    flowSpeedMps,
    elementCount,
  });
  return { system, ...eigenSummary(system) };
}

function findCriticalSpeed(elementCount = 8) {
  const step = 0.25;
  const maxSpeed = 16;
  let lowSpeed = 0;
  let lowValue = summaryAtSpeed(0, elementCount).maxReal;
  assert.ok(lowValue < 0, "zero-flow shower-head system should be stable");

  let bracket = null;
  for (let speed = step; speed <= maxSpeed + 1e-12; speed += step) {
    const current = summaryAtSpeed(speed, elementCount);
    if (lowValue <= 0 && current.maxReal > 0) {
      bracket = [lowSpeed, speed];
      break;
    }
    lowSpeed = speed;
    lowValue = current.maxReal;
  }
  assert.ok(bracket, "bent-head flow sweep should find a stability crossing");

  let [low, high] = bracket;
  for (let i = 0; i < 32; i += 1) {
    const mid = 0.5 * (low + high);
    if (summaryAtSpeed(mid, elementCount).maxReal > 0) high = mid;
    else low = mid;
  }
  const speed = 0.5 * (low + high);
  return {
    speed,
    ...summaryAtSpeed(speed, elementCount),
  };
}

// Eccentric rigid head: translational/rotational inertia must couple.
const headMass = showerHeadTipMassMatrix({
  headMassKg: 0.20,
  headRotInertiaAboutComKgM2: 0.002,
  headComAxialOffsetM: 0.055,
});
assert.ok(nearly(headMass[0][1], headMass[1][0]));
assert.ok(headMass[0][1] > 0, "axial COM offset must couple y and theta inertia");
assert.ok(
  headMass[0][0] * headMass[1][1] - headMass[0][1] ** 2 > 0,
  "rigid-head tip mass matrix must remain positive definite",
);

// Exact momentum balance for a 90-degree bend with equal inlet/outlet area.
const rho = 997;
const area = Math.PI * 0.006 ** 2 / 4;
const U = 5;
const mdot = rho * area * U;
const ninety = showerHeadMomentumReaction2D({
  flowSpeedMps: U,
  fluidDensityKgM3: rho,
  hoseAreaM2: area,
  head: {
    headMassKg: 0.20,
    headRotInertiaAboutComKgM2: 0.002,
    headComAxialOffsetM: 0.055,
    nozzleAxialOffsetM: 0.13,
    nozzleTransverseOffsetM: 0,
    outletAngleRad: Math.PI / 2,
    outletAreaRatio: 1,
  },
});
assert.ok(nearly(ninety.forceXYN[0], mdot * U, 1e-12));
assert.ok(nearly(ninety.forceXYN[1], -mdot * U, 1e-12));

// Reversing bend direction must reverse the reference transverse reaction.
const bendPlus = showerHeadMomentumReaction2D({
  flowSpeedMps: U,
  fluidDensityKgM3: rho,
  hoseAreaM2: area,
  head: {
    headMassKg: 0.20,
    headRotInertiaAboutComKgM2: 0.002,
    headComAxialOffsetM: 0.055,
    nozzleAxialOffsetM: 0.13,
    nozzleTransverseOffsetM: 0,
    outletAngleRad: Math.PI / 6,
    outletAreaRatio: 1,
  },
});
const bendMinus = showerHeadMomentumReaction2D({
  flowSpeedMps: U,
  fluidDensityKgM3: rho,
  hoseAreaM2: area,
  head: {
    headMassKg: 0.20,
    headRotInertiaAboutComKgM2: 0.002,
    headComAxialOffsetM: 0.055,
    nozzleAxialOffsetM: 0.13,
    nozzleTransverseOffsetM: 0,
    outletAngleRad: -Math.PI / 6,
    outletAreaRatio: 1,
  },
});
assert.ok(nearly(bendPlus.forceXYN[1], -bendMinus.forceXYN[1], 1e-12));

// No flow means no head momentum correction.
const noFlowBoundary = linearizeShowerHeadMomentumBoundary({
  flowSpeedMps: 0,
  fluidDensityKgM3: rho,
  hoseAreaM2: area,
});
assert.ok(noFlowBoundary.force0.every((value) => Math.abs(value) < 1e-14));
assert.ok(maxAbsMatrix(noFlowBoundary.lhsStiffness2x2) < 1e-14);

// Most important consistency check: a zero-offset COM head with straight,
// equal-area outlet must reproduce the H1-2 scalar tip-mass model exactly.
const flowSpeed = 7;
const baseline = assembleConveyingFluidBeam({
  flowSpeedMps: flowSpeed,
  tipMassKg: 0.20,
  tipRotInertiaKgM2: 0.002,
});
const straightHead = assembleShowerHeadConveyingBeam({
  flowSpeedMps: flowSpeed,
  headMassKg: 0.20,
  headRotInertiaAboutComKgM2: 0.002,
  headComAxialOffsetM: 0,
  nozzleAxialOffsetM: 0.13,
  nozzleTransverseOffsetM: 0.035,
  outletAngleRad: 0,
  outletAreaRatio: 1,
});
assert.ok(
  maxMatrixDifference(
    baseline.reduced.mass,
    straightHead.reduced.mass,
  ) < 1e-12,
  "straight-head boundary must reproduce H1-2 mass",
);
assert.ok(
  maxMatrixDifference(
    baseline.reduced.damping,
    straightHead.reduced.damping,
  ) < 1e-12,
  "straight-head boundary must reproduce H1-2 damping",
);
assert.ok(
  maxMatrixDifference(
    baseline.reduced.stiffness,
    straightHead.reduced.stiffness,
  ) < 1e-12,
  "straight equal-area outlet must add no duplicate follower stiffness",
);
assert.ok(
  straightHead.reduced.headForce0.every((value) => Math.abs(value) < 1e-14),
  "straight equal-area outlet must add no duplicate momentum force",
);

// Bent default head must create an explicit reference load and change the
// linearized boundary stiffness.
const bentAt6 = assembleShowerHeadConveyingBeam({ flowSpeedMps: 6 });
assert.ok(
  Math.abs(bentAt6.reduced.headForce0.at(-2)) > 1e-4,
  "inclined outlet must create a transverse reference reaction",
);
assert.ok(
  maxAbsMatrix(bentAt6.reduced.headBoundaryStiffness) > 1e-4,
  "inclined outlet must create a configuration-dependent boundary term",
);

// The H1-3 default remains a flow-induced oscillatory stability problem.
const critical8 = findCriticalSpeed(8);
assert.ok(
  Math.abs(critical8.maxReal) < 1e-7,
  "H1-3 critical speed should solve max Re(lambda)=0",
);
assert.ok(
  Math.abs(critical8.imagAtMaxReal) > 1,
  "H1-3 default crossing should remain oscillatory",
);

const critical6 = findCriticalSpeed(6);
assert.ok(
  Math.abs(critical6.speed - critical8.speed) / critical8.speed < 0.005,
  "6 -> 8 element H1-3 critical speed should be mesh converged",
);

const low = summaryAtSpeed(0.65 * critical8.speed);
const high = summaryAtSpeed(1.20 * critical8.speed);
assert.ok(low.maxReal < 0, "below H1-3 critical speed must be stable");
assert.ok(high.maxReal > 0, "above H1-3 critical speed must be unstable");

const flowRateLpm = critical8.system.flowRateM3s * 60000;
console.log(
  [
    "Flexible hose H1-3 checks OK:",
    `critical U=${critical8.speed.toFixed(4)} m/s`,
    `critical Q=${flowRateLpm.toFixed(2)} L/min`,
    `flutter omega=${Math.abs(critical8.imagAtMaxReal).toFixed(3)} rad/s`,
    `head Fy@6=${bentAt6.reduced.headForce0.at(-2).toFixed(4)} N`,
    `head M@6=${bentAt6.reduced.headForce0.at(-1).toFixed(4)} N m`,
  ].join(" "),
);
