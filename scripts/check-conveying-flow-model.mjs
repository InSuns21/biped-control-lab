import assert from "node:assert/strict";
import {
  EigenvalueDecomposition,
  Matrix,
} from "ml-matrix";
import {
  assembleConveyingFluidBeam,
  conveyingFlowElementMatrices,
  flowRateM3sFromSpeed,
  hoseFlowAreaM2,
} from "../docs/js/shower/flexible/conveying-flow.js";
import {
  dryStaticTipShapeReduced,
} from "../docs/js/shower/flexible/assemble.js";
import {
  createNewmarkGeneralLinear,
} from "../docs/js/shower/flexible/integrator.js";
import {
  secondOrderStateMatrix,
} from "../docs/js/shower/flexible/stability.js";

const nearly = (a, b, tol = 1e-12) => Math.abs(a - b) <= tol;

function maxAbsMatrix(matrix) {
  let max = 0;
  for (const row of matrix) {
    for (const value of row) max = Math.max(max, Math.abs(value));
  }
  return max;
}

function matrixDifferenceMax(a, b) {
  let max = 0;
  for (let i = 0; i < a.length; i += 1) {
    for (let j = 0; j < a[i].length; j += 1) {
      max = Math.max(max, Math.abs(a[i][j] - b[i][j]));
    }
  }
  return max;
}

function matrixSumMax(a, b) {
  let max = 0;
  for (let i = 0; i < a.length; i += 1) {
    for (let j = 0; j < a[i].length; j += 1) {
      max = Math.max(max, Math.abs(a[i][j] + b[i][j]));
    }
  }
  return max;
}

function eigenSummary(system) {
  const stateMatrix = new Matrix(secondOrderStateMatrix(system.reduced));
  const evd = new EigenvalueDecomposition(stateMatrix);
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
  const system = assembleConveyingFluidBeam({
    flowSpeedMps,
    elementCount,
  });
  return {
    system,
    ...eigenSummary(system),
  };
}

function findCriticalSpeed(elementCount = 8) {
  const scanStep = 0.25;
  const maxSpeed = 14;
  let previousSpeed = 0;
  let previous = summaryAtSpeed(previousSpeed, elementCount);

  assert.ok(
    previous.maxReal < 0,
    "filled stationary hose should be stable with the default damping",
  );

  let bracket = null;
  for (
    let speed = scanStep;
    speed <= maxSpeed + 1e-12;
    speed += scanStep
  ) {
    const current = summaryAtSpeed(speed, elementCount);
    if (previous.maxReal <= 0 && current.maxReal > 0) {
      bracket = {
        lowSpeed: previousSpeed,
        highSpeed: speed,
        lowValue: previous.maxReal,
        highValue: current.maxReal,
      };
      break;
    }
    previousSpeed = speed;
    previous = current;
  }

  assert.ok(bracket, "flow sweep should find a stability crossing");

  let low = bracket.lowSpeed;
  let high = bracket.highSpeed;
  for (let i = 0; i < 30; i += 1) {
    const mid = 0.5 * (low + high);
    if (summaryAtSpeed(mid, elementCount).maxReal > 0) high = mid;
    else low = mid;
  }

  const criticalSpeedMps = 0.5 * (low + high);
  const critical = summaryAtSpeed(criticalSpeedMps, elementCount);
  return {
    criticalSpeedMps,
    maxReal: critical.maxReal,
    imagAtCritical: critical.imagAtMaxReal,
    flowRateLpm: flowRateM3sFromSpeed(
      criticalSpeedMps,
      critical.system.params.hoseInnerDiameterM,
    ) * 60000,
  };
}

function timeHistoryTipRms({
  flowSpeedMps,
  dt,
  durationS,
  windowS = 2,
}) {
  const system = assembleConveyingFluidBeam({ flowSpeedMps });
  const integrator = createNewmarkGeneralLinear(system.reduced, dt);
  const q0 = dryStaticTipShapeReduced(system, 0.005);
  let state = integrator.initialize({
    q: q0,
    v: Array(q0.length).fill(0),
  });

  const windowSamples = Math.round(windowS / dt);
  const first = [];
  const last = [];

  for (let i = 0; i < Math.round(durationS / dt); i += 1) {
    state = integrator.step(state);
    const tip = state.q.at(-2);
    assert.ok(Number.isFinite(tip), "tip displacement must stay finite");

    if (i < windowSamples) first.push(tip);
    if (i >= Math.round(durationS / dt) - windowSamples) last.push(tip);
  }

  const rms = (values) => Math.sqrt(
    values.reduce((sum, value) => sum + value * value, 0) / values.length,
  );

  return {
    firstRms: rms(first),
    lastRms: rms(last),
  };
}

// H1-2 element parity under flow reversal.
const elementParams = {
  lengthM: 0.15,
  fluidMassPerM: 0.04,
};
const plus = conveyingFlowElementMatrices({
  ...elementParams,
  flowSpeedMps: 3,
});
const minus = conveyingFlowElementMatrices({
  ...elementParams,
  flowSpeedMps: -3,
});
const zero = conveyingFlowElementMatrices({
  ...elementParams,
  flowSpeedMps: 0,
});

assert.ok(
  maxAbsMatrix(zero.velocityCoupling) < 1e-14,
  "U = 0 must remove the velocity-linear flow coupling",
);
assert.ok(
  maxAbsMatrix(zero.speedSquaredStiffness) < 1e-14,
  "U = 0 must remove the U^2 flow coupling",
);
assert.ok(
  matrixSumMax(plus.velocityCoupling, minus.velocityCoupling) < 1e-12,
  "G_flow(-U) must equal -G_flow(U)",
);
assert.ok(
  matrixDifferenceMax(
    plus.speedSquaredStiffness,
    minus.speedSquaredStiffness,
  ) < 1e-12,
  "K_flow(-U) must equal K_flow(U)",
);
assert.ok(
  matrixDifferenceMax(plus.fluidMass, minus.fluidMass) < 1e-12,
  "fluid added mass must not depend on flow direction",
);

// A stationary filled hose still carries fluid inertia even though the
// velocity-dependent couplings vanish.
const stationaryFilled = assembleConveyingFluidBeam({ flowSpeedMps: 0 });
assert.ok(
  maxAbsMatrix(stationaryFilled.reduced.fluidMass) > 0,
  "U = 0 filled hose should retain internal-fluid mass",
);
assert.ok(
  maxAbsMatrix(stationaryFilled.reduced.velocityCoupling) < 1e-14,
);
assert.ok(
  maxAbsMatrix(stationaryFilled.reduced.speedSquaredStiffness) < 1e-14,
);

// Eigenvalue sweep: stable -> near critical -> unstable.
const low = summaryAtSpeed(6);
const high = summaryAtSpeed(12);
assert.ok(
  low.maxReal < -0.02,
  "default low-flow case should have decaying modes",
);
assert.ok(
  high.maxReal > 0.05,
  "default high-flow case should have a growing mode",
);

const critical8 = findCriticalSpeed(8);
assert.ok(
  Math.abs(critical8.maxReal) < 1e-7,
  "critical-speed bisection should land near Re(lambda)=0",
);
assert.ok(
  Math.abs(critical8.imagAtCritical) > 1,
  "default instability should be oscillatory flutter, not static divergence",
);

// Critical-flow estimate must be mesh-converged enough for the later UI.
const critical4 = findCriticalSpeed(4);
const critical6 = findCriticalSpeed(6);
assert.ok(
  Math.abs(critical6.criticalSpeedMps - critical8.criticalSpeedMps)
    / critical8.criticalSpeedMps < 0.002,
  "6 -> 8 element critical speed should be mesh converged",
);
assert.ok(
  Math.abs(critical4.criticalSpeedMps - critical8.criticalSpeedMps)
    / critical8.criticalSpeedMps < 0.01,
  "4 -> 8 element critical speed should remain within 1%",
);

// Time-domain result must agree qualitatively with the eigenvalue sign.
const lowTime = timeHistoryTipRms({
  flowSpeedMps: 6,
  dt: 0.002,
  durationS: 20,
});
assert.ok(
  lowTime.lastRms < 0.1 * lowTime.firstRms,
  "below-critical perturbation should decay strongly in time domain",
);

const highTime = timeHistoryTipRms({
  flowSpeedMps: 12,
  dt: 0.002,
  durationS: 25,
});
assert.ok(
  highTime.lastRms > 2 * highTime.firstRms,
  "above-critical perturbation should grow in time domain",
);

// Newmark result should be stable under a 2x time-step refinement.
const highFine = timeHistoryTipRms({
  flowSpeedMps: 12,
  dt: 0.001,
  durationS: 15,
});
const highCoarse = timeHistoryTipRms({
  flowSpeedMps: 12,
  dt: 0.002,
  durationS: 15,
});
assert.ok(
  Math.abs(highFine.lastRms - highCoarse.lastRms) / highFine.lastRms < 0.01,
  "above-critical RMS should change by less than 1% under dt refinement",
);

const area = hoseFlowAreaM2(
  stationaryFilled.params.hoseInnerDiameterM,
);
assert.ok(nearly(
  stationaryFilled.fluidMassPerM,
  stationaryFilled.params.waterDensityKgM3 * area,
));

console.log(
  [
    "Flexible hose H1-2 checks OK:",
    `critical U=${critical8.criticalSpeedMps.toFixed(4)} m/s`,
    `critical Q=${critical8.flowRateLpm.toFixed(2)} L/min`,
    `flutter omega=${Math.abs(critical8.imagAtCritical).toFixed(3)} rad/s`,
    `low Re(lambda)=${low.maxReal.toFixed(4)} 1/s`,
    `high Re(lambda)=${high.maxReal.toFixed(4)} 1/s`,
    `time RMS low ratio=${(lowTime.lastRms / lowTime.firstRms).toExponential(2)}`,
    `high ratio=${(highTime.lastRms / highTime.firstRms).toFixed(2)}`,
  ].join(" "),
);
