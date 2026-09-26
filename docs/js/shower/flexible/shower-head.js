import {
  matrixLinearCombination,
  zeros,
} from "./linear-algebra.js";
import {
  DEFAULT_CONVEYING_FLOW_PARAMS,
  assembleConveyingFluidBeam,
} from "./conveying-flow.js";

const degToRad = (deg) => deg * Math.PI / 180;

export const DEFAULT_SHOWER_HEAD_PARAMS = Object.freeze({
  headMassKg: 0.20,
  headRotInertiaAboutComKgM2: 0.002,
  headComAxialOffsetM: 0.055,
  nozzleAxialOffsetM: 0.13,
  nozzleTransverseOffsetM: 0.035,
  outletAngleRad: degToRad(35),
  outletAreaRatio: 1.0,
});

export function showerHeadTipMassMatrix({
  headMassKg,
  headRotInertiaAboutComKgM2,
  headComAxialOffsetM,
}) {
  if (!(headMassKg >= 0)) {
    throw new RangeError("headMassKg must be non-negative");
  }
  if (!(headRotInertiaAboutComKgM2 >= 0)) {
    throw new RangeError("headRotInertiaAboutComKgM2 must be non-negative");
  }
  if (!Number.isFinite(headComAxialOffsetM)) {
    throw new RangeError("headComAxialOffsetM must be finite");
  }

  const m = headMassKg;
  const e = headComAxialOffsetM;
  return [
    [m, m * e],
    [m * e, headRotInertiaAboutComKgM2 + m * e * e],
  ];
}

function rotate2([x, y], angleRad) {
  const c = Math.cos(angleRad);
  const s = Math.sin(angleRad);
  return [c * x - s * y, s * x + c * y];
}

export function showerHeadMomentumReaction2D({
  flowSpeedMps,
  fluidDensityKgM3,
  hoseAreaM2,
  tipAngleRad = 0,
  head = DEFAULT_SHOWER_HEAD_PARAMS,
}) {
  if (!(flowSpeedMps >= 0)) {
    throw new RangeError("H1-3 reference model requires non-negative flowSpeedMps");
  }
  if (!(fluidDensityKgM3 > 0)) {
    throw new RangeError("fluidDensityKgM3 must be positive");
  }
  if (!(hoseAreaM2 > 0)) {
    throw new RangeError("hoseAreaM2 must be positive");
  }
  if (!(head.outletAreaRatio > 0)) {
    throw new RangeError("outletAreaRatio must be positive");
  }

  const outletAreaM2 = hoseAreaM2 * head.outletAreaRatio;
  const volumeFlowM3s = hoseAreaM2 * flowSpeedMps;
  const massFlowKgS = fluidDensityKgM3 * volumeFlowM3s;
  const outletSpeedMps = volumeFlowM3s / outletAreaM2;

  const inletDirection = [
    Math.cos(tipAngleRad),
    Math.sin(tipAngleRad),
  ];
  const outletDirection = [
    Math.cos(tipAngleRad + head.outletAngleRad),
    Math.sin(tipAngleRad + head.outletAngleRad),
  ];

  // Increment relative to the straight-through reference already represented
  // by the H1-2 conveying-pipe model. At phi=0 and equal inlet/outlet areas
  // this correction is identically zero.
  const forceXYN = [
    massFlowKgS * (
      flowSpeedMps * inletDirection[0]
      - outletSpeedMps * outletDirection[0]
    ),
    massFlowKgS * (
      flowSpeedMps * inletDirection[1]
      - outletSpeedMps * outletDirection[1]
    ),
  ];

  const nozzleOffsetWorldM = rotate2(
    [head.nozzleAxialOffsetM, head.nozzleTransverseOffsetM],
    tipAngleRad,
  );
  const momentNm = nozzleOffsetWorldM[0] * forceXYN[1]
    - nozzleOffsetWorldM[1] * forceXYN[0];

  return {
    outletAreaM2,
    volumeFlowM3s,
    massFlowKgS,
    outletSpeedMps,
    inletDirection,
    outletDirection,
    forceXYN,
    nozzleOffsetWorldM,
    generalizedForce: [forceXYN[1], momentNm],
  };
}

export function linearizeShowerHeadMomentumBoundary({
  flowSpeedMps,
  fluidDensityKgM3,
  hoseAreaM2,
  head = DEFAULT_SHOWER_HEAD_PARAMS,
  angleStepRad = 1e-6,
}) {
  const atZero = showerHeadMomentumReaction2D({
    flowSpeedMps,
    fluidDensityKgM3,
    hoseAreaM2,
    tipAngleRad: 0,
    head,
  });
  const plus = showerHeadMomentumReaction2D({
    flowSpeedMps,
    fluidDensityKgM3,
    hoseAreaM2,
    tipAngleRad: angleStepRad,
    head,
  });
  const minus = showerHeadMomentumReaction2D({
    flowSpeedMps,
    fluidDensityKgM3,
    hoseAreaM2,
    tipAngleRad: -angleStepRad,
    head,
  });

  const derivative = plus.generalizedForce.map(
    (value, i) => (value - minus.generalizedForce[i]) / (2 * angleStepRad),
  );

  // Q_head(theta) ~= Q0 + dQ/dtheta * theta.
  // Move the linearized configuration-dependent load to the left:
  // K_head q = -dQ/dtheta * theta.
  const lhsStiffness2x2 = [
    [0, -derivative[0]],
    [0, -derivative[1]],
  ];

  return {
    force0: [...atZero.generalizedForce],
    derivativeWrtTipAngle: derivative,
    lhsStiffness2x2,
    exactAtReference: atZero,
  };
}

function cloneMatrix(matrix) {
  return matrix.map((row) => [...row]);
}

function addTipBlock(matrix, block) {
  const n = matrix.length;
  const start = n - 2;
  for (let i = 0; i < 2; i += 1) {
    for (let j = 0; j < 2; j += 1) {
      matrix[start + i][start + j] += block[i][j];
    }
  }
}

function submatrix(matrix, indices) {
  return indices.map((i) => indices.map((j) => matrix[i][j]));
}

function reduceVector(vector, indices) {
  return indices.map((i) => vector[i]);
}

export function assembleShowerHeadConveyingBeam(overrides = {}) {
  const params = {
    ...DEFAULT_CONVEYING_FLOW_PARAMS,
    ...DEFAULT_SHOWER_HEAD_PARAMS,
    ...overrides,
  };
  const head = {
    headMassKg: params.headMassKg,
    headRotInertiaAboutComKgM2: params.headRotInertiaAboutComKgM2,
    headComAxialOffsetM: params.headComAxialOffsetM,
    nozzleAxialOffsetM: params.nozzleAxialOffsetM,
    nozzleTransverseOffsetM: params.nozzleTransverseOffsetM,
    outletAngleRad: params.outletAngleRad,
    outletAreaRatio: params.outletAreaRatio,
  };

  // H1-2's scalar tip mass is disabled here and replaced by the consistent
  // eccentric rigid-head 2x2 boundary mass matrix below.
  const base = assembleConveyingFluidBeam({
    ...params,
    tipMassKg: 0,
    tipRotInertiaKgM2: 0,
  });

  const headMass2x2 = showerHeadTipMassMatrix(head);
  const headMass = zeros(base.dofCount);
  addTipBlock(headMass, headMass2x2);

  const mass = cloneMatrix(base.full.mass);
  addTipBlock(mass, headMass2x2);

  const headBoundary = linearizeShowerHeadMomentumBoundary({
    flowSpeedMps: params.flowSpeedMps,
    fluidDensityKgM3: params.waterDensityKgM3,
    hoseAreaM2: base.flowAreaM2,
    head,
  });
  const headBoundaryStiffness = zeros(base.dofCount);
  addTipBlock(headBoundaryStiffness, headBoundary.lhsStiffness2x2);

  const structuralDamping = matrixLinearCombination([
    { scale: params.rayleighMassPerS, matrix: mass },
    { scale: params.rayleighStiffnessS, matrix: base.full.bendingStiffness },
  ]);
  const damping = matrixLinearCombination([
    { scale: 1, matrix: structuralDamping },
    { scale: 1, matrix: base.full.velocityCoupling },
  ]);
  const stiffness = matrixLinearCombination([
    { scale: 1, matrix: base.full.bendingStiffness },
    { scale: 1, matrix: base.full.speedSquaredStiffness },
    { scale: 1, matrix: headBoundaryStiffness },
  ]);

  const force0 = Array(base.dofCount).fill(0);
  force0[base.dofCount - 2] = headBoundary.force0[0];
  force0[base.dofCount - 1] = headBoundary.force0[1];

  const reduce = (matrix) => submatrix(matrix, base.freeDofIndices);

  return {
    ...base,
    params,
    head,
    headBoundary,
    full: {
      ...base.full,
      headMass,
      mass,
      headBoundaryStiffness,
      structuralDamping,
      damping,
      stiffness,
      headForce0: force0,
    },
    reduced: {
      ...base.reduced,
      headMass: reduce(headMass),
      mass: reduce(mass),
      headBoundaryStiffness: reduce(headBoundaryStiffness),
      structuralDamping: reduce(structuralDamping),
      damping: reduce(damping),
      stiffness: reduce(stiffness),
      headForce0: reduceVector(force0, base.freeDofIndices),
    },
  };
}
