import {
  matrixLinearCombination,
  zeros,
} from "./linear-algebra.js";
import {
  DEFAULT_DRY_BEAM_PARAMS,
} from "./assemble.js";
import { hermiteBeamElementMatrices } from "./beam-element.js";

export const DEFAULT_CONVEYING_FLOW_PARAMS = Object.freeze({
  ...DEFAULT_DRY_BEAM_PARAMS,
  waterDensityKgM3: 997,
  hoseInnerDiameterM: 0.006,
  flowSpeedMps: 0,
});

export function hoseFlowAreaM2(innerDiameterM) {
  if (!(innerDiameterM > 0)) {
    throw new RangeError("innerDiameterM must be positive");
  }
  return Math.PI * innerDiameterM * innerDiameterM / 4;
}

export function flowSpeedFromRateM3s(flowRateM3s, innerDiameterM) {
  return flowRateM3s / hoseFlowAreaM2(innerDiameterM);
}

export function flowRateM3sFromSpeed(flowSpeedMps, innerDiameterM) {
  return flowSpeedMps * hoseFlowAreaM2(innerDiameterM);
}

function scaleMatrix(matrix, scale) {
  return matrix.map((row) => row.map((value) => value * scale));
}

export function conveyingFlowElementMatrices({
  lengthM,
  fluidMassPerM,
  flowSpeedMps,
}) {
  if (!(lengthM > 0)) throw new RangeError("lengthM must be positive");
  if (!(fluidMassPerM >= 0)) {
    throw new RangeError("fluidMassPerM must be non-negative");
  }
  if (!Number.isFinite(flowSpeedMps)) {
    throw new RangeError("flowSpeedMps must be finite");
  }

  const l = lengthM;
  const l2 = l * l;

  // Integral N^T N ds, written in the same consistent-mass form as H1-1.
  const fluidMass = scaleMatrix([
    [156, 22 * l, 54, -13 * l],
    [22 * l, 4 * l2, 13 * l, -3 * l2],
    [54, 13 * l, 156, -22 * l],
    [-13 * l, -3 * l2, -22 * l, 4 * l2],
  ], fluidMassPerM * l / 420);

  // Integral N^T N_s ds.
  const velocityCouplingBase = [
    [-1 / 2, l / 10, 1 / 2, -l / 10],
    [-l / 10, 0, l / 10, -l2 / 60],
    [-1 / 2, -l / 10, 1 / 2, l / 10],
    [l / 10, l2 / 60, -l / 10, 0],
  ];

  // Integral N^T N_ss ds.
  const speedSquaredCouplingBase = [
    [-6 / (5 * l), -11 / 10, 6 / (5 * l), -1 / 10],
    [-1 / 10, -2 * l / 15, 1 / 10, l / 30],
    [6 / (5 * l), 1 / 10, -6 / (5 * l), 11 / 10],
    [-1 / 10, l / 30, 1 / 10, -2 * l / 15],
  ];

  const velocityCoupling = scaleMatrix(
    velocityCouplingBase,
    2 * fluidMassPerM * flowSpeedMps,
  );
  const speedSquaredStiffness = scaleMatrix(
    speedSquaredCouplingBase,
    fluidMassPerM * flowSpeedMps * flowSpeedMps,
  );

  return {
    fluidMass,
    velocityCoupling,
    speedSquaredStiffness,
  };
}

function addElementMatrix(globalMatrix, elementMatrix, dofIndices) {
  for (let i = 0; i < dofIndices.length; i += 1) {
    for (let j = 0; j < dofIndices.length; j += 1) {
      globalMatrix[dofIndices[i]][dofIndices[j]] += elementMatrix[i][j];
    }
  }
}

function submatrix(matrix, indices) {
  return indices.map((i) => indices.map((j) => matrix[i][j]));
}

function validateParams(params) {
  if (!(params.lengthM > 0)) throw new RangeError("lengthM must be positive");
  if (!Number.isInteger(params.elementCount) || params.elementCount < 1) {
    throw new RangeError("elementCount must be a positive integer");
  }
  if (!(params.flexuralRigidityNm2 > 0)) {
    throw new RangeError("flexuralRigidityNm2 must be positive");
  }
  if (!(params.structuralMassPerM > 0)) {
    throw new RangeError("structuralMassPerM must be positive");
  }
  if (!(params.waterDensityKgM3 > 0)) {
    throw new RangeError("waterDensityKgM3 must be positive");
  }
  if (!(params.hoseInnerDiameterM > 0)) {
    throw new RangeError("hoseInnerDiameterM must be positive");
  }
  if (!Number.isFinite(params.flowSpeedMps)) {
    throw new RangeError("flowSpeedMps must be finite");
  }
  if (params.tipMassKg < 0 || params.tipRotInertiaKgM2 < 0) {
    throw new RangeError("tip inertias must be non-negative");
  }
  if (params.rayleighMassPerS < 0 || params.rayleighStiffnessS < 0) {
    throw new RangeError("Rayleigh damping coefficients must be non-negative");
  }
}

export function assembleConveyingFluidBeam(overrides = {}) {
  const params = { ...DEFAULT_CONVEYING_FLOW_PARAMS, ...overrides };
  validateParams(params);

  const nodeCount = params.elementCount + 1;
  const dofCount = 2 * nodeCount;
  const elementLengthM = params.lengthM / params.elementCount;
  const flowAreaM2 = hoseFlowAreaM2(params.hoseInnerDiameterM);
  const fluidMassPerM = params.waterDensityKgM3 * flowAreaM2;

  const structuralMass = zeros(dofCount);
  const fluidMass = zeros(dofCount);
  const bendingStiffness = zeros(dofCount);
  const velocityCoupling = zeros(dofCount);
  const speedSquaredStiffness = zeros(dofCount);

  const structuralElement = hermiteBeamElementMatrices({
    lengthM: elementLengthM,
    flexuralRigidityNm2: params.flexuralRigidityNm2,
    massPerLengthKgM: params.structuralMassPerM,
  });
  const flowElement = conveyingFlowElementMatrices({
    lengthM: elementLengthM,
    fluidMassPerM,
    flowSpeedMps: params.flowSpeedMps,
  });

  for (let e = 0; e < params.elementCount; e += 1) {
    const dofIndices = [
      2 * e,
      2 * e + 1,
      2 * (e + 1),
      2 * (e + 1) + 1,
    ];
    addElementMatrix(structuralMass, structuralElement.mass, dofIndices);
    addElementMatrix(bendingStiffness, structuralElement.stiffness, dofIndices);
    addElementMatrix(fluidMass, flowElement.fluidMass, dofIndices);
    addElementMatrix(
      velocityCoupling,
      flowElement.velocityCoupling,
      dofIndices,
    );
    addElementMatrix(
      speedSquaredStiffness,
      flowElement.speedSquaredStiffness,
      dofIndices,
    );
  }

  const totalMass = matrixLinearCombination([
    { scale: 1, matrix: structuralMass },
    { scale: 1, matrix: fluidMass },
  ]);

  const tipDisplacementDof = dofCount - 2;
  const tipRotationDof = dofCount - 1;
  totalMass[tipDisplacementDof][tipDisplacementDof] += params.tipMassKg;
  totalMass[tipRotationDof][tipRotationDof] += params.tipRotInertiaKgM2;

  // Educational damping model. The mass-proportional term uses the filled-hose
  // inertia, while stiffness-proportional damping uses the dry bending matrix.
  const structuralDamping = matrixLinearCombination([
    { scale: params.rayleighMassPerS, matrix: totalMass },
    { scale: params.rayleighStiffnessS, matrix: bendingStiffness },
  ]);

  const effectiveDamping = matrixLinearCombination([
    { scale: 1, matrix: structuralDamping },
    { scale: 1, matrix: velocityCoupling },
  ]);
  const effectiveStiffness = matrixLinearCombination([
    { scale: 1, matrix: bendingStiffness },
    { scale: 1, matrix: speedSquaredStiffness },
  ]);

  const freeDofIndices = Array.from(
    { length: dofCount - 2 },
    (_, i) => i + 2,
  );
  const reduce = (matrix) => submatrix(matrix, freeDofIndices);

  return {
    params,
    nodeCount,
    dofCount,
    elementLengthM,
    flowAreaM2,
    fluidMassPerM,
    flowRateM3s: flowRateM3sFromSpeed(
      params.flowSpeedMps,
      params.hoseInnerDiameterM,
    ),
    freeDofIndices,
    full: {
      structuralMass,
      fluidMass,
      mass: totalMass,
      bendingStiffness,
      structuralDamping,
      velocityCoupling,
      speedSquaredStiffness,
      damping: effectiveDamping,
      stiffness: effectiveStiffness,
    },
    reduced: {
      structuralMass: reduce(structuralMass),
      fluidMass: reduce(fluidMass),
      mass: reduce(totalMass),
      bendingStiffness: reduce(bendingStiffness),
      structuralDamping: reduce(structuralDamping),
      velocityCoupling: reduce(velocityCoupling),
      speedSquaredStiffness: reduce(speedSquaredStiffness),
      damping: reduce(effectiveDamping),
      stiffness: reduce(effectiveStiffness),
    },
  };
}
