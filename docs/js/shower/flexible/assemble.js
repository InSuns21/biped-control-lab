import {
  matrixLinearCombination,
  zeros,
} from "./linear-algebra.js";
import { hermiteBeamElementMatrices } from "./beam-element.js";

export const DEFAULT_DRY_BEAM_PARAMS = Object.freeze({
  lengthM: 1.2,
  elementCount: 8,
  flexuralRigidityNm2: 0.7,
  structuralMassPerM: 0.25,
  tipMassKg: 0.20,
  tipRotInertiaKgM2: 0.002,
  rayleighMassPerS: 0.08,
  rayleighStiffnessS: 0.0002,
});

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
  if (params.tipMassKg < 0 || params.tipRotInertiaKgM2 < 0) {
    throw new RangeError("tip inertias must be non-negative");
  }
  if (params.rayleighMassPerS < 0 || params.rayleighStiffnessS < 0) {
    throw new RangeError("Rayleigh damping coefficients must be non-negative");
  }
}

export function assembleDryBeam(overrides = {}) {
  const params = { ...DEFAULT_DRY_BEAM_PARAMS, ...overrides };
  validateParams(params);

  const nodeCount = params.elementCount + 1;
  const dofCount = 2 * nodeCount;
  const elementLengthM = params.lengthM / params.elementCount;
  const mass = zeros(dofCount);
  const stiffness = zeros(dofCount);

  const element = hermiteBeamElementMatrices({
    lengthM: elementLengthM,
    flexuralRigidityNm2: params.flexuralRigidityNm2,
    massPerLengthKgM: params.structuralMassPerM,
  });

  for (let e = 0; e < params.elementCount; e += 1) {
    const dofIndices = [
      2 * e,
      2 * e + 1,
      2 * (e + 1),
      2 * (e + 1) + 1,
    ];
    addElementMatrix(mass, element.mass, dofIndices);
    addElementMatrix(stiffness, element.stiffness, dofIndices);
  }

  const tipDisplacementDof = dofCount - 2;
  const tipRotationDof = dofCount - 1;
  mass[tipDisplacementDof][tipDisplacementDof] += params.tipMassKg;
  mass[tipRotationDof][tipRotationDof] += params.tipRotInertiaKgM2;

  const damping = matrixLinearCombination([
    { scale: params.rayleighMassPerS, matrix: mass },
    { scale: params.rayleighStiffnessS, matrix: stiffness },
  ]);

  // H1-1 validation uses a clamped base: y_0 = theta_0 = 0.
  const freeDofIndices = Array.from(
    { length: dofCount - 2 },
    (_, i) => i + 2,
  );

  return {
    params,
    nodeCount,
    dofCount,
    elementLengthM,
    freeDofIndices,
    full: { mass, damping, stiffness },
    reduced: {
      mass: submatrix(mass, freeDofIndices),
      damping: submatrix(damping, freeDofIndices),
      stiffness: submatrix(stiffness, freeDofIndices),
    },
  };
}

export function dryStaticTipShapeReduced(system, tipDisplacementM) {
  const q = [];
  const L = system.params.lengthM;

  for (let node = 1; node < system.nodeCount; node += 1) {
    const x = node * system.elementLengthM;
    const xi = x / L;
    const shape = 0.5 * xi * xi * (3 - xi);
    const slope = 1.5 * xi * (2 - xi) / L;
    q.push(tipDisplacementM * shape);
    q.push(tipDisplacementM * slope);
  }

  return q;
}
