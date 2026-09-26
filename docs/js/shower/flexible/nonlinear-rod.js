import {
  solveLinear,
  zeros,
} from "./linear-algebra.js";

export const DEFAULT_NONLINEAR_ROD_PARAMS = Object.freeze({
  lengthM: 1.2,
  segmentCount: 12,
  flexuralRigidityNm2: 0.7,
  structuralMassPerM: 0.25,
  fluidMassPerM: 0,
  gravityMps2: 9.81,
  headMassKg: 0.20,
  headRotInertiaKgM2: 0.002,
  headComAxialOffsetM: 0.055,
  bendingDampingNms: 0.002,
  baseAngleRad: 0,
});

function validateParams(params) {
  if (!(params.lengthM > 0)) throw new RangeError("lengthM must be positive");
  if (!Number.isInteger(params.segmentCount) || params.segmentCount < 2) {
    throw new RangeError("segmentCount must be an integer >= 2");
  }
  if (!(params.flexuralRigidityNm2 > 0)) {
    throw new RangeError("flexuralRigidityNm2 must be positive");
  }
  if (!(params.structuralMassPerM > 0)) {
    throw new RangeError("structuralMassPerM must be positive");
  }
  if (!(params.fluidMassPerM >= 0)) {
    throw new RangeError("fluidMassPerM must be non-negative");
  }
  if (!(params.gravityMps2 >= 0)) {
    throw new RangeError("gravityMps2 must be non-negative");
  }
  if (!(params.headMassKg >= 0) || !(params.headRotInertiaKgM2 >= 0)) {
    throw new RangeError("head mass/inertia must be non-negative");
  }
  if (!(params.bendingDampingNms >= 0)) {
    throw new RangeError("bendingDampingNms must be non-negative");
  }
}

function addVectors(a, b) {
  return a.map((value, i) => value + b[i]);
}

function addScaledInPlace(target, source, scale = 1) {
  for (let i = 0; i < target.length; i += 1) {
    target[i] += scale * source[i];
  }
}

function vectorNorm(values) {
  return Math.sqrt(values.reduce((sum, value) => sum + value * value, 0));
}

function submatrix(matrix, indices) {
  return indices.map((i) => indices.map((j) => matrix[i][j]));
}

function reduceVector(vector, indices) {
  return indices.map((i) => vector[i]);
}

function segmentTangent(angleRad) {
  return [Math.sin(angleRad), Math.cos(angleRad)];
}

function segmentNormalDerivative(angleRad) {
  return [Math.cos(angleRad), -Math.sin(angleRad)];
}

function pointJacobianForCoefficients(anglesRad, coefficients) {
  const count = anglesRad.length;
  const jacobian = [
    Array(count).fill(0),
    Array(count).fill(0),
  ];
  for (let k = 0; k < count; k += 1) {
    const coefficient = coefficients[k] ?? 0;
    if (coefficient === 0) continue;
    const n = segmentNormalDerivative(anglesRad[k]);
    jacobian[0][k] = coefficient * n[0];
    jacobian[1][k] = coefficient * n[1];
  }
  return jacobian;
}

function pointBiasAcceleration(anglesRad, angularRatesRadS, coefficients) {
  const acceleration = [0, 0];
  for (let k = 0; k < anglesRad.length; k += 1) {
    const coefficient = coefficients[k] ?? 0;
    if (coefficient === 0) continue;
    const tangent = segmentTangent(anglesRad[k]);
    const omega2 = angularRatesRadS[k] * angularRatesRadS[k];
    acceleration[0] -= coefficient * tangent[0] * omega2;
    acceleration[1] -= coefficient * tangent[1] * omega2;
  }
  return acceleration;
}

function addPointMassToMassMatrix(matrix, jacobian, massKg) {
  if (!(massKg > 0)) return;
  const n = matrix.length;
  for (let i = 0; i < n; i += 1) {
    for (let j = 0; j < n; j += 1) {
      matrix[i][j] += massKg * (
        jacobian[0][i] * jacobian[0][j]
        + jacobian[1][i] * jacobian[1][j]
      );
    }
  }
}

function addPointMassBias(force, jacobian, acceleration, massKg) {
  if (!(massKg > 0)) return;
  for (let i = 0; i < force.length; i += 1) {
    force[i] += massKg * (
      jacobian[0][i] * acceleration[0]
      + jacobian[1][i] * acceleration[1]
    );
  }
}

function addCartesianForceGeneralized(force, jacobian, cartesianForce) {
  for (let i = 0; i < force.length; i += 1) {
    force[i] += jacobian[0][i] * cartesianForce[0]
      + jacobian[1][i] * cartesianForce[1];
  }
}

function segmentCenterCoefficients(segmentIndex, segmentLengthM, count) {
  const coefficients = Array(count).fill(0);
  for (let k = 0; k <= segmentIndex; k += 1) {
    coefficients[k] = k < segmentIndex
      ? segmentLengthM
      : 0.5 * segmentLengthM;
  }
  return coefficients;
}

function tipCoefficients(segmentLengthM, count, axialOffsetM = 0) {
  const coefficients = Array(count).fill(segmentLengthM);
  coefficients[count - 1] += axialOffsetM;
  return coefficients;
}

export function createNonlinearRod(overrides = {}) {
  const params = {
    ...DEFAULT_NONLINEAR_ROD_PARAMS,
    ...overrides,
  };
  validateParams(params);

  const segmentLengthM = params.lengthM / params.segmentCount;
  const lineMassPerM = params.structuralMassPerM + params.fluidMassPerM;
  const segmentMassKg = lineMassPerM * segmentLengthM;
  const freeAngleIndices = Array.from(
    { length: params.segmentCount - 1 },
    (_, index) => index + 1,
  );

  return {
    params,
    segmentLengthM,
    lineMassPerM,
    segmentMassKg,
    freeAngleIndices,
  };
}

export function normalizeRodState(system, {
  anglesRad,
  angularRatesRadS,
}) {
  const count = system.params.segmentCount;
  if (anglesRad.length !== count || angularRatesRadS.length !== count) {
    throw new RangeError("rod state length must equal segmentCount");
  }
  const angles = [...anglesRad];
  const rates = [...angularRatesRadS];
  angles[0] = system.params.baseAngleRad;
  rates[0] = 0;
  return {
    anglesRad: angles,
    angularRatesRadS: rates,
  };
}

export function straightRodState(system, {
  angleRad = system.params.baseAngleRad,
} = {}) {
  return normalizeRodState(system, {
    anglesRad: Array(system.params.segmentCount).fill(angleRad),
    angularRatesRadS: Array(system.params.segmentCount).fill(0),
  });
}

export function rodKinematics(system, anglesInput) {
  const { params, segmentLengthM } = system;
  const count = params.segmentCount;
  const anglesRad = [...anglesInput];
  anglesRad[0] = params.baseAngleRad;

  const nodes = [[0, 0]];
  const segmentCenters = [];

  for (let i = 0; i < count; i += 1) {
    const tangent = segmentTangent(anglesRad[i]);
    const start = nodes[i];
    segmentCenters.push([
      start[0] + 0.5 * segmentLengthM * tangent[0],
      start[1] + 0.5 * segmentLengthM * tangent[1],
    ]);
    nodes.push([
      start[0] + segmentLengthM * tangent[0],
      start[1] + segmentLengthM * tangent[1],
    ]);
  }

  const tipTangent = segmentTangent(anglesRad.at(-1));
  const headCom = [
    nodes.at(-1)[0] + params.headComAxialOffsetM * tipTangent[0],
    nodes.at(-1)[1] + params.headComAxialOffsetM * tipTangent[1],
  ];

  return {
    anglesRad,
    nodes,
    segmentCenters,
    tip: nodes.at(-1),
    tipAngleRad: anglesRad.at(-1),
    headCom,
  };
}

export function bendingEnergy(system, anglesInput) {
  const { params, segmentLengthM } = system;
  const angles = [...anglesInput];
  angles[0] = params.baseAngleRad;
  const coefficient = params.flexuralRigidityNm2 / segmentLengthM;
  let energy = 0;
  for (let i = 1; i < angles.length; i += 1) {
    const delta = angles[i] - angles[i - 1];
    energy += 0.5 * coefficient * delta * delta;
  }
  return energy;
}

export function generalizedBendingForce(system, anglesInput) {
  const { params, segmentLengthM } = system;
  const angles = [...anglesInput];
  angles[0] = params.baseAngleRad;
  const force = Array(angles.length).fill(0);
  const coefficient = params.flexuralRigidityNm2 / segmentLengthM;

  for (let i = 1; i < angles.length; i += 1) {
    const delta = angles[i] - angles[i - 1];
    const torque = coefficient * delta;
    force[i - 1] += torque;
    force[i] -= torque;
  }

  return force;
}

export function generalizedBendingDampingForce(
  system,
  angularRatesInput,
) {
  const rates = [...angularRatesInput];
  rates[0] = 0;
  const force = Array(rates.length).fill(0);
  const coefficient = system.params.bendingDampingNms
    / system.segmentLengthM;

  for (let i = 1; i < rates.length; i += 1) {
    const relativeRate = rates[i] - rates[i - 1];
    const torque = coefficient * relativeRate;
    force[i - 1] += torque;
    force[i] -= torque;
  }

  return force;
}

export function massMatrixAndBias(
  system,
  anglesInput,
  angularRatesInput,
) {
  const { params, segmentLengthM, segmentMassKg } = system;
  const count = params.segmentCount;
  const angles = [...anglesInput];
  const rates = [...angularRatesInput];
  angles[0] = params.baseAngleRad;
  rates[0] = 0;

  const mass = zeros(count);
  const bias = Array(count).fill(0);
  const segmentRotInertiaKgM2 = segmentMassKg
    * segmentLengthM * segmentLengthM / 12;

  for (let i = 0; i < count; i += 1) {
    const coefficients = segmentCenterCoefficients(
      i,
      segmentLengthM,
      count,
    );
    const jacobian = pointJacobianForCoefficients(angles, coefficients);
    const biasAcceleration = pointBiasAcceleration(
      angles,
      rates,
      coefficients,
    );
    addPointMassToMassMatrix(mass, jacobian, segmentMassKg);
    addPointMassBias(bias, jacobian, biasAcceleration, segmentMassKg);
    mass[i][i] += segmentRotInertiaKgM2;
  }

  if (params.headMassKg > 0) {
    const coefficients = tipCoefficients(
      segmentLengthM,
      count,
      params.headComAxialOffsetM,
    );
    const jacobian = pointJacobianForCoefficients(angles, coefficients);
    const biasAcceleration = pointBiasAcceleration(
      angles,
      rates,
      coefficients,
    );
    addPointMassToMassMatrix(mass, jacobian, params.headMassKg);
    addPointMassBias(
      bias,
      jacobian,
      biasAcceleration,
      params.headMassKg,
    );
  }
  mass[count - 1][count - 1] += params.headRotInertiaKgM2;

  return { mass, bias };
}

export function generalizedGravityForce(system, anglesInput) {
  const { params, segmentLengthM, segmentMassKg } = system;
  const count = params.segmentCount;
  const angles = [...anglesInput];
  angles[0] = params.baseAngleRad;
  const force = Array(count).fill(0);

  const gravityForce = [0, segmentMassKg * params.gravityMps2];
  for (let i = 0; i < count; i += 1) {
    const coefficients = segmentCenterCoefficients(
      i,
      segmentLengthM,
      count,
    );
    const jacobian = pointJacobianForCoefficients(angles, coefficients);
    addCartesianForceGeneralized(force, jacobian, gravityForce);
  }

  if (params.headMassKg > 0) {
    const coefficients = tipCoefficients(
      segmentLengthM,
      count,
      params.headComAxialOffsetM,
    );
    const jacobian = pointJacobianForCoefficients(angles, coefficients);
    addCartesianForceGeneralized(force, jacobian, [
      0,
      params.headMassKg * params.gravityMps2,
    ]);
  }

  return force;
}

export function generalizedTipLoad(
  system,
  anglesInput,
  {
    forceXYN = [0, 0],
    momentNm = 0,
  } = {},
) {
  const { params, segmentLengthM } = system;
  const angles = [...anglesInput];
  angles[0] = params.baseAngleRad;
  const force = Array(params.segmentCount).fill(0);
  const coefficients = tipCoefficients(
    segmentLengthM,
    params.segmentCount,
    0,
  );
  const jacobian = pointJacobianForCoefficients(angles, coefficients);
  addCartesianForceGeneralized(force, jacobian, forceXYN);
  force[force.length - 1] += momentNm;
  return force;
}

export function generalizedRodForce(
  system,
  stateInput,
  tipLoad = null,
) {
  const state = normalizeRodState(system, stateInput);
  const force = generalizedBendingForce(system, state.anglesRad);
  addScaledInPlace(
    force,
    generalizedGravityForce(system, state.anglesRad),
  );
  addScaledInPlace(
    force,
    generalizedBendingDampingForce(
      system,
      state.angularRatesRadS,
    ),
  );
  if (tipLoad) {
    addScaledInPlace(
      force,
      generalizedTipLoad(system, state.anglesRad, tipLoad),
    );
  }
  force[0] = 0;
  return force;
}

export function rodAcceleration(
  system,
  stateInput,
  tipLoad = null,
) {
  const state = normalizeRodState(system, stateInput);
  const { mass, bias } = massMatrixAndBias(
    system,
    state.anglesRad,
    state.angularRatesRadS,
  );
  const generalizedForce = generalizedRodForce(
    system,
    state,
    tipLoad,
  );
  const rhs = generalizedForce.map(
    (value, i) => value - bias[i],
  );
  const free = system.freeAngleIndices;
  const freeAcceleration = solveLinear(
    submatrix(mass, free),
    reduceVector(rhs, free),
  );
  const acceleration = Array(system.params.segmentCount).fill(0);
  for (let i = 0; i < free.length; i += 1) {
    acceleration[free[i]] = freeAcceleration[i];
  }
  return acceleration;
}

export function gravitationalPotential(system, anglesInput) {
  const { params, segmentMassKg } = system;
  const kinematics = rodKinematics(system, anglesInput);
  let potential = 0;

  for (const center of kinematics.segmentCenters) {
    potential -= segmentMassKg * params.gravityMps2 * center[1];
  }
  potential -= params.headMassKg
    * params.gravityMps2
    * kinematics.headCom[1];
  return potential;
}

export function totalMechanicalEnergy(system, stateInput) {
  const state = normalizeRodState(system, stateInput);
  const { mass } = massMatrixAndBias(
    system,
    state.anglesRad,
    Array(state.angularRatesRadS.length).fill(0),
  );
  let kinetic = 0;
  for (let i = 0; i < mass.length; i += 1) {
    for (let j = 0; j < mass.length; j += 1) {
      kinetic += 0.5
        * state.angularRatesRadS[i]
        * mass[i][j]
        * state.angularRatesRadS[j];
    }
  }

  return kinetic
    + bendingEnergy(system, state.anglesRad)
    + gravitationalPotential(system, state.anglesRad);
}

function freeResidual(system, anglesInput, tipLoad) {
  const state = normalizeRodState(system, {
    anglesRad: anglesInput,
    angularRatesRadS: Array(system.params.segmentCount).fill(0),
  });
  const force = generalizedRodForce(system, state, tipLoad);
  return reduceVector(force, system.freeAngleIndices);
}

export function solveStaticRodEquilibrium(
  system,
  {
    tipLoad = null,
    initialAnglesRad = null,
    tolerance = 1e-9,
    maxIterations = 60,
    finiteDifferenceRad = 1e-6,
  } = {},
) {
  const count = system.params.segmentCount;
  let angles = initialAnglesRad
    ? [...initialAnglesRad]
    : Array(count).fill(system.params.baseAngleRad);
  angles[0] = system.params.baseAngleRad;

  for (let iteration = 0; iteration < maxIterations; iteration += 1) {
    const residual = freeResidual(system, angles, tipLoad);
    const residualNorm = vectorNorm(residual);
    if (residualNorm <= tolerance) {
      return {
        converged: true,
        iteration,
        residualNorm,
        anglesRad: angles,
        kinematics: rodKinematics(system, angles),
      };
    }

    const free = system.freeAngleIndices;
    const jacobian = zeros(free.length);
    for (let column = 0; column < free.length; column += 1) {
      const dof = free[column];
      const plus = [...angles];
      const minus = [...angles];
      plus[dof] += finiteDifferenceRad;
      minus[dof] -= finiteDifferenceRad;
      const residualPlus = freeResidual(system, plus, tipLoad);
      const residualMinus = freeResidual(system, minus, tipLoad);
      for (let row = 0; row < free.length; row += 1) {
        jacobian[row][column] = (
          residualPlus[row] - residualMinus[row]
        ) / (2 * finiteDifferenceRad);
      }
    }

    const delta = solveLinear(
      jacobian,
      residual.map((value) => -value),
    );
    let stepScale = 1;
    let accepted = false;
    while (stepScale >= 1 / 1024) {
      const candidate = [...angles];
      for (let i = 0; i < free.length; i += 1) {
        candidate[free[i]] += stepScale * delta[i];
      }
      const nextNorm = vectorNorm(
        freeResidual(system, candidate, tipLoad),
      );
      if (nextNorm < residualNorm) {
        angles = candidate;
        accepted = true;
        break;
      }
      stepScale *= 0.5;
    }

    if (!accepted) {
      return {
        converged: false,
        iteration,
        residualNorm,
        anglesRad: angles,
        kinematics: rodKinematics(system, angles),
      };
    }
  }

  const residualNorm = vectorNorm(
    freeResidual(system, angles, tipLoad),
  );
  return {
    converged: residualNorm <= tolerance,
    iteration: maxIterations,
    residualNorm,
    anglesRad: angles,
    kinematics: rodKinematics(system, angles),
  };
}

function derivative(system, stateInput, tipLoad) {
  const state = normalizeRodState(system, stateInput);
  return {
    anglesRad: [...state.angularRatesRadS],
    angularRatesRadS: rodAcceleration(system, state, tipLoad),
  };
}

function addStateScaled(state, derivativeState, scale) {
  return {
    anglesRad: state.anglesRad.map(
      (value, i) => value + scale * derivativeState.anglesRad[i],
    ),
    angularRatesRadS: state.angularRatesRadS.map(
      (value, i) => value + scale * derivativeState.angularRatesRadS[i],
    ),
  };
}

export function stepNonlinearRodRK4(
  system,
  stateInput,
  dt,
  tipLoad = null,
) {
  if (!(dt > 0)) throw new RangeError("dt must be positive");
  const state = normalizeRodState(system, stateInput);

  const k1 = derivative(system, state, tipLoad);
  const k2 = derivative(
    system,
    addStateScaled(state, k1, 0.5 * dt),
    tipLoad,
  );
  const k3 = derivative(
    system,
    addStateScaled(state, k2, 0.5 * dt),
    tipLoad,
  );
  const k4 = derivative(
    system,
    addStateScaled(state, k3, dt),
    tipLoad,
  );

  const next = {
    anglesRad: state.anglesRad.map((value, i) => value + dt / 6 * (
      k1.anglesRad[i]
      + 2 * k2.anglesRad[i]
      + 2 * k3.anglesRad[i]
      + k4.anglesRad[i]
    )),
    angularRatesRadS: state.angularRatesRadS.map(
      (value, i) => value + dt / 6 * (
        k1.angularRatesRadS[i]
        + 2 * k2.angularRatesRadS[i]
        + 2 * k3.angularRatesRadS[i]
        + k4.angularRatesRadS[i]
      ),
    ),
  };

  return normalizeRodState(system, next);
}

export function linearizedAngleSystemAtStraight(system) {
  const zeroRates = Array(system.params.segmentCount).fill(0);
  const straight = Array(system.params.segmentCount)
    .fill(system.params.baseAngleRad);
  const { mass } = massMatrixAndBias(system, straight, zeroRates);
  const free = system.freeAngleIndices;
  const stiffnessFull = zeros(system.params.segmentCount);
  const coefficient = system.params.flexuralRigidityNm2
    / system.segmentLengthM;

  for (let i = 1; i < system.params.segmentCount; i += 1) {
    stiffnessFull[i][i] += coefficient;
    stiffnessFull[i - 1][i - 1] += coefficient;
    stiffnessFull[i][i - 1] -= coefficient;
    stiffnessFull[i - 1][i] -= coefficient;
  }

  return {
    reduced: {
      mass: submatrix(mass, free),
      stiffness: submatrix(stiffnessFull, free),
    },
  };
}
