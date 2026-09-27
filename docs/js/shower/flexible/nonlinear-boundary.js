import {
  bendingEnergy,
  generalizedBendingForce,
  generalizedGravityForce,
  generalizedRayleighDampingForce,
  generalizedTipLoad,
  gravitationalPotential,
  massMatrixAndBias,
  rodKinematics,
} from "./nonlinear-rod.js";
import {
  solveLinear,
  zeros,
} from "./linear-algebra.js";

export const ZERO_HAND_BOUNDARY = Object.freeze({
  lateralPositionM: 0,
  lateralVelocityMps: 0,
  lateralAccelerationMps2: 0,
  angleRad: 0,
  angularRateRadS: 0,
  angularAccelerationRadS2: 0,
});

function boundaryWithDefaults(boundary = {}) {
  return {
    ...ZERO_HAND_BOUNDARY,
    ...boundary,
  };
}

function dynamicSystem(system, boundary) {
  return {
    ...system,
    params: {
      ...system.params,
      baseAngleRad: boundary.angleRad,
    },
  };
}

function tangent(angleRad) {
  return [Math.sin(angleRad), Math.cos(angleRad)];
}

function normalDerivative(angleRad) {
  return [Math.cos(angleRad), -Math.sin(angleRad)];
}

function segmentCenterCoefficients(system, segmentIndex) {
  const count = system.params.segmentCount;
  const l = system.segmentLengthM;
  const coefficients = Array(count).fill(0);
  for (let k = 0; k <= segmentIndex; k += 1) {
    coefficients[k] = k < segmentIndex ? l : 0.5 * l;
  }
  return coefficients;
}

function tipCoefficients(system, axialOffsetM = 0) {
  const coefficients = Array(system.params.segmentCount)
    .fill(system.segmentLengthM);
  coefficients[coefficients.length - 1] += axialOffsetM;
  return coefficients;
}

function pointBiasAcceleration(anglesRad, ratesRadS, coefficients) {
  const value = [0, 0];
  for (let k = 0; k < anglesRad.length; k += 1) {
    const coefficient = coefficients[k] ?? 0;
    if (coefficient === 0) continue;
    const t = tangent(anglesRad[k]);
    const omega2 = ratesRadS[k] * ratesRadS[k];
    value[0] -= coefficient * t[0] * omega2;
    value[1] -= coefficient * t[1] * omega2;
  }
  return value;
}

function pointXJacobian(anglesRad, coefficients) {
  return anglesRad.map((angle, k) => (
    (coefficients[k] ?? 0) * normalDerivative(angle)[0]
  ));
}

function submatrixRect(matrix, rows, columns) {
  return rows.map((i) => columns.map((j) => matrix[i][j]));
}

function reduceVector(vector, indices) {
  return indices.map((index) => vector[index]);
}

function matVec(matrix, vector) {
  return matrix.map(
    (row) => row.reduce(
      (sum, value, index) => sum + value * vector[index],
      0,
    ),
  );
}

function addInPlace(target, source) {
  for (let i = 0; i < target.length; i += 1) {
    target[i] += source[i];
  }
}

function resolveTipLoad(system, state, tipLoad) {
  if (!tipLoad) return null;
  if (typeof tipLoad === "function") {
    return tipLoad({
      system,
      state,
      kinematics: rodKinematics(system, state.anglesRad),
    });
  }
  return tipLoad;
}

function resolveGeneralizedForce(
  system,
  state,
  forceSource,
) {
  if (!forceSource) {
    return Array(system.params.segmentCount).fill(0);
  }
  const value = typeof forceSource === "function"
    ? forceSource({
      system,
      state,
      kinematics: rodKinematics(system, state.anglesRad),
    })
    : forceSource;
  if (
    !Array.isArray(value)
    || value.length !== system.params.segmentCount
  ) {
    throw new RangeError(
      "additionalGeneralizedForce must match segmentCount",
    );
  }
  return value;
}

function resolveCartesianResultant(
  system,
  state,
  source,
) {
  if (!source) return [0, 0];
  const value = typeof source === "function"
    ? source({
      system,
      state,
      kinematics: rodKinematics(system, state.anglesRad),
    })
    : source;
  if (!Array.isArray(value) || value.length !== 2) {
    throw new RangeError(
      "additionalCartesianResultant must be [Fx, Fy]",
    );
  }
  return value;
}

export function stateWithHandBoundary(
  system,
  stateInput,
  boundaryInput,
) {
  const boundary = boundaryWithDefaults(boundaryInput);
  const anglesRad = [...stateInput.anglesRad];
  const angularRatesRadS = [...stateInput.angularRatesRadS];
  if (
    anglesRad.length !== system.params.segmentCount
    || angularRatesRadS.length !== system.params.segmentCount
  ) {
    throw new RangeError("rod state length must equal segmentCount");
  }
  anglesRad[0] = boundary.angleRad;
  angularRatesRadS[0] = boundary.angularRateRadS;
  return {
    anglesRad,
    angularRatesRadS,
  };
}

export function rodKinematicsWithHandBoundary(
  system,
  stateInput,
  boundaryInput,
) {
  const boundary = boundaryWithDefaults(boundaryInput);
  const state = stateWithHandBoundary(
    system,
    stateInput,
    boundary,
  );
  const localSystem = dynamicSystem(system, boundary);
  const local = rodKinematics(localSystem, state.anglesRad);
  const shift = ([x, y]) => [
    x + boundary.lateralPositionM,
    y,
  ];

  return {
    ...local,
    nodes: local.nodes.map(shift),
    segmentCenters: local.segmentCenters.map(shift),
    tip: shift(local.tip),
    headCom: shift(local.headCom),
    base: [boundary.lateralPositionM, 0],
  };
}

export function extendedMassAndBias(
  system,
  stateInput,
  boundaryInput,
) {
  const boundary = boundaryWithDefaults(boundaryInput);
  const state = stateWithHandBoundary(
    system,
    stateInput,
    boundary,
  );
  const localSystem = dynamicSystem(system, boundary);
  const { mass: angularMass, bias: angularBias } = massMatrixAndBias(
    localSystem,
    state.anglesRad,
    state.angularRatesRadS,
  );

  const count = system.params.segmentCount;
  const size = count + 1;
  const mass = zeros(size);
  const bias = Array(size).fill(0);

  for (let i = 0; i < count; i += 1) {
    for (let j = 0; j < count; j += 1) {
      mass[i + 1][j + 1] = angularMass[i][j];
    }
    bias[i + 1] = angularBias[i];
  }

  const segmentMassKg = system.segmentMassKg;
  const totalMassKg = segmentMassKg * count
    + system.params.headMassKg;
  mass[0][0] = totalMassKg;

  for (let segment = 0; segment < count; segment += 1) {
    const coefficients = segmentCenterCoefficients(
      localSystem,
      segment,
    );
    const jx = pointXJacobian(
      state.anglesRad,
      coefficients,
    );
    const aBias = pointBiasAcceleration(
      state.anglesRad,
      state.angularRatesRadS,
      coefficients,
    );
    for (let k = 0; k < count; k += 1) {
      mass[0][k + 1] += segmentMassKg * jx[k];
      mass[k + 1][0] = mass[0][k + 1];
    }
    bias[0] += segmentMassKg * aBias[0];
  }

  if (system.params.headMassKg > 0) {
    const coefficients = tipCoefficients(
      localSystem,
      system.params.headComAxialOffsetM,
    );
    const jx = pointXJacobian(
      state.anglesRad,
      coefficients,
    );
    const aBias = pointBiasAcceleration(
      state.anglesRad,
      state.angularRatesRadS,
      coefficients,
    );
    for (let k = 0; k < count; k += 1) {
      mass[0][k + 1] += system.params.headMassKg * jx[k];
      mass[k + 1][0] = mass[0][k + 1];
    }
    bias[0] += system.params.headMassKg * aBias[0];
  }

  return {
    mass,
    bias,
    state,
    system: localSystem,
  };
}

export function extendedGeneralizedForce(
  system,
  stateInput,
  boundaryInput,
  {
    tipLoad = null,
    additionalGeneralizedForce = null,
    additionalCartesianResultant = null,
  } = {},
) {
  const boundary = boundaryWithDefaults(boundaryInput);
  const state = stateWithHandBoundary(
    system,
    stateInput,
    boundary,
  );
  const localSystem = dynamicSystem(system, boundary);

  const angularForce = generalizedBendingForce(
    localSystem,
    state.anglesRad,
  );
  addInPlace(
    angularForce,
    generalizedGravityForce(
      localSystem,
      state.anglesRad,
    ),
  );
  addInPlace(
    angularForce,
    generalizedRayleighDampingForce(
      localSystem,
      state.anglesRad,
      state.angularRatesRadS,
    ),
  );

  const resolvedTip = resolveTipLoad(
    localSystem,
    state,
    tipLoad,
  );
  if (resolvedTip) {
    addInPlace(
      angularForce,
      generalizedTipLoad(
        localSystem,
        state.anglesRad,
        resolvedTip,
      ),
    );
  }

  const additional = resolveGeneralizedForce(
    localSystem,
    state,
    additionalGeneralizedForce,
  );
  addInPlace(angularForce, additional);

  const cartesianResultant = resolveCartesianResultant(
    localSystem,
    state,
    additionalCartesianResultant,
  );

  const force = Array(system.params.segmentCount + 1).fill(0);
  force[0] = cartesianResultant[0]
    + (resolvedTip?.forceXYN?.[0] ?? 0);
  for (let i = 0; i < angularForce.length; i += 1) {
    force[i + 1] = angularForce[i];
  }

  return {
    force,
    state,
    system: localSystem,
    resolvedTip,
    cartesianResultant,
  };
}

export function handBoundaryDynamics(
  system,
  stateInput,
  boundaryInput,
  loadOptions = {},
) {
  const boundary = boundaryWithDefaults(boundaryInput);
  const massData = extendedMassAndBias(
    system,
    stateInput,
    boundary,
  );
  const forceData = extendedGeneralizedForce(
    system,
    massData.state,
    boundary,
    loadOptions,
  );

  const size = system.params.segmentCount + 1;
  const boundaryIndices = [0, 1];
  const freeIndices = Array.from(
    { length: size - 2 },
    (_, index) => index + 2,
  );
  const prescribedAcceleration = [
    boundary.lateralAccelerationMps2,
    boundary.angularAccelerationRadS2,
  ];

  const freeRhs = reduceVector(
    forceData.force.map(
      (value, i) => value - massData.bias[i],
    ),
    freeIndices,
  );
  const coupling = matVec(
    submatrixRect(
      massData.mass,
      freeIndices,
      boundaryIndices,
    ),
    prescribedAcceleration,
  );
  for (let i = 0; i < freeRhs.length; i += 1) {
    freeRhs[i] -= coupling[i];
  }

  const freeAcceleration = solveLinear(
    submatrixRect(
      massData.mass,
      freeIndices,
      freeIndices,
    ),
    freeRhs,
  );

  const acceleration = Array(size).fill(0);
  acceleration[0] = prescribedAcceleration[0];
  acceleration[1] = prescribedAcceleration[1];
  for (let i = 0; i < freeIndices.length; i += 1) {
    acceleration[freeIndices[i]] = freeAcceleration[i];
  }

  const lhs = matVec(massData.mass, acceleration);
  const reaction = boundaryIndices.map(
    (row) => lhs[row]
      + massData.bias[row]
      - forceData.force[row],
  );

  const angularAccelerationsRadS2 = Array(
    system.params.segmentCount,
  ).fill(0);
  angularAccelerationsRadS2[0] =
    boundary.angularAccelerationRadS2;
  for (let i = 1; i < angularAccelerationsRadS2.length; i += 1) {
    angularAccelerationsRadS2[i] = acceleration[i + 1];
  }

  const handPowerW = reaction[0] * boundary.lateralVelocityMps
    + reaction[1] * boundary.angularRateRadS;

  return {
    state: massData.state,
    angularAccelerationsRadS2,
    reactionForceXN: reaction[0],
    reactionMomentNm: reaction[1],
    handPowerW,
    extendedAcceleration: acceleration,
    extendedMass: massData.mass,
    extendedBias: massData.bias,
    extendedForce: forceData.force,
  };
}

function addStateScaled(state, derivative, scale) {
  return {
    anglesRad: state.anglesRad.map(
      (value, i) => value + scale * derivative.anglesRad[i],
    ),
    angularRatesRadS: state.angularRatesRadS.map(
      (value, i) => value + scale * derivative.angularRatesRadS[i],
    ),
  };
}

function derivativeAt(
  system,
  stateInput,
  boundary,
  loadOptions,
) {
  const dynamics = handBoundaryDynamics(
    system,
    stateInput,
    boundary,
    loadOptions,
  );
  return {
    anglesRad: [...dynamics.state.angularRatesRadS],
    angularRatesRadS: [...dynamics.angularAccelerationsRadS2],
  };
}

export function stepRodWithHandBoundaryRK4(
  system,
  stateInput,
  timeS,
  dt,
  boundaryAtTime,
  loadOptions = {},
) {
  if (!(dt > 0)) throw new RangeError("dt must be positive");
  if (typeof boundaryAtTime !== "function") {
    throw new TypeError("boundaryAtTime must be a function");
  }

  const b1 = boundaryWithDefaults(boundaryAtTime(timeS));
  const s1 = stateWithHandBoundary(system, stateInput, b1);
  const k1 = derivativeAt(system, s1, b1, loadOptions);

  const b2 = boundaryWithDefaults(
    boundaryAtTime(timeS + 0.5 * dt),
  );
  const s2 = stateWithHandBoundary(
    system,
    addStateScaled(s1, k1, 0.5 * dt),
    b2,
  );
  const k2 = derivativeAt(system, s2, b2, loadOptions);

  const s3 = stateWithHandBoundary(
    system,
    addStateScaled(s1, k2, 0.5 * dt),
    b2,
  );
  const k3 = derivativeAt(system, s3, b2, loadOptions);

  const b4 = boundaryWithDefaults(boundaryAtTime(timeS + dt));
  const s4 = stateWithHandBoundary(
    system,
    addStateScaled(s1, k3, dt),
    b4,
  );
  const k4 = derivativeAt(system, s4, b4, loadOptions);

  const next = {
    anglesRad: s1.anglesRad.map((value, i) => value + dt / 6 * (
      k1.anglesRad[i]
      + 2 * k2.anglesRad[i]
      + 2 * k3.anglesRad[i]
      + k4.anglesRad[i]
    )),
    angularRatesRadS: s1.angularRatesRadS.map(
      (value, i) => value + dt / 6 * (
        k1.angularRatesRadS[i]
        + 2 * k2.angularRatesRadS[i]
        + 2 * k3.angularRatesRadS[i]
        + k4.angularRatesRadS[i]
      ),
    ),
  };
  const normalizedNext = stateWithHandBoundary(
    system,
    next,
    b4,
  );
  const diagnostics = handBoundaryDynamics(
    system,
    normalizedNext,
    b4,
    loadOptions,
  );

  return {
    state: normalizedNext,
    boundary: b4,
    diagnostics,
  };
}

function smoothBumpUnit(r) {
  if (r <= 0 || r >= 1) {
    return {
      value: 0,
      first: 0,
      second: 0,
    };
  }
  const r2 = r * r;
  const r3 = r2 * r;
  const r4 = r3 * r;
  const r5 = r4 * r;
  const r6 = r5 * r;
  return {
    value: 64 * (r3 - 3 * r4 + 3 * r5 - r6),
    first: 64 * (3 * r2 - 12 * r3 + 15 * r4 - 6 * r5),
    second: 64 * (6 * r - 36 * r2 + 60 * r3 - 30 * r4),
  };
}

export function smoothHandPulse(
  timeS,
  {
    startTimeS = 0,
    durationS = 0.35,
    lateralAmplitudeM = 0,
    angleAmplitudeRad = 0,
  } = {},
) {
  if (!(durationS > 0)) {
    throw new RangeError("durationS must be positive");
  }
  const r = (timeS - startTimeS) / durationS;
  const bump = smoothBumpUnit(r);
  const velocityScale = bump.first / durationS;
  const accelerationScale = bump.second
    / (durationS * durationS);

  return {
    lateralPositionM: lateralAmplitudeM * bump.value,
    lateralVelocityMps: lateralAmplitudeM * velocityScale,
    lateralAccelerationMps2:
      lateralAmplitudeM * accelerationScale,
    angleRad: angleAmplitudeRad * bump.value,
    angularRateRadS: angleAmplitudeRad * velocityScale,
    angularAccelerationRadS2:
      angleAmplitudeRad * accelerationScale,
  };
}

export function sumHandBoundaries(boundaries) {
  return boundaries.reduce(
    (sum, boundary) => ({
      lateralPositionM:
        sum.lateralPositionM + boundary.lateralPositionM,
      lateralVelocityMps:
        sum.lateralVelocityMps + boundary.lateralVelocityMps,
      lateralAccelerationMps2:
        sum.lateralAccelerationMps2
        + boundary.lateralAccelerationMps2,
      angleRad: sum.angleRad + boundary.angleRad,
      angularRateRadS:
        sum.angularRateRadS + boundary.angularRateRadS,
      angularAccelerationRadS2:
        sum.angularAccelerationRadS2
        + boundary.angularAccelerationRadS2,
    }),
    { ...ZERO_HAND_BOUNDARY },
  );
}

export function mechanicalEnergyWithHandBoundary(
  system,
  stateInput,
  boundaryInput,
) {
  const boundary = boundaryWithDefaults(boundaryInput);
  const data = extendedMassAndBias(
    system,
    stateInput,
    boundary,
  );
  const velocity = [
    boundary.lateralVelocityMps,
    ...data.state.angularRatesRadS,
  ];

  let kineticJ = 0;
  for (let i = 0; i < data.mass.length; i += 1) {
    for (let j = 0; j < data.mass.length; j += 1) {
      kineticJ += 0.5
        * velocity[i]
        * data.mass[i][j]
        * velocity[j];
    }
  }

  return {
    kineticJ,
    bendingJ: bendingEnergy(
      data.system,
      data.state.anglesRad,
    ),
    gravityJ: gravitationalPotential(
      data.system,
      data.state.anglesRad,
    ),
    totalJ: kineticJ
      + bendingEnergy(
        data.system,
        data.state.anglesRad,
      )
      + gravitationalPotential(
        data.system,
        data.state.anglesRad,
      ),
  };
}
