function tangent(angleRad) {
  return [Math.sin(angleRad), Math.cos(angleRad)];
}

function normalDerivative(angleRad) {
  return [Math.cos(angleRad), -Math.sin(angleRad)];
}

function addPointForce(
  force,
  anglesRad,
  coefficients,
  forceXYN,
) {
  for (let k = 0; k < force.length; k += 1) {
    const coefficient = coefficients[k] ?? 0;
    if (coefficient === 0) continue;
    const n = normalDerivative(anglesRad[k]);
    force[k] += coefficient * (
      n[0] * forceXYN[0]
      + n[1] * forceXYN[1]
    );
  }
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

function nodeCoefficients(system, nodeIndex) {
  const count = system.params.segmentCount;
  const l = system.segmentLengthM;
  const coefficients = Array(count).fill(0);
  for (let k = 0; k < nodeIndex; k += 1) {
    coefficients[k] = l;
  }
  return coefficients;
}

function normalizedState(system, stateInput) {
  const anglesRad = [...stateInput.anglesRad];
  const angularRatesRadS = [...stateInput.angularRatesRadS];
  if (
    anglesRad.length !== system.params.segmentCount
    || angularRatesRadS.length !== system.params.segmentCount
  ) {
    throw new RangeError("flow state must match segmentCount");
  }
  anglesRad[0] = system.params.baseAngleRad;
  angularRatesRadS[0] = 0;
  return { anglesRad, angularRatesRadS };
}

export function generalizedCoriolisFlowForce(
  system,
  stateInput,
  {
    flowSpeedMps,
    fluidMassPerM = system.params.fluidMassPerM,
  },
) {
  if (!Number.isFinite(flowSpeedMps)) {
    throw new RangeError("flowSpeedMps must be finite");
  }
  if (!(fluidMassPerM >= 0)) {
    throw new RangeError("fluidMassPerM must be non-negative");
  }

  const state = normalizedState(system, stateInput);
  const force = Array(system.params.segmentCount).fill(0);
  const l = system.segmentLengthM;

  // Exact finite-angle counterpart of -2 m_f U r_st.
  // For an inextensible planar rod r_s=t(theta), so
  // r_st = theta_dot n(theta).
  for (let i = 0; i < system.params.segmentCount; i += 1) {
    const n = normalDerivative(state.anglesRad[i]);
    const scalar = -2
      * fluidMassPerM
      * flowSpeedMps
      * l
      * state.angularRatesRadS[i];
    const cartesianForce = [
      scalar * n[0],
      scalar * n[1],
    ];
    addPointForce(
      force,
      state.anglesRad,
      segmentCenterCoefficients(system, i),
      cartesianForce,
    );
  }

  return force;
}

export function generalizedCentrifugalFlowForce(
  system,
  stateInput,
  {
    flowSpeedMps,
    fluidMassPerM = system.params.fluidMassPerM,
  },
) {
  if (!Number.isFinite(flowSpeedMps)) {
    throw new RangeError("flowSpeedMps must be finite");
  }
  if (!(fluidMassPerM >= 0)) {
    throw new RangeError("fluidMassPerM must be non-negative");
  }

  const state = normalizedState(system, stateInput);
  const force = Array(system.params.segmentCount).fill(0);
  const u2 = flowSpeedMps * flowSpeedMps;

  // Exact finite-angle counterpart of -m_f U^2 r_ss.
  // Piecewise-constant segment angles place curvature at joints. Across joint
  // i, integral(kappa n ds) = t_i - t_(i-1), so the integrated convective
  // load is -m_f U^2 (t_i - t_(i-1)).
  for (let i = 1; i < system.params.segmentCount; i += 1) {
    const before = tangent(state.anglesRad[i - 1]);
    const after = tangent(state.anglesRad[i]);
    const cartesianForce = [
      -fluidMassPerM * u2 * (after[0] - before[0]),
      -fluidMassPerM * u2 * (after[1] - before[1]),
    ];
    addPointForce(
      force,
      state.anglesRad,
      nodeCoefficients(system, i),
      cartesianForce,
    );
  }

  return force;
}

export function generalizedConveyingFlowForce(
  system,
  stateInput,
  options,
) {
  const coriolis = generalizedCoriolisFlowForce(
    system,
    stateInput,
    options,
  );
  const centrifugal = generalizedCentrifugalFlowForce(
    system,
    stateInput,
    options,
  );
  return coriolis.map((value, i) => value + centrifugal[i]);
}

export function createConveyingFlowGeneralizedForce(options) {
  return ({ system, state }) => generalizedConveyingFlowForce(
    system,
    state,
    options,
  );
}
