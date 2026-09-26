export function degToRad(deg) {
  return deg * Math.PI / 180;
}

export function lpmToM3s(lpm) {
  return lpm / 1000 / 60;
}

export function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

export const DEFAULT_ONE_AXIS_PARAMS = Object.freeze({
  massKg: 0.35,
  gravityMps2: 9.81,
  comLeverM: 0.17,
  nozzleOffsetXM: 0.045,
  inertiaKgM2: 0.018,
  hoseSpringNm: 0.22,
  hoseDampingNms: 0.04,
  waterDensityKgM3: 997,
  effectiveAreaM2: 2.0e-5,
  thrustCoefficient: 0.85,
  flowNominalM3s: lpmToM3s(8),
  flowMaxM3s: lpmToM3s(10),
  holdTiltMaxRad: degToRad(25),
});

export const DEFAULT_PD_GAINS = Object.freeze({
  kp: 1.8,
  kd: 0.35,
});

export function jetThrustFromFlow(flowRateM3s, params = DEFAULT_ONE_AXIS_PARAMS) {
  if (flowRateM3s <= 0) return 0;
  return params.thrustCoefficient
    * params.waterDensityKgM3
    * flowRateM3s
    * flowRateM3s
    / params.effectiveAreaM2;
}

export function gravityTorque1D(thetaRad, params = DEFAULT_ONE_AXIS_PARAMS) {
  // Pivot is above the COM. Positive tilt therefore receives a restoring torque.
  return -params.massKg
    * params.gravityMps2
    * params.comLeverM
    * Math.sin(thetaRad);
}

export function waterReactionTorque1D(
  thrustN,
  params = DEFAULT_ONE_AXIS_PARAMS,
) {
  // Water exits downward (-body Y), so the head receives +body Y reaction.
  // A lateral offset of the resultant nozzle line creates a moment about +Z.
  return params.nozzleOffsetXM * thrustN;
}

export function hoseHoldingTorque1D(
  thetaRad,
  omegaRadS,
  holdTiltRad,
  params = DEFAULT_ONE_AXIS_PARAMS,
) {
  return params.hoseSpringNm * Math.sin(holdTiltRad - thetaRad)
    - params.hoseDampingNms * omegaRadS;
}

export function feedforwardHoldTilt1D(
  flowRateM3s,
  params = DEFAULT_ONE_AXIS_PARAMS,
) {
  const thrustN = jetThrustFromFlow(flowRateM3s, params);
  const tauJet = waterReactionTorque1D(thrustN, params);
  const ratio = clamp(-tauJet / params.hoseSpringNm, -1, 1);
  return Math.asin(ratio);
}

export function controllerHoldTiltCommand(state, control = {}, params = DEFAULT_ONE_AXIS_PARAMS) {
  const mode = control.mode ?? "manual";
  if (mode === "manual") return control.holdTiltCommandRad ?? 0;

  const flowRateM3s = control.flowRateM3s ?? params.flowNominalM3s;
  const feedforward = control.feedforward === false
    ? 0
    : feedforwardHoldTilt1D(flowRateM3s, params);
  const kp = control.kp ?? DEFAULT_PD_GAINS.kp;
  const kd = mode === "p" ? 0 : (control.kd ?? DEFAULT_PD_GAINS.kd);

  return feedforward - kp * state.thetaRad - kd * state.omegaRadS;
}

export function stepOneAxis(
  state,
  control = {},
  dt,
  params = DEFAULT_ONE_AXIS_PARAMS,
) {
  if (!(dt > 0)) throw new RangeError("dt must be positive");

  const flowCommandM3s = control.flowRateM3s ?? params.flowNominalM3s;
  const flowRateM3s = clamp(flowCommandM3s, 0, params.flowMaxM3s);
  const holdTiltCommandRad = controllerHoldTiltCommand(
    state,
    { ...control, flowRateM3s },
    params,
  );
  const holdTiltAppliedRad = clamp(
    holdTiltCommandRad,
    -params.holdTiltMaxRad,
    params.holdTiltMaxRad,
  );

  const thrustN = jetThrustFromFlow(flowRateM3s, params);
  const gravityTorqueNm = gravityTorque1D(state.thetaRad, params);
  const waterReactionTorqueNm = waterReactionTorque1D(thrustN, params);
  const hoseHoldingTorqueNm = hoseHoldingTorque1D(
    state.thetaRad,
    state.omegaRadS,
    holdTiltAppliedRad,
    params,
  );
  const disturbanceTorqueNm = control.disturbanceTorqueNm ?? 0;
  const totalTorqueNm = gravityTorqueNm
    + waterReactionTorqueNm
    + hoseHoldingTorqueNm
    + disturbanceTorqueNm;
  const alphaRadS2 = totalTorqueNm / params.inertiaKgM2;

  const omegaRadS = state.omegaRadS + alphaRadS2 * dt;
  const thetaRad = state.thetaRad + omegaRadS * dt;

  return {
    state: { thetaRad, omegaRadS },
    diagnostics: {
      flowCommandM3s,
      flowRateM3s,
      flowSaturated: flowRateM3s !== flowCommandM3s,
      holdTiltCommandRad,
      holdTiltAppliedRad,
      holdTiltSaturated: holdTiltAppliedRad !== holdTiltCommandRad,
      thrustN,
      gravityTorqueNm,
      waterReactionTorqueNm,
      hoseHoldingTorqueNm,
      disturbanceTorqueNm,
      totalTorqueNm,
      alphaRadS2,
    },
  };
}
