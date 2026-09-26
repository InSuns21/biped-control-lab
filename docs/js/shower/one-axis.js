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
  comLeverM: 0.12,
  nozzleLeverM: 0.38,
  inertiaKgM2: 0.018,
  waterDensityKgM3: 997,
  effectiveAreaM2: 2.0e-5,
  thrustCoefficient: 0.85,
  flowNominalM3s: lpmToM3s(8),
  flowMaxM3s: lpmToM3s(10),
  gimbalMaxRad: degToRad(25),
});

export const DEFAULT_PD_GAINS = Object.freeze({
  kp: 2.5,
  kd: 0.7,
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
  return params.massKg
    * params.gravityMps2
    * params.comLeverM
    * Math.sin(thetaRad);
}

export function jetTorque1D(deltaRad, thrustN, params = DEFAULT_ONE_AXIS_PARAMS) {
  return params.nozzleLeverM * thrustN * Math.sin(deltaRad);
}

export function controllerDeltaCommand(state, control = {}) {
  const mode = control.mode ?? "pd";
  if (mode === "manual") return control.deltaCommandRad ?? 0;

  const kp = control.kp ?? DEFAULT_PD_GAINS.kp;
  const kd = mode === "p" ? 0 : (control.kd ?? DEFAULT_PD_GAINS.kd);
  return -kp * state.thetaRad - kd * state.omegaRadS;
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
  const deltaCommandRad = controllerDeltaCommand(state, control);
  const deltaAppliedRad = clamp(
    deltaCommandRad,
    -params.gimbalMaxRad,
    params.gimbalMaxRad,
  );

  const thrustN = jetThrustFromFlow(flowRateM3s, params);
  const gravityTorqueNm = gravityTorque1D(state.thetaRad, params);
  const jetTorqueNm = jetTorque1D(deltaAppliedRad, thrustN, params);
  const disturbanceTorqueNm = control.disturbanceTorqueNm ?? 0;
  const totalTorqueNm = gravityTorqueNm + jetTorqueNm + disturbanceTorqueNm;
  const alphaRadS2 = totalTorqueNm / params.inertiaKgM2;

  // Semi-implicit Euler: update angular velocity first, then angle.
  const omegaRadS = state.omegaRadS + alphaRadS2 * dt;
  const thetaRad = state.thetaRad + omegaRadS * dt;

  return {
    state: { thetaRad, omegaRadS },
    diagnostics: {
      flowCommandM3s,
      flowRateM3s,
      flowSaturated: flowRateM3s !== flowCommandM3s,
      deltaCommandRad,
      deltaAppliedRad,
      gimbalSaturated: deltaAppliedRad !== deltaCommandRad,
      thrustN,
      gravityTorqueNm,
      jetTorqueNm,
      disturbanceTorqueNm,
      totalTorqueNm,
      alphaRadS2,
    },
  };
}
