import {
  DEFAULT_ONE_AXIS_PARAMS,
  clamp,
  jetThrustFromFlow,
} from "./one-axis.js";
import {
  quatIdentity,
  quatIntegrateBodyRate,
  quatInverseRotateVector,
  quatNormalize,
} from "./quaternion.js";

export const DEFAULT_RIGID_BODY_PARAMS = Object.freeze({
  massKg: DEFAULT_ONE_AXIS_PARAMS.massKg,
  gravityMps2: DEFAULT_ONE_AXIS_PARAMS.gravityMps2,
  comLeverM: DEFAULT_ONE_AXIS_PARAMS.comLeverM,
  nozzleLeverM: DEFAULT_ONE_AXIS_PARAMS.nozzleLeverM,
  // X/Z are symmetric so either tilt axis matches the X1-1 scalar inertia.
  inertiaDiagKgM2: Object.freeze([
    DEFAULT_ONE_AXIS_PARAMS.inertiaKgM2,
    0.009,
    DEFAULT_ONE_AXIS_PARAMS.inertiaKgM2,
  ]),
  waterDensityKgM3: DEFAULT_ONE_AXIS_PARAMS.waterDensityKgM3,
  effectiveAreaM2: DEFAULT_ONE_AXIS_PARAMS.effectiveAreaM2,
  thrustCoefficient: DEFAULT_ONE_AXIS_PARAMS.thrustCoefficient,
  flowNominalM3s: DEFAULT_ONE_AXIS_PARAMS.flowNominalM3s,
  flowMaxM3s: DEFAULT_ONE_AXIS_PARAMS.flowMaxM3s,
  gimbalMaxRad: DEFAULT_ONE_AXIS_PARAMS.gimbalMaxRad,
});

export function cross3(a, b) {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}

export function add3(a, b) {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

export function jetForceBody3D(deltaXRad, deltaZRad, thrustN) {
  // Start from body +Y, rotate by +deltaX about body X, then +deltaZ
  // about body Z. With deltaX=0 this preserves the X1-1 +Z slice.
  const sx = Math.sin(deltaXRad);
  const cx = Math.cos(deltaXRad);
  const sz = Math.sin(deltaZRad);
  const cz = Math.cos(deltaZRad);
  return [
    -thrustN * sz * cx,
    thrustN * cz * cx,
    thrustN * sx,
  ];
}

export function jetTorqueBody3D(
  deltaXRad,
  deltaZRad,
  thrustN,
  params = DEFAULT_RIGID_BODY_PARAMS,
) {
  return cross3(
    [0, params.nozzleLeverM, 0],
    jetForceBody3D(deltaXRad, deltaZRad, thrustN),
  );
}

export function gravityTorqueBody3D(
  qBodyToWorld,
  params = DEFAULT_RIGID_BODY_PARAMS,
) {
  const gravityForceWorld = [0, -params.massKg * params.gravityMps2, 0];
  const gravityForceBody = quatInverseRotateVector(
    qBodyToWorld,
    gravityForceWorld,
  );
  return cross3([0, params.comLeverM, 0], gravityForceBody);
}

export function angularAccelerationBody3D(
  omegaBodyRadS,
  torqueBodyNm,
  inertiaDiagKgM2,
) {
  const angularMomentumBody = omegaBodyRadS.map(
    (omega, i) => omega * inertiaDiagKgM2[i],
  );
  const gyroscopicBody = cross3(omegaBodyRadS, angularMomentumBody);
  return torqueBodyNm.map(
    (tau, i) => (tau - gyroscopicBody[i]) / inertiaDiagKgM2[i],
  );
}

export function createRigidBodyState(overrides = {}) {
  return {
    qBodyToWorld: quatNormalize(overrides.qBodyToWorld ?? quatIdentity()),
    omegaBodyRadS: [...(overrides.omegaBodyRadS ?? [0, 0, 0])],
  };
}

export function stepRigidBody3D(
  state,
  control = {},
  dt,
  params = DEFAULT_RIGID_BODY_PARAMS,
) {
  if (!(dt > 0)) throw new RangeError("dt must be positive");

  const flowCommandM3s = control.flowRateM3s ?? params.flowNominalM3s;
  const flowRateM3s = clamp(flowCommandM3s, 0, params.flowMaxM3s);
  const deltaXCommandRad = control.deltaXCommandRad ?? 0;
  const deltaZCommandRad = control.deltaZCommandRad ?? 0;
  const deltaXAppliedRad = clamp(
    deltaXCommandRad,
    -params.gimbalMaxRad,
    params.gimbalMaxRad,
  );
  const deltaZAppliedRad = clamp(
    deltaZCommandRad,
    -params.gimbalMaxRad,
    params.gimbalMaxRad,
  );

  const thrustN = jetThrustFromFlow(flowRateM3s, params);
  const jetForceBodyN = jetForceBody3D(
    deltaXAppliedRad,
    deltaZAppliedRad,
    thrustN,
  );
  const jetTorqueBodyNm = cross3(
    [0, params.nozzleLeverM, 0],
    jetForceBodyN,
  );
  const gravityTorqueBodyNm = gravityTorqueBody3D(
    state.qBodyToWorld,
    params,
  );
  const disturbanceTorqueBodyNm = [
    ...(control.disturbanceTorqueBodyNm ?? [0, 0, 0]),
  ];
  const totalTorqueBodyNm = add3(
    add3(jetTorqueBodyNm, gravityTorqueBodyNm),
    disturbanceTorqueBodyNm,
  );
  const alphaBodyRadS2 = angularAccelerationBody3D(
    state.omegaBodyRadS,
    totalTorqueBodyNm,
    params.inertiaDiagKgM2,
  );

  // Semi-implicit Euler for angular velocity, then quaternion update using
  // the new body angular velocity.
  const omegaBodyRadS = state.omegaBodyRadS.map(
    (omega, i) => omega + alphaBodyRadS2[i] * dt,
  );
  const qBodyToWorld = quatIntegrateBodyRate(
    state.qBodyToWorld,
    omegaBodyRadS,
    dt,
  );

  return {
    state: { qBodyToWorld, omegaBodyRadS },
    diagnostics: {
      flowCommandM3s,
      flowRateM3s,
      flowSaturated: flowRateM3s !== flowCommandM3s,
      deltaXCommandRad,
      deltaZCommandRad,
      deltaXAppliedRad,
      deltaZAppliedRad,
      gimbalXSaturated: deltaXAppliedRad !== deltaXCommandRad,
      gimbalZSaturated: deltaZAppliedRad !== deltaZCommandRad,
      thrustN,
      jetForceBodyN,
      jetTorqueBodyNm,
      gravityTorqueBodyNm,
      disturbanceTorqueBodyNm,
      totalTorqueBodyNm,
      alphaBodyRadS2,
    },
  };
}
