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
  quatRotateVector,
} from "./quaternion.js";

export const DEFAULT_RIGID_BODY_PARAMS = Object.freeze({
  massKg: DEFAULT_ONE_AXIS_PARAMS.massKg,
  gravityMps2: DEFAULT_ONE_AXIS_PARAMS.gravityMps2,
  comPositionBodyM: Object.freeze([0, -0.17, 0]),
  nozzlePositionBodyM: Object.freeze([0.045, -0.33, 0.02]),
  inertiaDiagKgM2: Object.freeze([0.018, 0.009, 0.018]),
  hoseSpringNm: DEFAULT_ONE_AXIS_PARAMS.hoseSpringNm,
  hoseDampingNms: DEFAULT_ONE_AXIS_PARAMS.hoseDampingNms,
  yawDampingNms: 0.008,
  waterDensityKgM3: DEFAULT_ONE_AXIS_PARAMS.waterDensityKgM3,
  effectiveAreaM2: DEFAULT_ONE_AXIS_PARAMS.effectiveAreaM2,
  thrustCoefficient: DEFAULT_ONE_AXIS_PARAMS.thrustCoefficient,
  flowNominalM3s: DEFAULT_ONE_AXIS_PARAMS.flowNominalM3s,
  flowMaxM3s: DEFAULT_ONE_AXIS_PARAMS.flowMaxM3s,
  holdTiltMaxRad: DEFAULT_ONE_AXIS_PARAMS.holdTiltMaxRad,
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

export function scale3(v, s) {
  return [v[0] * s, v[1] * s, v[2] * s];
}

export function holdDirectionWorld3D(holdTiltXRad, holdTiltZRad) {
  // Direction from the hanging head toward the hand. Zero input is world +Y.
  const sx = Math.sin(holdTiltXRad);
  const cx = Math.cos(holdTiltXRad);
  const sz = Math.sin(holdTiltZRad);
  const cz = Math.cos(holdTiltZRad);
  return [-sz * cx, cz * cx, sx];
}

export function waterReactionForceBody3D(thrustN) {
  // The water jet leaves along body -Y, so the body receives +Y reaction.
  return [0, thrustN, 0];
}

export function waterReactionTorqueBody3D(
  thrustN,
  params = DEFAULT_RIGID_BODY_PARAMS,
) {
  return cross3(
    params.nozzlePositionBodyM,
    waterReactionForceBody3D(thrustN),
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
  return cross3(params.comPositionBodyM, gravityForceBody);
}

export function hoseHoldingTorqueBody3D(
  qBodyToWorld,
  omegaBodyRadS,
  holdTiltXRad,
  holdTiltZRad,
  params = DEFAULT_RIGID_BODY_PARAMS,
) {
  const currentHoseAxisWorld = quatRotateVector(
    qBodyToWorld,
    [0, 1, 0],
  );
  const targetHoseAxisWorld = holdDirectionWorld3D(
    holdTiltXRad,
    holdTiltZRad,
  );

  // a x b rotates the current body +Y axis toward the hand-controlled target.
  const alignmentTorqueWorld = scale3(
    cross3(currentHoseAxisWorld, targetHoseAxisWorld),
    params.hoseSpringNm,
  );
  const alignmentTorqueBody = quatInverseRotateVector(
    qBodyToWorld,
    alignmentTorqueWorld,
  );

  return [
    alignmentTorqueBody[0] - params.hoseDampingNms * omegaBodyRadS[0],
    alignmentTorqueBody[1] - params.yawDampingNms * omegaBodyRadS[1],
    alignmentTorqueBody[2] - params.hoseDampingNms * omegaBodyRadS[2],
  ];
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

export function jetDownErrorRad(qBodyToWorld) {
  const jetDirectionWorld = quatRotateVector(qBodyToWorld, [0, -1, 0]);
  const dot = clamp(-jetDirectionWorld[1], -1, 1);
  return Math.acos(dot);
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
  const holdTiltXCommandRad = control.holdTiltXCommandRad ?? 0;
  const holdTiltZCommandRad = control.holdTiltZCommandRad ?? 0;
  const holdTiltXAppliedRad = clamp(
    holdTiltXCommandRad,
    -params.holdTiltMaxRad,
    params.holdTiltMaxRad,
  );
  const holdTiltZAppliedRad = clamp(
    holdTiltZCommandRad,
    -params.holdTiltMaxRad,
    params.holdTiltMaxRad,
  );

  const thrustN = jetThrustFromFlow(flowRateM3s, params);
  const waterReactionForceBodyN = waterReactionForceBody3D(thrustN);
  const waterReactionTorqueBodyNm = waterReactionTorqueBody3D(
    thrustN,
    params,
  );
  const gravityTorqueBodyNm = gravityTorqueBody3D(
    state.qBodyToWorld,
    params,
  );
  const hoseHoldingTorqueBodyNm = hoseHoldingTorqueBody3D(
    state.qBodyToWorld,
    state.omegaBodyRadS,
    holdTiltXAppliedRad,
    holdTiltZAppliedRad,
    params,
  );
  const disturbanceTorqueBodyNm = [
    ...(control.disturbanceTorqueBodyNm ?? [0, 0, 0]),
  ];

  const totalTorqueBodyNm = add3(
    add3(waterReactionTorqueBodyNm, gravityTorqueBodyNm),
    add3(hoseHoldingTorqueBodyNm, disturbanceTorqueBodyNm),
  );
  const alphaBodyRadS2 = angularAccelerationBody3D(
    state.omegaBodyRadS,
    totalTorqueBodyNm,
    params.inertiaDiagKgM2,
  );

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
      holdTiltXCommandRad,
      holdTiltZCommandRad,
      holdTiltXAppliedRad,
      holdTiltZAppliedRad,
      holdTiltXSaturated: holdTiltXAppliedRad !== holdTiltXCommandRad,
      holdTiltZSaturated: holdTiltZAppliedRad !== holdTiltZCommandRad,
      thrustN,
      holdDirectionWorld: holdDirectionWorld3D(
        holdTiltXAppliedRad,
        holdTiltZAppliedRad,
      ),
      waterJetDirectionBody: [0, -1, 0],
      waterReactionForceBodyN,
      waterReactionTorqueBodyNm,
      gravityTorqueBodyNm,
      hoseHoldingTorqueBodyNm,
      disturbanceTorqueBodyNm,
      totalTorqueBodyNm,
      alphaBodyRadS2,
      jetDownErrorRad: jetDownErrorRad(qBodyToWorld),
    },
  };
}
