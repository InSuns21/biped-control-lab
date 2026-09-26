import assert from "node:assert/strict";
import {
  DEFAULT_ONE_AXIS_PARAMS,
  DEFAULT_PD_GAINS,
  degToRad,
  gravityTorque1D,
  jetThrustFromFlow,
  jetTorque1D,
  lpmToM3s,
  stepOneAxis,
} from "../docs/js/shower/one-axis.js";
import {
  quatFromAxisAngle,
  quatNorm,
  quatRotateVector,
} from "../docs/js/shower/quaternion.js";
import {
  DEFAULT_RIGID_BODY_PARAMS,
  angularAccelerationBody3D,
  createRigidBodyState,
  jetForceBody3D,
  jetTorqueBody3D,
  stepRigidBody3D,
} from "../docs/js/shower/rigid-body.js";

const params = DEFAULT_ONE_AXIS_PARAMS;
const nearly = (a, b, tol = 1e-12) => Math.abs(a - b) <= tol;
const vectorNearly = (a, b, tol = 1e-12) => (
  a.length === b.length && a.every((x, i) => nearly(x, b[i], tol))
);

assert.equal(
  jetThrustFromFlow(0, params),
  0,
  "Q = 0 should produce zero jet thrust",
);

const thrust4Lpm = jetThrustFromFlow(lpmToM3s(4), params);
const thrust8Lpm = jetThrustFromFlow(lpmToM3s(8), params);
assert.ok(
  nearly(thrust8Lpm / thrust4Lpm, 4),
  "doubling flow should quadruple idealized jet thrust",
);

assert.ok(
  nearly(jetTorque1D(0, thrust8Lpm, params), 0),
  "zero gimbal angle should produce zero jet torque",
);

const positiveJetTorque = jetTorque1D(degToRad(10), thrust8Lpm, params);
const negativeJetTorque = jetTorque1D(degToRad(-10), thrust8Lpm, params);
assert.ok(
  nearly(positiveJetTorque, -negativeJetTorque),
  "reversing gimbal angle should reverse jet torque",
);

assert.ok(
  gravityTorque1D(degToRad(5), params) > 0,
  "positive tilt should receive destabilizing gravity torque",
);

const saturated = stepOneAxis(
  { thetaRad: degToRad(30), omegaRadS: 0 },
  { mode: "pd", kp: 4, kd: 1, flowRateM3s: lpmToM3s(20) },
  0.001,
  params,
);

assert.equal(
  saturated.diagnostics.gimbalSaturated,
  true,
  "large PD command should hit gimbal saturation",
);
assert.equal(
  saturated.diagnostics.flowSaturated,
  true,
  "flow command above Q_max should be clamped",
);
assert.ok(
  nearly(
    Math.abs(saturated.diagnostics.deltaAppliedRad),
    params.gimbalMaxRad,
  ),
  "applied gimbal angle should equal the limit",
);
assert.ok(
  saturated.diagnostics.deltaCommandRad
    !== saturated.diagnostics.deltaAppliedRad,
  "command and applied gimbal angle should remain distinguishable",
);

let pdState = { thetaRad: degToRad(8), omegaRadS: 0 };
let pdDiagnostics;
for (let t = 0; t < 6; t += 0.001) {
  const out = stepOneAxis(
    pdState,
    { mode: "pd", ...DEFAULT_PD_GAINS },
    0.001,
    params,
  );
  pdState = out.state;
  pdDiagnostics = out.diagnostics;
}

assert.ok(
  Math.abs(pdState.thetaRad) < degToRad(0.05),
  "default PD should recover a small tilt close to upright",
);
assert.ok(
  Math.abs(pdState.omegaRadS) < 0.002,
  "default PD should damp angular velocity close to zero",
);
assert.equal(
  pdDiagnostics.gimbalSaturated,
  false,
  "settled default PD state should be unsaturated",
);

let pState = { thetaRad: degToRad(8), omegaRadS: 0 };
for (let t = 0; t < 6; t += 0.001) {
  pState = stepOneAxis(
    pState,
    { mode: "p", kp: DEFAULT_PD_GAINS.kp },
    0.001,
    params,
  ).state;
}

assert.ok(
  Math.abs(pState.omegaRadS) > 0.05,
  "P-only case should retain appreciable oscillatory motion in this ideal model",
);

// X1-2 quaternion convention: +90 deg about +Z maps body +Y toward world -X.
const qPlusZ90 = quatFromAxisAngle([0, 0, 1], Math.PI / 2);
assert.ok(
  vectorNearly(
    quatRotateVector(qPlusZ90, [0, 1, 0]),
    [-1, 0, 0],
    1e-12,
  ),
  "body->world quaternion convention should follow the right-hand rule",
);

// Positive gimbal angles create positive torques about the corresponding axis.
const thrust3D = jetThrustFromFlow(
  DEFAULT_RIGID_BODY_PARAMS.flowNominalM3s,
  DEFAULT_RIGID_BODY_PARAMS,
);
const torquePlusX = jetTorqueBody3D(
  degToRad(10),
  0,
  thrust3D,
  DEFAULT_RIGID_BODY_PARAMS,
);
const torquePlusZ = jetTorqueBody3D(
  0,
  degToRad(10),
  thrust3D,
  DEFAULT_RIGID_BODY_PARAMS,
);
assert.ok(
  torquePlusX[0] > 0 && nearly(torquePlusX[1], 0) && nearly(torquePlusX[2], 0),
  "positive delta_x should create positive body-X torque",
);
assert.ok(
  nearly(torquePlusZ[0], 0) && nearly(torquePlusZ[1], 0) && torquePlusZ[2] > 0,
  "positive delta_z should create positive body-Z torque",
);

const combinedForce = jetForceBody3D(
  degToRad(13),
  degToRad(-9),
  thrust3D,
);
assert.ok(
  nearly(Math.hypot(...combinedForce), thrust3D, 1e-12),
  "2-axis TVC should rotate thrust without changing its magnitude",
);

// Euler rigid-body equation must include omega x (I omega), not only tau / I.
assert.ok(
  vectorNearly(
    angularAccelerationBody3D([1, 2, 3], [0, 0, 0], [2, 3, 4]),
    [-3, 2, -0.5],
    1e-12,
  ),
  "asymmetric inertia should produce the expected gyroscopic coupling",
);

// The +Z slice of X1-2 must exactly preserve X1-1 torque/acceleration signs.
const sliceTheta = degToRad(7);
const sliceOmega = 0.12;
const sliceDeltaZ = degToRad(-5);
const sliceDt = 0.0005;
const oneAxisSlice = stepOneAxis(
  { thetaRad: sliceTheta, omegaRadS: sliceOmega },
  {
    mode: "manual",
    deltaCommandRad: sliceDeltaZ,
    flowRateM3s: lpmToM3s(8),
  },
  sliceDt,
  DEFAULT_ONE_AXIS_PARAMS,
);
const rigidSlice = stepRigidBody3D(
  createRigidBodyState({
    qBodyToWorld: quatFromAxisAngle([0, 0, 1], sliceTheta),
    omegaBodyRadS: [0, 0, sliceOmega],
  }),
  {
    deltaZCommandRad: sliceDeltaZ,
    flowRateM3s: lpmToM3s(8),
  },
  sliceDt,
  DEFAULT_RIGID_BODY_PARAMS,
);
assert.ok(
  nearly(
    oneAxisSlice.diagnostics.alphaRadS2,
    rigidSlice.diagnostics.alphaBodyRadS2[2],
    1e-12,
  ),
  "X1-2 +Z slice should reproduce X1-1 angular acceleration",
);
assert.ok(
  nearly(
    oneAxisSlice.state.omegaRadS,
    rigidSlice.state.omegaBodyRadS[2],
    1e-12,
  ),
  "X1-2 +Z slice should reproduce X1-1 angular velocity update",
);

// X and Z gimbal commands are saturated independently and retain raw commands.
const saturated3D = stepRigidBody3D(
  createRigidBodyState(),
  {
    deltaXCommandRad: degToRad(60),
    deltaZCommandRad: degToRad(-70),
    flowRateM3s: lpmToM3s(20),
  },
  0.001,
  DEFAULT_RIGID_BODY_PARAMS,
);
assert.equal(
  saturated3D.diagnostics.gimbalXSaturated,
  true,
  "delta_x should report saturation",
);
assert.equal(
  saturated3D.diagnostics.gimbalZSaturated,
  true,
  "delta_z should report saturation",
);
assert.equal(
  saturated3D.diagnostics.flowSaturated,
  true,
  "3D flow should report saturation",
);
assert.ok(
  nearly(
    saturated3D.diagnostics.deltaXAppliedRad,
    DEFAULT_RIGID_BODY_PARAMS.gimbalMaxRad,
  ),
  "positive delta_x should clamp to +delta_max",
);
assert.ok(
  nearly(
    saturated3D.diagnostics.deltaZAppliedRad,
    -DEFAULT_RIGID_BODY_PARAMS.gimbalMaxRad,
  ),
  "negative delta_z should clamp to -delta_max",
);

// With spherical inertia and no external torque, body angular rate stays constant.
// Repeated quaternion normalization must keep the attitude on S^3.
const torqueFreeParams = {
  ...DEFAULT_RIGID_BODY_PARAMS,
  massKg: 0,
  inertiaDiagKgM2: [0.02, 0.02, 0.02],
  flowNominalM3s: 0,
  flowMaxM3s: 0,
};
let rigidState = createRigidBodyState({
  qBodyToWorld: quatFromAxisAngle([1, 2, 3], 0.7),
  omegaBodyRadS: [1.3, -0.7, 0.5],
});
for (let i = 0; i < 20000; i += 1) {
  rigidState = stepRigidBody3D(
    rigidState,
    { flowRateM3s: 0 },
    0.0005,
    torqueFreeParams,
  ).state;
}
assert.ok(
  nearly(quatNorm(rigidState.qBodyToWorld), 1, 1e-12),
  "quaternion norm should remain one after long integration",
);
assert.ok(
  vectorNearly(rigidState.omegaBodyRadS, [1.3, -0.7, 0.5], 1e-10),
  "spherical torque-free body should preserve body angular velocity",
);

console.log(
  "Shower TVC checks OK: X1-1 scalar model and X1-2 quaternion/3D rigid body",
);
