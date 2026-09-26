import assert from "node:assert/strict";
import {
  DEFAULT_ONE_AXIS_PARAMS,
  DEFAULT_PD_GAINS,
  degToRad,
  feedforwardHoldTilt1D,
  gravityTorque1D,
  hoseHoldingTorque1D,
  jetThrustFromFlow,
  lpmToM3s,
  stepOneAxis,
  waterReactionTorque1D,
} from "../docs/js/shower/one-axis.js";
import {
  quatFromAxisAngle,
  quatNorm,
} from "../docs/js/shower/quaternion.js";
import {
  DEFAULT_RIGID_BODY_PARAMS,
  angularAccelerationBody3D,
  createRigidBodyState,
  gravityTorqueBody3D,
  hoseHoldingTorqueBody3D,
  jetDownErrorRad,
  stepRigidBody3D,
  waterReactionTorqueBody3D,
} from "../docs/js/shower/rigid-body.js";

const params = DEFAULT_ONE_AXIS_PARAMS;
const nearly = (a, b, tol = 1e-12) => Math.abs(a - b) <= tol;
const vectorNearly = (a, b, tol = 1e-12) => (
  a.length === b.length && a.every((x, i) => nearly(x, b[i], tol))
);

assert.equal(
  jetThrustFromFlow(0, params),
  0,
  "Q = 0 should produce zero water reaction",
);

const thrust4Lpm = jetThrustFromFlow(lpmToM3s(4), params);
const thrust8Lpm = jetThrustFromFlow(lpmToM3s(8), params);
assert.ok(
  nearly(thrust8Lpm / thrust4Lpm, 4),
  "doubling flow should quadruple the idealized momentum-flux force",
);

assert.ok(
  gravityTorque1D(degToRad(5), params) < 0,
  "with the hand above the COM, gravity must restore a positive tilt",
);

assert.ok(
  waterReactionTorque1D(thrust8Lpm, params) > 0,
  "the offset downward jet should create the documented +Z reaction torque",
);

assert.ok(
  hoseHoldingTorque1D(0, 0, degToRad(-5), params) < 0,
  "counter-tilting the held hose should create a counteracting -Z torque",
);

const feedforwardTilt = feedforwardHoldTilt1D(
  params.flowNominalM3s,
  params,
);
const balanced = stepOneAxis(
  { thetaRad: 0, omegaRadS: 0 },
  {
    mode: "manual",
    holdTiltCommandRad: feedforwardTilt,
    flowRateM3s: params.flowNominalM3s,
  },
  0.001,
  params,
);
assert.ok(
  Math.abs(balanced.diagnostics.totalTorqueNm) < 1e-12,
  "feedforward hand tilt should balance the nominal off-axis water torque at vertical",
);

let pdState = { thetaRad: degToRad(10), omegaRadS: 0 };
for (let t = 0; t < 5; t += 0.001) {
  pdState = stepOneAxis(
    pdState,
    { mode: "pd", ...DEFAULT_PD_GAINS },
    0.001,
    params,
  ).state;
}
assert.ok(
  Math.abs(pdState.thetaRad) < degToRad(0.1),
  "PD plus nominal-flow feedforward should return the hanging head close to vertical",
);
assert.ok(
  Math.abs(pdState.omegaRadS) < 0.01,
  "PD should damp the hanging-head angular velocity",
);

const saturated = stepOneAxis(
  { thetaRad: 0, omegaRadS: 0 },
  {
    mode: "manual",
    holdTiltCommandRad: degToRad(60),
    flowRateM3s: lpmToM3s(20),
  },
  0.001,
  params,
);
assert.equal(saturated.diagnostics.holdTiltSaturated, true);
assert.equal(saturated.diagnostics.flowSaturated, true);
assert.ok(
  nearly(
    saturated.diagnostics.holdTiltAppliedRad,
    params.holdTiltMaxRad,
  ),
  "held-hose direction should clamp to the physical limit",
);

// 3D identity is the intended hanging pose: body -Y / water jet points world down.
const identity = createRigidBodyState();
assert.ok(
  nearly(jetDownErrorRad(identity.qBodyToWorld), 0),
  "identity attitude should point the shower jet straight down",
);

// Gravity is restoring in both tilt axes because the COM lies below the hand.
const qTiltZ = quatFromAxisAngle([0, 0, 1], degToRad(7));
const gravityZ = gravityTorqueBody3D(qTiltZ, DEFAULT_RIGID_BODY_PARAMS);
assert.ok(
  gravityZ[2] < 0,
  "positive +Z tilt should receive negative restoring gravity torque",
);

const thrust3D = jetThrustFromFlow(
  DEFAULT_RIGID_BODY_PARAMS.flowNominalM3s,
  DEFAULT_RIGID_BODY_PARAMS,
);
const waterTorque3D = waterReactionTorqueBody3D(
  thrust3D,
  DEFAULT_RIGID_BODY_PARAMS,
);
assert.ok(
  waterTorque3D[0] < 0 && waterTorque3D[2] > 0,
  "the offset nozzle line should create the expected 3D water-reaction torque",
);

const holdCounter = hoseHoldingTorqueBody3D(
  identity.qBodyToWorld,
  [0, 0, 0],
  degToRad(5),
  degToRad(-10),
  DEFAULT_RIGID_BODY_PARAMS,
);
assert.ok(
  holdCounter[0] > 0 && holdCounter[2] < 0,
  "hand-controlled hose direction should oppose the nominal water torque in both tilt axes",
);

// Euler rigid-body equation must include omega x (I omega).
assert.ok(
  vectorNearly(
    angularAccelerationBody3D([1, 2, 3], [0, 0, 0], [2, 3, 4]),
    [-3, 2, -0.5],
    1e-12,
  ),
  "asymmetric inertia should produce the expected gyroscopic coupling",
);

const saturated3D = stepRigidBody3D(
  createRigidBodyState(),
  {
    holdTiltXCommandRad: degToRad(60),
    holdTiltZCommandRad: degToRad(-70),
    flowRateM3s: lpmToM3s(20),
  },
  0.001,
  DEFAULT_RIGID_BODY_PARAMS,
);
assert.equal(saturated3D.diagnostics.holdTiltXSaturated, true);
assert.equal(saturated3D.diagnostics.holdTiltZSaturated, true);
assert.equal(saturated3D.diagnostics.flowSaturated, true);
assert.ok(
  nearly(
    saturated3D.diagnostics.holdTiltXAppliedRad,
    DEFAULT_RIGID_BODY_PARAMS.holdTiltMaxRad,
  ),
);
assert.ok(
  nearly(
    saturated3D.diagnostics.holdTiltZAppliedRad,
    -DEFAULT_RIGID_BODY_PARAMS.holdTiltMaxRad,
  ),
);

// With no forces, no spring/damping, and spherical inertia, angular rate is constant.
const torqueFreeParams = {
  ...DEFAULT_RIGID_BODY_PARAMS,
  massKg: 0,
  hoseSpringNm: 0,
  hoseDampingNms: 0,
  yawDampingNms: 0,
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
  "Shower checks OK: hanging hose geometry, restoring gravity, water reaction, holding torque, quaternion 3D",
);
