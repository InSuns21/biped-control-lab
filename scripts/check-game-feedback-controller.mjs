import assert from "node:assert/strict";
import {
  DEFAULT_HAND_ACTUATOR_LIMITS,
} from "../docs/js/shower/flexible/hand-actuator.js";
import {
  aimingHandReference,
  referenceEquilibriumKinematics,
  rotateRodVector,
  transformRodPoint,
} from "../docs/js/shower/flexible/game-feedback-controller.js";
import {
  createAimTarget,
} from "../docs/js/shower/flexible/game.js";

const nearly = (a, b, tol = 1e-10) =>
  Math.abs(a - b) <= tol;

const equilibriumKinematics = {
  nodes: [
    [0, 0],
    [0.08, 0.45],
    [0.16, 0.92],
  ],
  tip: [0.16, 0.92],
  tipAngleRad: 0.18,
};

const equilibriumReaction = {
  nozzleOffsetWorldM: [0.06, 0.04],
  outletDirection: [0.42, 0.907524],
};

const nozzle = [
  equilibriumKinematics.tip[0]
    + equilibriumReaction.nozzleOffsetWorldM[0],
  equilibriumKinematics.tip[1]
    + equilibriumReaction.nozzleOffsetWorldM[1],
];

// Centered target must preserve the original equilibrium reference.
const centered = createAimTarget({
  nozzleOrigin: nozzle,
  outletDirection: equilibriumReaction.outletDirection,
  distanceM: 0.5,
  normalOffsetM: 0,
  radiusM: 0.1,
});
const centeredRef = aimingHandReference({
  equilibriumKinematics,
  equilibriumReaction,
  target: centered,
  limits: DEFAULT_HAND_ACTUATOR_LIMITS,
});
assert.ok(nearly(centeredRef.target.lateralPositionM, 0, 2e-6));
assert.ok(nearly(centeredRef.target.angleRad, 0, 2e-6));
assert.ok(Math.abs(centeredRef.residualSignedMissM) < 2e-6);

// Moving the bullseye normal to the water ray must require a real hand pose.
const shifted = createAimTarget({
  nozzleOrigin: nozzle,
  outletDirection: equilibriumReaction.outletDirection,
  distanceM: 0.5,
  normalOffsetM: 0.14,
  radiusM: 0.09,
});
const shiftedRef = aimingHandReference({
  equilibriumKinematics,
  equilibriumReaction,
  target: shifted,
  limits: DEFAULT_HAND_ACTUATOR_LIMITS,
});
assert.ok(
  Math.abs(shiftedRef.target.angleRad) > 0.05,
  "shifted target must command a visible hand angle",
);
assert.ok(
  Math.abs(shiftedRef.target.lateralPositionM)
    <= DEFAULT_HAND_ACTUATOR_LIMITS.lateralMaxM + 1e-12,
);
assert.ok(
  Math.abs(shiftedRef.target.angleRad)
    <= DEFAULT_HAND_ACTUATOR_LIMITS.angleMaxRad + 1e-12,
);
assert.ok(
  Math.abs(shiftedRef.residualSignedMissM) < 0.03,
  "rigid reference should geometrically bring the ray near the target",
);

// Reference kinematics must apply the same rigid transform to every point.
const transformed = referenceEquilibriumKinematics(
  equilibriumKinematics,
  shiftedRef.target,
);
const expectedTip = transformRodPoint(
  equilibriumKinematics.tip,
  shiftedRef.target,
);
assert.ok(nearly(transformed.tip[0], expectedTip[0]));
assert.ok(nearly(transformed.tip[1], expectedTip[1]));
assert.ok(nearly(
  transformed.tipAngleRad,
  equilibriumKinematics.tipAngleRad
    + shiftedRef.target.angleRad,
));

const unitDown = rotateRodVector([0, 1], Math.PI / 2);
assert.ok(nearly(unitDown[0], 1));
assert.ok(nearly(unitDown[1], 0));

console.log(
  "H1-6-3 aiming reference checks OK:",
  JSON.stringify({
    centered: centeredRef,
    shifted: shiftedRef,
    transformedTip: transformed.tip,
  }),
);
