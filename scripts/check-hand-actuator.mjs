import assert from "node:assert/strict";
import {
  actuatorBoundaryTrajectory,
  clampHandTarget,
  createHandActuatorState,
  DEFAULT_HAND_ACTUATOR_LIMITS,
  pointerDeltaToHandTarget,
  stepHandActuator,
} from "../docs/js/shower/flexible/hand-actuator.js";

const dt = 0.002;
const limits = DEFAULT_HAND_ACTUATOR_LIMITS;

// Pointer mapping: horizontal drag controls lateral target and vertical drag
// controls angle target, with both axes clamped to actuator travel.
const pointerStart = {
  lateralPositionM: 0,
  angleRad: 0,
};
const pointerMoved = pointerDeltaToHandTarget(
  pointerStart,
  {
    deltaXPx: 180,
    deltaYPx: -270,
    widthPx: 360,
    heightPx: 540,
  },
  limits,
);
assert.ok(
  Math.abs(pointerMoved.lateralPositionM - 0.08) < 1e-12,
  "half-width drag should reach +80 mm lateral target",
);
assert.ok(
  Math.abs(pointerMoved.angleRad - Math.PI / 6) < 1e-12,
  "half-height upward drag should reach +30 deg target",
);
const pointerClamped = pointerDeltaToHandTarget(
  pointerMoved,
  {
    deltaXPx: 1000,
    deltaYPx: -1000,
    widthPx: 360,
    heightPx: 540,
  },
  limits,
);
assert.equal(pointerClamped.lateralPositionM, limits.lateralMaxM);
assert.equal(pointerClamped.angleRad, limits.angleMaxRad);


let state = createHandActuatorState();
const target = clampHandTarget({
  lateralPositionM: 0.20,
  angleRad: Math.PI,
});
assert.equal(target.lateralPositionM, limits.lateralMaxM);
assert.equal(target.angleRad, limits.angleMaxRad);

let maxSpeed = 0;
let maxAccel = 0;
let maxAngularSpeed = 0;
let maxAngularAccel = 0;

for (let i = 0; i < 1800; i += 1) {
  const step = stepHandActuator(state, target, dt);
  maxSpeed = Math.max(
    maxSpeed,
    Math.abs(step.state.lateralVelocityMps),
  );
  maxAccel = Math.max(
    maxAccel,
    Math.abs(step.acceleration.lateralAccelerationMps2),
  );
  maxAngularSpeed = Math.max(
    maxAngularSpeed,
    Math.abs(step.state.angularRateRadS),
  );
  maxAngularAccel = Math.max(
    maxAngularAccel,
    Math.abs(step.acceleration.angularAccelerationRadS2),
  );
  state = step.state;
}

assert.ok(
  Math.abs(state.lateralPositionM - target.lateralPositionM) < 5e-4,
  "lateral actuator should settle near its clamped target",
);
assert.ok(
  Math.abs(state.angleRad - target.angleRad) < 5e-4,
  "angular actuator should settle near its clamped target",
);
assert.ok(
  maxSpeed <= limits.lateralMaxSpeedMps + 1e-10,
  "lateral speed limit must hold",
);
assert.ok(
  maxAccel <= limits.lateralMaxAccelerationMps2 + 1e-10,
  "lateral acceleration limit must hold",
);
assert.ok(
  maxAngularSpeed <= limits.angularMaxSpeedRadS + 1e-10,
  "angular speed limit must hold",
);
assert.ok(
  maxAngularAccel <= limits.angularMaxAccelerationRadS2 + 1e-10,
  "angular acceleration limit must hold",
);

// Reversal should remain bounded and actually reverse.
const reverseTarget = {
  lateralPositionM: limits.lateralMinM,
  angleRad: limits.angleMinRad,
};
let sawNegativeVelocity = false;
let sawNegativeAngularVelocity = false;
for (let i = 0; i < 1800; i += 1) {
  const step = stepHandActuator(state, reverseTarget, dt);
  if (step.state.lateralVelocityMps < -1e-3) {
    sawNegativeVelocity = true;
  }
  if (step.state.angularRateRadS < -1e-3) {
    sawNegativeAngularVelocity = true;
  }
  assert.ok(
    Math.abs(step.state.lateralVelocityMps)
      <= limits.lateralMaxSpeedMps + 1e-10,
  );
  assert.ok(
    Math.abs(step.state.angularRateRadS)
      <= limits.angularMaxSpeedRadS + 1e-10,
  );
  state = step.state;
}
assert.ok(sawNegativeVelocity);
assert.ok(sawNegativeAngularVelocity);
assert.ok(
  Math.abs(state.lateralPositionM - reverseTarget.lateralPositionM)
    < 5e-4,
);
assert.ok(
  Math.abs(state.angleRad - reverseTarget.angleRad)
    < 5e-4,
);

// The polynomial boundary trajectory used by the RK4 boundary solver must
// agree with the actuator endpoint to integration precision when no position
// hard-stop is hit.
const start = createHandActuatorState({
  lateralPositionM: 0.01,
  lateralVelocityMps: -0.05,
  angleRad: 0.08,
  angularRateRadS: 0.2,
});
const midTarget = {
  lateralPositionM: 0.04,
  angleRad: -0.12,
};
const planned = stepHandActuator(start, midTarget, dt);
const trajectory = actuatorBoundaryTrajectory(start, planned, dt);
const end = trajectory(dt);

assert.ok(
  Math.abs(
    end.lateralPositionM - planned.state.lateralPositionM,
  ) < 1e-12,
);
assert.ok(
  Math.abs(
    end.lateralVelocityMps - planned.state.lateralVelocityMps,
  ) < 1e-12,
);
assert.ok(
  Math.abs(end.angleRad - planned.state.angleRad) < 1e-12,
);
assert.ok(
  Math.abs(
    end.angularRateRadS - planned.state.angularRateRadS,
  ) < 1e-12,
);

// Returning to neutral should stop without numerical chatter.
const neutral = {
  lateralPositionM: 0,
  angleRad: 0,
};
for (let i = 0; i < 2000; i += 1) {
  state = stepHandActuator(state, neutral, dt).state;
}
assert.ok(Math.abs(state.lateralPositionM) < 2e-4);
assert.ok(Math.abs(state.lateralVelocityMps) < 2e-3);
assert.ok(Math.abs(state.angleRad) < 2e-4);
assert.ok(Math.abs(state.angularRateRadS) < 2e-3);

console.log(
  "H1-5-1 hand actuator checks OK:",
  JSON.stringify({
    maxSpeed,
    maxAccel,
    maxAngularSpeed,
    maxAngularAccel,
    finalNeutral: state,
  }),
);
