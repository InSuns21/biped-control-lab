export const DEFAULT_HAND_ACTUATOR_LIMITS = Object.freeze({
  lateralMinM: -0.08,
  lateralMaxM: 0.08,
  lateralMaxSpeedMps: 0.45,
  lateralMaxAccelerationMps2: 4.0,
  angleMinRad: -30 * Math.PI / 180,
  angleMaxRad: 30 * Math.PI / 180,
  angularMaxSpeedRadS: 2.8,
  angularMaxAccelerationRadS2: 20,
  lateralNaturalRatePerS: 12,
  angularNaturalRatePerS: 14,
});

export function clamp(value, lower, upper) {
  return Math.min(upper, Math.max(lower, value));
}

export function clampHandTarget(
  target,
  limits = DEFAULT_HAND_ACTUATOR_LIMITS,
) {
  return {
    lateralPositionM: clamp(
      target.lateralPositionM ?? 0,
      limits.lateralMinM,
      limits.lateralMaxM,
    ),
    angleRad: clamp(
      target.angleRad ?? 0,
      limits.angleMinRad,
      limits.angleMaxRad,
    ),
  };
}

export function createHandActuatorState(overrides = {}) {
  return {
    lateralPositionM: overrides.lateralPositionM ?? 0,
    lateralVelocityMps: overrides.lateralVelocityMps ?? 0,
    angleRad: overrides.angleRad ?? 0,
    angularRateRadS: overrides.angularRateRadS ?? 0,
  };
}

function axisStep({
  position,
  velocity,
  target,
  minPosition,
  maxPosition,
  maxSpeed,
  maxAcceleration,
  naturalRate,
  dt,
}) {
  const omega = naturalRate;
  const desiredAcceleration = omega * omega * (target - position)
    - 2 * omega * velocity;
  let acceleration = clamp(
    desiredAcceleration,
    -maxAcceleration,
    maxAcceleration,
  );

  let nextVelocity = clamp(
    velocity + acceleration * dt,
    -maxSpeed,
    maxSpeed,
  );
  acceleration = (nextVelocity - velocity) / dt;

  let nextPosition = position
    + 0.5 * (velocity + nextVelocity) * dt;

  let saturatedPosition = false;
  if (nextPosition < minPosition) {
    nextPosition = minPosition;
    if (nextVelocity < 0) nextVelocity = 0;
    saturatedPosition = true;
  } else if (nextPosition > maxPosition) {
    nextPosition = maxPosition;
    if (nextVelocity > 0) nextVelocity = 0;
    saturatedPosition = true;
  }

  acceleration = (nextVelocity - velocity) / dt;

  return {
    position: nextPosition,
    velocity: nextVelocity,
    acceleration,
    saturatedAcceleration:
      Math.abs(desiredAcceleration) > maxAcceleration + 1e-12,
    saturatedSpeed:
      Math.abs(velocity + desiredAcceleration * dt)
        > maxSpeed + 1e-12,
    saturatedPosition,
  };
}

export function stepHandActuator(
  stateInput,
  targetInput,
  dt,
  limits = DEFAULT_HAND_ACTUATOR_LIMITS,
) {
  if (!(dt > 0)) throw new RangeError("dt must be positive");
  const state = createHandActuatorState(stateInput);
  const target = clampHandTarget(targetInput, limits);

  const lateral = axisStep({
    position: state.lateralPositionM,
    velocity: state.lateralVelocityMps,
    target: target.lateralPositionM,
    minPosition: limits.lateralMinM,
    maxPosition: limits.lateralMaxM,
    maxSpeed: limits.lateralMaxSpeedMps,
    maxAcceleration: limits.lateralMaxAccelerationMps2,
    naturalRate: limits.lateralNaturalRatePerS,
    dt,
  });

  const angular = axisStep({
    position: state.angleRad,
    velocity: state.angularRateRadS,
    target: target.angleRad,
    minPosition: limits.angleMinRad,
    maxPosition: limits.angleMaxRad,
    maxSpeed: limits.angularMaxSpeedRadS,
    maxAcceleration: limits.angularMaxAccelerationRadS2,
    naturalRate: limits.angularNaturalRatePerS,
    dt,
  });

  return {
    state: {
      lateralPositionM: lateral.position,
      lateralVelocityMps: lateral.velocity,
      angleRad: angular.position,
      angularRateRadS: angular.velocity,
    },
    target,
    acceleration: {
      lateralAccelerationMps2: lateral.acceleration,
      angularAccelerationRadS2: angular.acceleration,
    },
    saturation: {
      lateralAcceleration: lateral.saturatedAcceleration,
      lateralSpeed: lateral.saturatedSpeed,
      lateralPosition: lateral.saturatedPosition,
      angularAcceleration: angular.saturatedAcceleration,
      angularSpeed: angular.saturatedSpeed,
      angularPosition: angular.saturatedPosition,
    },
  };
}

export function actuatorBoundaryTrajectory(
  startStateInput,
  actuatorStep,
  dt,
) {
  const start = createHandActuatorState(startStateInput);
  const ax = actuatorStep.acceleration.lateralAccelerationMps2;
  const alpha = actuatorStep.acceleration.angularAccelerationRadS2;

  return (offsetS) => {
    const tau = clamp(offsetS, 0, dt);
    return {
      lateralPositionM: start.lateralPositionM
        + start.lateralVelocityMps * tau
        + 0.5 * ax * tau * tau,
      lateralVelocityMps:
        start.lateralVelocityMps + ax * tau,
      lateralAccelerationMps2: ax,
      angleRad: start.angleRad
        + start.angularRateRadS * tau
        + 0.5 * alpha * tau * tau,
      angularRateRadS:
        start.angularRateRadS + alpha * tau,
      angularAccelerationRadS2: alpha,
    };
  };
}

export function handActuatorLimitsLabel(
  limits = DEFAULT_HAND_ACTUATOR_LIMITS,
) {
  return {
    lateralRangeMm: [
      1000 * limits.lateralMinM,
      1000 * limits.lateralMaxM,
    ],
    lateralMaxSpeedMps: limits.lateralMaxSpeedMps,
    lateralMaxAccelerationMps2:
      limits.lateralMaxAccelerationMps2,
    angleRangeDeg: [
      limits.angleMinRad * 180 / Math.PI,
      limits.angleMaxRad * 180 / Math.PI,
    ],
    angularMaxSpeedDegS:
      limits.angularMaxSpeedRadS * 180 / Math.PI,
    angularMaxAccelerationDegS2:
      limits.angularMaxAccelerationRadS2 * 180 / Math.PI,
  };
}
