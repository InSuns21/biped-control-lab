import {
  clampHandTarget,
  DEFAULT_HAND_ACTUATOR_LIMITS,
} from "./hand-actuator.js";
import {
  rodKinematicsWithHandBoundary,
  stateWithHandBoundary,
  ZERO_HAND_BOUNDARY,
} from "./nonlinear-boundary.js";

export const DEFAULT_P_GAINS = Object.freeze({
  lateralPositionGain: 0.80,
  angleGain: 0.85,
});

export const DEFAULT_PD_GAINS = Object.freeze({
  lateralPositionGain: 0.95,
  lateralVelocityGainS: 0.18,
  angleGain: 1.05,
  angularRateGainS: 0.16,
});

function wrapAngleRad(value) {
  return Math.atan2(Math.sin(value), Math.cos(value));
}

function boundaryWithDefaults(boundary = {}) {
  return {
    ...ZERO_HAND_BOUNDARY,
    ...boundary,
  };
}

function tipVelocityFromState(
  system,
  stateInput,
  boundaryInput,
) {
  const boundary = boundaryWithDefaults(boundaryInput);
  const state = stateWithHandBoundary(
    system,
    stateInput,
    boundary,
  );
  const l = system.segmentLengthM;

  let vx = boundary.lateralVelocityMps;
  let vy = 0;

  for (let i = 0; i < state.anglesRad.length; i += 1) {
    const theta = state.anglesRad[i];
    const omega = state.angularRatesRadS[i];
    vx += l * Math.cos(theta) * omega;
    vy -= l * Math.sin(theta) * omega;
  }

  return [vx, vy];
}

export function senseTipFeedback(
  system,
  stateInput,
  boundaryInput,
  equilibriumKinematics,
) {
  if (!equilibriumKinematics?.tip) {
    throw new RangeError(
      "equilibriumKinematics with tip is required",
    );
  }

  const boundary = boundaryWithDefaults(boundaryInput);
  const current = rodKinematicsWithHandBoundary(
    system,
    stateInput,
    boundary,
  );
  const tipVelocity = tipVelocityFromState(
    system,
    stateInput,
    boundary,
  );
  const state = stateWithHandBoundary(
    system,
    stateInput,
    boundary,
  );

  return {
    tipLateralErrorM:
      current.tip[0] - equilibriumKinematics.tip[0],
    tipVerticalErrorM:
      current.tip[1] - equilibriumKinematics.tip[1],
    tipLateralVelocityMps: tipVelocity[0],
    tipVerticalVelocityMps: tipVelocity[1],
    tipAngleErrorRad: wrapAngleRad(
      current.tipAngleRad - equilibriumKinematics.tipAngleRad,
    ),
    tipAngularRateRadS: state.angularRatesRadS.at(-1),
    currentTip: [...current.tip],
    currentTipAngleRad: current.tipAngleRad,
  };
}

export function pHandTarget(
  sensing,
  gains = DEFAULT_P_GAINS,
  limits = DEFAULT_HAND_ACTUATOR_LIMITS,
) {
  return clampHandTarget(
    {
      lateralPositionM:
        -gains.lateralPositionGain
        * sensing.tipLateralErrorM,
      angleRad:
        -gains.angleGain
        * sensing.tipAngleErrorRad,
    },
    limits,
  );
}

export function pdHandTarget(
  sensing,
  gains = DEFAULT_PD_GAINS,
  limits = DEFAULT_HAND_ACTUATOR_LIMITS,
) {
  return clampHandTarget(
    {
      lateralPositionM:
        -gains.lateralPositionGain
        * sensing.tipLateralErrorM
        - gains.lateralVelocityGainS
        * sensing.tipLateralVelocityMps,
      angleRad:
        -gains.angleGain
        * sensing.tipAngleErrorRad
        - gains.angularRateGainS
        * sensing.tipAngularRateRadS,
    },
    limits,
  );
}

export function controllerHandTarget(
  mode,
  sensing,
  {
    pGains = DEFAULT_P_GAINS,
    pdGains = DEFAULT_PD_GAINS,
    limits = DEFAULT_HAND_ACTUATOR_LIMITS,
  } = {},
) {
  if (mode === "p") {
    return pHandTarget(sensing, pGains, limits);
  }
  if (mode === "pd") {
    return pdHandTarget(sensing, pdGains, limits);
  }
  if (mode === "human" || mode === "off") {
    return null;
  }
  throw new RangeError(`unknown control mode: ${mode}`);
}
