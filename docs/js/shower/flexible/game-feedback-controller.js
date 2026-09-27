import {
  clampHandTarget,
  DEFAULT_HAND_ACTUATOR_LIMITS,
} from "./hand-actuator.js";
import {
  controllerHandTargetAroundReference,
  senseTipFeedback,
} from "./feedback-controller.js";
import {
  fullStateFeedbackHandTarget,
} from "./state-feedback-controller.js";

export const FIXED_STATE_FEEDBACK_GAIN_SCALE = 0.55;

function normalize2(vector) {
  const norm = Math.hypot(vector[0], vector[1]);
  if (!(norm > 1e-12)) {
    throw new RangeError("2D vector must be non-zero");
  }
  return [vector[0] / norm, vector[1] / norm];
}

function cross2(a, b) {
  return a[0] * b[1] - a[1] * b[0];
}

function dot2(a, b) {
  return a[0] * b[0] + a[1] * b[1];
}

export function rotateRodVector(vector, angleRad) {
  const c = Math.cos(angleRad);
  const s = Math.sin(angleRad);
  return [
    c * vector[0] + s * vector[1],
    -s * vector[0] + c * vector[1],
  ];
}

export function transformRodPoint(
  point,
  {
    lateralPositionM = 0,
    angleRad = 0,
  } = {},
) {
  const rotated = rotateRodVector(point, angleRad);
  return [
    lateralPositionM + rotated[0],
    rotated[1],
  ];
}

export function referenceEquilibriumKinematics(
  equilibriumKinematics,
  referenceTarget,
) {
  if (!equilibriumKinematics?.tip) {
    throw new RangeError("equilibriumKinematics with tip is required");
  }
  const nodes = equilibriumKinematics.nodes?.map(
    (point) => transformRodPoint(point, referenceTarget),
  );
  return {
    ...equilibriumKinematics,
    ...(nodes ? { nodes } : {}),
    tip: transformRodPoint(
      equilibriumKinematics.tip,
      referenceTarget,
    ),
    tipAngleRad:
      equilibriumKinematics.tipAngleRad
      + referenceTarget.angleRad,
  };
}

function equilibriumNozzleOrigin(
  equilibriumKinematics,
  equilibriumReaction,
) {
  return [
    equilibriumKinematics.tip[0]
      + equilibriumReaction.nozzleOffsetWorldM[0],
    equilibriumKinematics.tip[1]
      + equilibriumReaction.nozzleOffsetWorldM[1],
  ];
}

export function aimingHandReference({
  equilibriumKinematics,
  equilibriumReaction,
  target,
  limits = DEFAULT_HAND_ACTUATOR_LIMITS,
}) {
  if (!target?.center) {
    return {
      target: clampHandTarget(
        { lateralPositionM: 0, angleRad: 0 },
        limits,
      ),
      rawTarget: { lateralPositionM: 0, angleRad: 0 },
      desiredOutletDirection: [
        ...equilibriumReaction.outletDirection,
      ],
      residualSignedMissM: 0,
    };
  }

  const nozzle0 = equilibriumNozzleOrigin(
    equilibriumKinematics,
    equilibriumReaction,
  );
  const outlet0 = normalize2(
    equilibriumReaction.outletDirection,
  );
  const targetVector = [
    target.center[0] - nozzle0[0],
    target.center[1] - nozzle0[1],
  ];
  const desiredOutletDirection = normalize2(targetVector);

  // Adding a positive rod angle uses the [cos sin; -sin cos] convention,
  // hence the minus sign relative to the usual atan2(cross, dot).
  const rawAngleRad = -Math.atan2(
    cross2(outlet0, desiredOutletDirection),
    dot2(outlet0, desiredOutletDirection),
  );
  const angleRad = clampHandTarget(
    {
      lateralPositionM: 0,
      angleRad: rawAngleRad,
    },
    limits,
  ).angleRad;

  const rotatedNozzle = rotateRodVector(nozzle0, angleRad);
  const rotatedOutlet = normalize2(
    rotateRodVector(outlet0, angleRad),
  );

  const targetFromRotatedNozzle = [
    target.center[0] - rotatedNozzle[0],
    target.center[1] - rotatedNozzle[1],
  ];
  const baseSignedMiss = cross2(
    rotatedOutlet,
    targetFromRotatedNozzle,
  );

  // A lateral base shift changes signed ray miss by d_y * x_hand.
  // Solve that scalar exactly when the outlet is not horizontal.
  const rawLateralPositionM = Math.abs(rotatedOutlet[1]) > 1e-6
    ? -baseSignedMiss / rotatedOutlet[1]
    : 0;

  const referenceTarget = clampHandTarget(
    {
      lateralPositionM: rawLateralPositionM,
      angleRad,
    },
    limits,
  );

  const movedNozzle = transformRodPoint(
    nozzle0,
    referenceTarget,
  );
  const movedOutlet = normalize2(
    rotateRodVector(
      outlet0,
      referenceTarget.angleRad,
    ),
  );
  const targetFromMovedNozzle = [
    target.center[0] - movedNozzle[0],
    target.center[1] - movedNozzle[1],
  ];

  return {
    target: referenceTarget,
    rawTarget: {
      lateralPositionM: rawLateralPositionM,
      angleRad: rawAngleRad,
    },
    desiredOutletDirection,
    residualSignedMissM: cross2(
      movedOutlet,
      targetFromMovedNozzle,
    ),
  };
}

export function automaticGameHandTarget(
  mode,
  {
    system,
    rodState,
    boundary,
    actuatorState,
    equilibriumKinematics,
    equilibriumReaction,
    aimTarget,
    lqrDesign = null,
    limits = DEFAULT_HAND_ACTUATOR_LIMITS,
    stateGainScale = FIXED_STATE_FEEDBACK_GAIN_SCALE,
  },
) {
  if (!["p", "pd", "state", "lqr"].includes(mode)) {
    throw new RangeError(
      `automatic game mode must be p/pd/state/lqr, got ${mode}`,
    );
  }

  const aiming = aimingHandReference({
    equilibriumKinematics,
    equilibriumReaction,
    target: aimTarget,
    limits,
  });
  const referenceKinematics = referenceEquilibriumKinematics(
    equilibriumKinematics,
    aiming.target,
  );

  if (mode === "p" || mode === "pd") {
    const sensing = senseTipFeedback(
      system,
      rodState,
      boundary,
      referenceKinematics,
    );
    const target = controllerHandTargetAroundReference(
      mode,
      sensing,
      aiming.target,
      { limits },
    );
    return {
      mode,
      target,
      referenceTarget: aiming.target,
      referenceKinematics,
      aiming,
      sensing,
      stateFeedback: null,
    };
  }

  if (!lqrDesign) {
    throw new RangeError(
      `${mode} requires a full-state design`,
    );
  }

  const stateFeedback = fullStateFeedbackHandTarget(
    lqrDesign,
    rodState,
    actuatorState,
    {
      referenceTarget: aiming.target,
      gainScale: mode === "state"
        ? stateGainScale
        : 1,
      limits,
    },
  );

  return {
    mode,
    target: stateFeedback.target,
    referenceTarget: aiming.target,
    referenceKinematics,
    aiming,
    sensing: null,
    stateFeedback,
  };
}
