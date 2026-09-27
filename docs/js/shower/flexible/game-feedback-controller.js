import {
  clampHandTarget,
  DEFAULT_HAND_ACTUATOR_LIMITS,
} from "./hand-actuator.js";
import {
  controllerHandTargetAroundReference,
  senseTipFeedback,
} from "./feedback-controller.js";
import {
  fullStateServoHandTarget,
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

function rigidReferenceRmsM(
  equilibriumKinematics,
  referenceTarget,
) {
  const nodes = equilibriumKinematics?.nodes;
  if (!Array.isArray(nodes) || nodes.length <= 1) return 0;

  let sumSq = 0;
  for (let i = 1; i < nodes.length; i += 1) {
    const moved = transformRodPoint(
      nodes[i],
      referenceTarget,
    );
    const dx = moved[0] - nodes[i][0];
    const dy = moved[1] - nodes[i][1];
    sumSq += dx * dx + dy * dy;
  }
  return Math.sqrt(sumSq / (nodes.length - 1));
}

function aimingReferenceCandidate({
  equilibriumKinematics,
  nozzle0,
  outlet0,
  target,
  angleRad,
  limits,
}) {
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

  // A lateral base shift changes signed miss by d_y * x_hand.
  const rawLateralPositionM = Math.abs(rotatedOutlet[1]) > 1e-7
    ? -baseSignedMiss / rotatedOutlet[1]
    : 0;
  const lateralPositionM = Math.min(
    limits.lateralMaxM,
    Math.max(limits.lateralMinM, rawLateralPositionM),
  );

  const movedNozzle = [
    lateralPositionM + rotatedNozzle[0],
    rotatedNozzle[1],
  ];
  const toTarget = [
    target.center[0] - movedNozzle[0],
    target.center[1] - movedNozzle[1],
  ];
  const residualSignedMissM = cross2(
    rotatedOutlet,
    toTarget,
  );
  const forwardDistanceM = dot2(
    rotatedOutlet,
    toTarget,
  );

  const maxRayDistanceM = target.maxRayDistanceM
    ?? target.referenceDistanceM
    ?? Infinity;
  const backwardPenalty = forwardDistanceM <= 0
    ? 10 + Math.abs(forwardDistanceM)
    : 0;
  const rangePenalty = Number.isFinite(maxRayDistanceM)
    && forwardDistanceM > maxRayDistanceM
    ? 4 * (forwardDistanceM - maxRayDistanceM)
    : 0;

  // Miss distance dominates. A tiny pose regularizer makes centered targets
  // select the neutral hand pose when several solutions are equivalent.
  const xRange = Math.max(
    1e-9,
    limits.lateralMaxM - limits.lateralMinM,
  );
  const angleRange = Math.max(
    1e-9,
    limits.angleMaxRad - limits.angleMinRad,
  );
  const referenceRmsM = rigidReferenceRmsM(
    equilibriumKinematics,
    {
      lateralPositionM,
      angleRad,
    },
  );
  const poseRegularizer = 1e-6 * (
    Math.abs(lateralPositionM) / xRange
    + Math.abs(angleRad) / angleRange
  );

  return {
    angleRad,
    lateralPositionM,
    rawLateralPositionM,
    outletDirection: rotatedOutlet,
    residualSignedMissM,
    forwardDistanceM,
    referenceRmsM,
    cost:
      Math.abs(residualSignedMissM)
      + backwardPenalty
      + rangePenalty
      + poseRegularizer,
  };
}

function searchAimingReference({
  equilibriumKinematics,
  nozzle0,
  outlet0,
  target,
  limits,
}) {
  let lower = limits.angleMinRad;
  let upper = limits.angleMaxRad;
  let best = null;

  // Coarse search followed by two deterministic local refinements. This is
  // cheap enough to run each physics step and, unlike "angle first, x second",
  // respects both hand travel constraints jointly.
  for (let pass = 0; pass < 3; pass += 1) {
    const samples = pass === 0 ? 41 : 17;
    const spacing = (upper - lower) / (samples - 1);

    for (let i = 0; i < samples; i += 1) {
      const angleRad = lower + i * spacing;
      const candidate = aimingReferenceCandidate({
        equilibriumKinematics,
        nozzle0,
        outlet0,
        target,
        angleRad,
        limits,
      });
      if (
        best === null
        || candidate.cost < best.cost
      ) {
        best = candidate;
      }
    }

    if (pass < 2) {
      lower = Math.max(
        limits.angleMinRad,
        best.angleRad - spacing,
      );
      upper = Math.min(
        limits.angleMaxRad,
        best.angleRad + spacing,
      );
    }
  }

  return best;
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
      forwardDistanceM: 0,
    };
  }

  const nozzle0 = equilibriumNozzleOrigin(
    equilibriumKinematics,
    equilibriumReaction,
  );
  const outlet0 = normalize2(
    equilibriumReaction.outletDirection,
  );
  const best = searchAimingReference({
    equilibriumKinematics,
    nozzle0,
    outlet0,
    target,
    limits,
  });

  return {
    target: {
      lateralPositionM: best.lateralPositionM,
      angleRad: best.angleRad,
    },
    rawTarget: {
      lateralPositionM: best.rawLateralPositionM,
      angleRad: best.angleRad,
    },
    desiredOutletDirection: [...best.outletDirection],
    residualSignedMissM: best.residualSignedMissM,
    forwardDistanceM: best.forwardDistanceM,
    referenceRmsM: best.referenceRmsM,
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

  const stateFeedback = fullStateServoHandTarget(
    lqrDesign,
    rodState,
    actuatorState,
    {
      feedforwardTarget: aiming.target,
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
