import {
  matVec,
} from "./linear-algebra.js";

function submatrixRect(matrix, rowIndices, colIndices) {
  return rowIndices.map(
    (i) => colIndices.map((j) => matrix[i][j]),
  );
}

function subtractVectors(a, b) {
  return a.map((value, i) => value - b[i]);
}

export function prescribedBaseBlocks(system) {
  const boundaryDofIndices = [0, 1];
  const freeDofIndices = system.freeDofIndices;

  return {
    boundaryDofIndices,
    freeDofIndices,
    massFreeBoundary: submatrixRect(
      system.full.mass,
      freeDofIndices,
      boundaryDofIndices,
    ),
    dampingFreeBoundary: submatrixRect(
      system.full.damping,
      freeDofIndices,
      boundaryDofIndices,
    ),
    stiffnessFreeBoundary: submatrixRect(
      system.full.stiffness,
      freeDofIndices,
      boundaryDofIndices,
    ),
  };
}

export function effectiveForceForPrescribedBase(
  system,
  boundaryState,
  freeExternalForce = system.reduced.headForce0,
) {
  const blocks = prescribedBaseBlocks(system);
  const inertia = matVec(
    blocks.massFreeBoundary,
    boundaryState.acceleration,
  );
  const damping = matVec(
    blocks.dampingFreeBoundary,
    boundaryState.velocity,
  );
  const stiffness = matVec(
    blocks.stiffnessFreeBoundary,
    boundaryState.position,
  );

  return subtractVectors(
    subtractVectors(
      subtractVectors(freeExternalForce, inertia),
      damping,
    ),
    stiffness,
  );
}

function smoothBumpUnit(r) {
  if (r <= 0 || r >= 1) {
    return {
      value: 0,
      first: 0,
      second: 0,
    };
  }

  const r2 = r * r;
  const r3 = r2 * r;
  const r4 = r3 * r;
  const r5 = r4 * r;
  const r6 = r5 * r;

  return {
    value: 64 * (r3 - 3 * r4 + 3 * r5 - r6),
    first: 64 * (3 * r2 - 12 * r3 + 15 * r4 - 6 * r5),
    second: 64 * (6 * r - 36 * r2 + 60 * r3 - 30 * r4),
  };
}

export function prescribedBasePulseState(
  timeS,
  {
    startTimeS = 0,
    durationS = 0.35,
    translationAmplitudeM = 0,
    rotationAmplitudeRad = 0,
  } = {},
) {
  if (!(durationS > 0)) throw new RangeError("durationS must be positive");

  const r = (timeS - startTimeS) / durationS;
  const bump = smoothBumpUnit(r);
  const valueScale = bump.value;
  const velocityScale = bump.first / durationS;
  const accelerationScale = bump.second / (durationS * durationS);

  return {
    position: [
      translationAmplitudeM * valueScale,
      rotationAmplitudeRad * valueScale,
    ],
    velocity: [
      translationAmplitudeM * velocityScale,
      rotationAmplitudeRad * velocityScale,
    ],
    acceleration: [
      translationAmplitudeM * accelerationScale,
      rotationAmplitudeRad * accelerationScale,
    ],
  };
}
