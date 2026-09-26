// Quaternion convention: [w, x, y, z], mapping body-frame vectors to world.
export function quatIdentity() {
  return [1, 0, 0, 0];
}

export function quatNorm(q) {
  return Math.hypot(q[0], q[1], q[2], q[3]);
}

export function quatNormalize(q) {
  const n = quatNorm(q);
  if (!(n > 0)) throw new RangeError("quaternion norm must be positive");
  return q.map((x) => x / n);
}

export function quatConjugate(q) {
  return [q[0], -q[1], -q[2], -q[3]];
}

export function quatMultiply(a, b) {
  const [aw, ax, ay, az] = a;
  const [bw, bx, by, bz] = b;
  return [
    aw * bw - ax * bx - ay * by - az * bz,
    aw * bx + ax * bw + ay * bz - az * by,
    aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw,
  ];
}

export function quatFromAxisAngle(axis, angleRad) {
  const n = Math.hypot(axis[0], axis[1], axis[2]);
  if (!(n > 0)) throw new RangeError("axis norm must be positive");
  const half = angleRad / 2;
  const s = Math.sin(half) / n;
  return quatNormalize([
    Math.cos(half),
    axis[0] * s,
    axis[1] * s,
    axis[2] * s,
  ]);
}

export function quatRotateVector(qInput, vector) {
  const q = quatNormalize(qInput);
  const p = [0, vector[0], vector[1], vector[2]];
  const rotated = quatMultiply(quatMultiply(q, p), quatConjugate(q));
  return rotated.slice(1);
}

export function quatInverseRotateVector(qInput, vector) {
  const q = quatNormalize(qInput);
  const p = [0, vector[0], vector[1], vector[2]];
  const rotated = quatMultiply(quatMultiply(quatConjugate(q), p), q);
  return rotated.slice(1);
}

export function quatIntegrateBodyRate(qInput, omegaBodyRadS, dt) {
  if (!(dt > 0)) throw new RangeError("dt must be positive");
  const q = quatNormalize(qInput);
  const omegaQuat = [0, ...omegaBodyRadS];

  // q maps body -> world and omega is expressed in body coordinates:
  // q_dot = 1/2 q ⊗ [0, omega_body].
  const qDot = quatMultiply(q, omegaQuat).map((x) => 0.5 * x);
  return quatNormalize(q.map((x, i) => x + qDot[i] * dt));
}
