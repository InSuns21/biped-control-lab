import {
  choleskyFactor,
  matVec,
  matrixLinearCombination,
  quadraticEnergy,
  solveSpd,
  solveWithCholeskyFactor,
} from "./linear-algebra.js";

function zeroVector(length) {
  return Array(length).fill(0);
}

function assertVectorLength(vector, length, name) {
  if (vector.length !== length) {
    throw new RangeError(`${name} length must be ${length}`);
  }
}

export function mechanicalEnergy(system, state) {
  return quadraticEnergy(system.mass, state.v)
    + quadraticEnergy(system.stiffness, state.q);
}

export function createNewmarkAverageAcceleration(system, dt) {
  if (!(dt > 0)) throw new RangeError("dt must be positive");

  const { mass, damping, stiffness } = system;
  const n = mass.length;
  const beta = 0.25;
  const gamma = 0.5;

  const a0 = 1 / (beta * dt * dt);
  const a1 = gamma / (beta * dt);
  const a2 = 1 / (beta * dt);
  const a3 = 1 / (2 * beta) - 1;
  const a4 = gamma / beta - 1;
  const a5 = dt * (gamma / (2 * beta) - 1);

  const effectiveStiffness = matrixLinearCombination([
    { scale: 1, matrix: stiffness },
    { scale: a0, matrix: mass },
    { scale: a1, matrix: damping },
  ]);
  const effectiveFactor = choleskyFactor(effectiveStiffness);

  function initialize({
    q = zeroVector(n),
    v = zeroVector(n),
    force = zeroVector(n),
  } = {}) {
    assertVectorLength(q, n, "q");
    assertVectorLength(v, n, "v");
    assertVectorLength(force, n, "force");

    const cv = matVec(damping, v);
    const kq = matVec(stiffness, q);
    const rhs = force.map((value, i) => value - cv[i] - kq[i]);
    const acceleration = solveSpd(mass, rhs);

    return {
      q: [...q],
      v: [...v],
      a: acceleration,
      force: [...force],
    };
  }

  function step(state, forceNext = zeroVector(n)) {
    assertVectorLength(forceNext, n, "forceNext");

    const massPredictor = state.q.map(
      (q, i) => a0 * q + a2 * state.v[i] + a3 * state.a[i],
    );
    const dampingPredictor = state.q.map(
      (q, i) => a1 * q + a4 * state.v[i] + a5 * state.a[i],
    );

    const mTerm = matVec(mass, massPredictor);
    const cTerm = matVec(damping, dampingPredictor);
    const rhs = forceNext.map(
      (value, i) => value + mTerm[i] + cTerm[i],
    );

    const qNext = solveWithCholeskyFactor(effectiveFactor, rhs);
    const aNext = qNext.map(
      (q, i) => a0 * (q - state.q[i])
        - a2 * state.v[i]
        - a3 * state.a[i],
    );
    const vNext = state.v.map(
      (v, i) => v + dt * (
        (1 - gamma) * state.a[i]
        + gamma * aNext[i]
      ),
    );

    return {
      q: qNext,
      v: vNext,
      a: aNext,
      force: [...forceNext],
    };
  }

  return {
    dt,
    initialize,
    step,
  };
}
