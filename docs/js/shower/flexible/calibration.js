import {
  solveLinear,
} from "./linear-algebra.js";
import {
  createNewmarkGeneralLinear,
} from "./integrator.js";
import {
  referenceInitialPerturbationReduced,
} from "./scenarios.js";
import {
  effectiveForceForPrescribedBase,
  prescribedBasePulseState,
} from "./boundary.js";

export const FAST_ONSET_TARGET = Object.freeze({
  onsetMinS: 1,
  onsetMaxS: 3,
  rmsFactor: 3,
  absoluteRmsM: 0.020,
  maxDynamicDisplacementM: 0.18,
  maxDynamicRotationRad: 0.35,
});

export function staticEquilibrium(system) {
  return solveLinear(
    system.reduced.stiffness,
    system.reduced.headForce0,
  );
}

export function stateDynamicMetrics(
  system,
  state,
  equilibrium,
) {
  let sumSq = 0;
  let displacementCount = 0;
  let maxDisplacementM = 0;
  let maxRotationRad = 0;

  for (let node = 1; node < system.nodeCount; node += 1) {
    const index = 2 * (node - 1);
    const displacement = state.q[index] - equilibrium[index];
    const rotation = state.q[index + 1] - equilibrium[index + 1];
    sumSq += displacement * displacement;
    displacementCount += 1;
    maxDisplacementM = Math.max(maxDisplacementM, Math.abs(displacement));
    maxRotationRad = Math.max(maxRotationRad, Math.abs(rotation));
  }

  return {
    rmsM: Math.sqrt(sumSq / Math.max(1, displacementCount)),
    maxDisplacementM,
    maxRotationRad,
    tipDisplacementM: state.q.at(-2) - equilibrium.at(-2),
    tipRotationRad: state.q.at(-1) - equilibrium.at(-1),
  };
}

export function equilibriumValidity(
  system,
  equilibrium,
  {
    maxDisplacementM = 0.18,
    maxRotationRad = 0.35,
  } = FAST_ONSET_TARGET,
) {
  let maximumDisplacementM = 0;
  let maximumRotationRad = 0;

  for (let node = 1; node < system.nodeCount; node += 1) {
    const index = 2 * (node - 1);
    maximumDisplacementM = Math.max(
      maximumDisplacementM,
      Math.abs(equilibrium[index]),
    );
    maximumRotationRad = Math.max(
      maximumRotationRad,
      Math.abs(equilibrium[index + 1]),
    );
  }

  return {
    valid: maximumDisplacementM <= maxDisplacementM
      && maximumRotationRad <= maxRotationRad,
    maximumDisplacementM,
    maximumRotationRad,
  };
}

export function initialCurvaturePerturbationReduced(
  system,
  {
    tipOffsetM = 0.008,
    tipRotationRad = 0,
    velocityAmplitudeMps = 0,
  } = {},
) {
  const q = [];
  const v = [];
  const L = system.params.lengthM;
  const lastNode = system.nodeCount - 1;

  for (let node = 1; node <= lastNode; node += 1) {
    const x = node / lastNode;
    const f = x * x * (3 - 2 * x);
    const dfDx = 6 * x * (1 - x);
    const g = x * x * (1 - x);
    const dgDx = 2 * x - 3 * x * x;

    const displacement = tipOffsetM * f
      - tipRotationRad * L * g;
    const rotation = tipOffsetM * dfDx / L
      - tipRotationRad * dgDx;

    const phase = 2 * Math.PI * x;
    const velocity = velocityAmplitudeMps
      * x * x
      * Math.sin(phase);
    const angularVelocity = velocityAmplitudeMps / L
      * (
        2 * x * Math.sin(phase)
        + 2 * Math.PI * x * x * Math.cos(phase)
      );

    q.push(displacement);
    q.push(rotation);
    v.push(velocity);
    v.push(angularVelocity);
  }

  return { q, v };
}

export function predictedFactorTimeS(growthRatePerS, factor) {
  if (!(growthRatePerS > 0) || !(factor > 1)) return Infinity;
  return Math.log(factor) / growthRatePerS;
}

export function simulateOnset(
  system,
  {
    durationS = 8,
    dt = 0.002,
    displacementAmplitudeM = 0.008,
    velocityAmplitudeMps = 0.02,
    basePulse = null,
    initialPerturbation = null,
    target = FAST_ONSET_TARGET,
  } = {},
) {
  const equilibrium = staticEquilibrium(system);
  const equilibriumCheck = equilibriumValidity(system, equilibrium, target);
  const initial = initialPerturbation
    ?? referenceInitialPerturbationReduced(system, {
      displacementAmplitudeM,
      velocityAmplitudeMps,
    });
  const integrator = createNewmarkGeneralLinear(system.reduced, dt);
  let state = integrator.initialize({
    q: equilibrium.map((value, i) => value + initial.q[i]),
    v: initial.v,
    force: system.reduced.headForce0,
  });

  const initialMetrics = stateDynamicMetrics(system, state, equilibrium);
  const onsetThresholdM = Math.max(
    target.absoluteRmsM,
    target.rmsFactor * initialMetrics.rmsM,
  );

  let onsetTimeS = null;
  let guardTimeS = null;
  let maxObservedRmsM = initialMetrics.rmsM;

  const steps = Math.round(durationS / dt);
  for (let step = 0; step < steps; step += 1) {
    const timeNextS = (step + 1) * dt;
    let forceNext = system.reduced.headForce0;

    if (basePulse) {
      const boundaryState = prescribedBasePulseState(
        timeNextS,
        basePulse,
      );
      forceNext = effectiveForceForPrescribedBase(
        system,
        boundaryState,
      );
    }

    state = integrator.step(state, forceNext);
    if (!state.q.every(Number.isFinite) || !state.v.every(Number.isFinite)) {
      return {
        equilibrium,
        equilibriumCheck,
        initialRmsM: initialMetrics.rmsM,
        onsetThresholdM,
        onsetTimeS,
        guardTimeS: guardTimeS ?? timeNextS,
        maxObservedRmsM: Infinity,
        numericalFailure: true,
      };
    }

    const metrics = stateDynamicMetrics(system, state, equilibrium);
    maxObservedRmsM = Math.max(maxObservedRmsM, metrics.rmsM);

    if (
      onsetTimeS === null
      && timeNextS >= target.onsetMinS
      && metrics.rmsM >= onsetThresholdM
    ) {
      onsetTimeS = timeNextS;
    }

    if (
      guardTimeS === null
      && (
        metrics.maxDisplacementM > target.maxDynamicDisplacementM
        || metrics.maxRotationRad > target.maxDynamicRotationRad
      )
    ) {
      guardTimeS = timeNextS;
      break;
    }
  }

  return {
    equilibrium,
    equilibriumCheck,
    initialRmsM: initialMetrics.rmsM,
    onsetThresholdM,
    onsetTimeS,
    guardTimeS,
    maxObservedRmsM,
    numericalFailure: false,
  };
}
