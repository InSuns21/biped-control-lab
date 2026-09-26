import assert from "node:assert/strict";
import {
  EigenvalueDecomposition,
  Matrix,
} from "ml-matrix";
import {
  assembleShowerHeadConveyingBeam,
} from "../docs/js/shower/flexible/shower-head.js";
import {
  flowSpeedFromRateM3s,
} from "../docs/js/shower/flexible/conveying-flow.js";
import {
  secondOrderStateMatrix,
} from "../docs/js/shower/flexible/stability.js";
import {
  FAST_ONSET_TARGET,
  equilibriumValidity,
  initialCurvaturePerturbationReduced,
  predictedFactorTimeS,
  simulateOnset,
  staticEquilibrium,
} from "../docs/js/shower/flexible/calibration.js";
import {
  prescribedBasePulseState,
} from "../docs/js/shower/flexible/boundary.js";

const lpmToM3s = (lpm) => lpm / 60000;
const degToRad = (deg) => deg * Math.PI / 180;

const BASE = Object.freeze({
  flowLpm: 18,
  flexuralRigidityNm2: 0.7,
  lengthM: 1.2,
  headMassKg: 0.20,
  rayleighMassPerS: 0.08,
  rayleighStiffnessS: 0.0002,
});

function systemFromCase(overrides = {}) {
  const p = { ...BASE, ...overrides };
  const flowSpeedMps = flowSpeedFromRateM3s(
    lpmToM3s(p.flowLpm),
    0.006,
  );
  return assembleShowerHeadConveyingBeam({
    flowSpeedMps,
    hoseInnerDiameterM: 0.006,
    elementCount: 8,
    flexuralRigidityNm2: p.flexuralRigidityNm2,
    lengthM: p.lengthM,
    headMassKg: p.headMassKg,
    headRotInertiaAboutComKgM2: 0.01 * p.headMassKg,
    rayleighMassPerS: p.rayleighMassPerS,
    rayleighStiffnessS: p.rayleighStiffnessS,
  });
}

function eigenGrowth(system) {
  const evd = new EigenvalueDecomposition(
    new Matrix(secondOrderStateMatrix(system.reduced)),
  );
  const real = evd.realEigenvalues;
  const imag = evd.imaginaryEigenvalues;
  let index = 0;
  for (let i = 1; i < real.length; i += 1) {
    if (real[i] > real[index]) index = i;
  }
  return {
    sigmaPerS: real[index],
    omegaRadS: imag[index],
    predicted3xS: predictedFactorTimeS(real[index], 3),
    predicted5xS: predictedFactorTimeS(real[index], 5),
  };
}

function evaluateCase(label, overrides = {}) {
  const params = { ...BASE, ...overrides };
  const system = systemFromCase(overrides);
  const equilibrium = staticEquilibrium(system);
  const validity = equilibriumValidity(system, equilibrium);
  const eigen = eigenGrowth(system);
  return {
    label,
    params,
    system,
    validity,
    ...eigen,
  };
}

function shortResult(result) {
  return {
    label: result.label,
    flowLpm: result.params.flowLpm,
    EI: result.params.flexuralRigidityNm2,
    L: result.params.lengthM,
    tipMass: result.params.headMassKg,
    alphaM: result.params.rayleighMassPerS,
    betaK: result.params.rayleighStiffnessS,
    validEq: result.validity.valid,
    eqYmm: 1000 * result.validity.maximumDisplacementM,
    eqThetaDeg: result.validity.maximumRotationRad * 180 / Math.PI,
    sigma: result.sigmaPerS,
    t3: result.predicted3xS,
    t5: result.predicted5xS,
    omega: result.omegaRadS,
  };
}

function printGroup(title, results, limit = results.length) {
  console.log("\n###", title);
  for (const result of results.slice(0, limit)) {
    console.log(JSON.stringify(shortResult(result)));
  }
}

const oneAtATime = [];

for (const value of [0.15, 0.25, 0.4, 0.55, 0.7, 0.9, 1.2]) {
  oneAtATime.push(evaluateCase(`EI=${value}`, {
    flexuralRigidityNm2: value,
  }));
}
for (const value of [0, 0.02, 0.05, 0.08, 0.12, 0.15]) {
  oneAtATime.push(evaluateCase(`alphaM=${value}`, {
    rayleighMassPerS: value,
  }));
}
for (const value of [0, 0.00005, 0.0001, 0.0002, 0.00035, 0.0005]) {
  oneAtATime.push(evaluateCase(`betaK=${value}`, {
    rayleighStiffnessS: value,
  }));
}
for (const value of [8, 12, 14, 16, 18, 20, 22, 24]) {
  oneAtATime.push(evaluateCase(`Q=${value}`, {
    flowLpm: value,
  }));
}
for (const value of [0.8, 1.0, 1.2, 1.4, 1.6, 1.8]) {
  oneAtATime.push(evaluateCase(`L=${value}`, {
    lengthM: value,
  }));
}
for (const value of [0.10, 0.15, 0.20, 0.25, 0.30, 0.35]) {
  oneAtATime.push(evaluateCase(`tipMass=${value}`, {
    headMassKg: value,
  }));
}

printGroup("H1-4A one-at-a-time sensitivity", oneAtATime);

const validGrowingOneAtATime = oneAtATime
  .filter((x) => x.validity.valid && x.sigmaPerS > 0)
  .sort((a, b) => b.sigmaPerS - a.sigmaPerS);
printGroup(
  "H1-4A fastest valid one-at-a-time cases",
  validGrowingOneAtATime,
  8,
);

const combinations = [];
for (const flexuralRigidityNm2 of [0.25, 0.4, 0.55, 0.7]) {
  for (const flowLpm of [16, 18, 20, 22, 24]) {
    for (const lengthM of [1.0, 1.2, 1.4, 1.6]) {
      for (const headMassKg of [0.15, 0.20, 0.30]) {
        for (const rayleighMassPerS of [0.02, 0.05, 0.08]) {
          combinations.push(evaluateCase(
            "combo",
            {
              flexuralRigidityNm2,
              flowLpm,
              lengthM,
              headMassKg,
              rayleighMassPerS,
            },
          ));
        }
      }
    }
  }
}

const validGrowingCombinations = combinations
  .filter((x) => x.validity.valid && x.sigmaPerS > 0)
  .sort((a, b) => b.sigmaPerS - a.sigmaPerS);
assert.ok(
  validGrowingCombinations.length > 0,
  "exploratory sweep should contain at least one valid growing case",
);
printGroup(
  "H1-4A fastest valid combination cases",
  validGrowingCombinations,
  12,
);

// Validate a handful of the eigenvalue-ranked cases in time domain.
const parameterTimeDomain = validGrowingCombinations
  .slice(0, 8)
  .map((candidate) => {
    const onset = simulateOnset(candidate.system, {
      durationS: 6,
    });
    return {
      candidate,
      onset,
      fast: onset.onsetTimeS !== null
        && onset.onsetTimeS >= FAST_ONSET_TARGET.onsetMinS
        && onset.onsetTimeS <= FAST_ONSET_TARGET.onsetMaxS
        && (onset.guardTimeS === null || onset.guardTimeS > onset.onsetTimeS),
    };
  });

console.log("\n### H1-4A parameter-only time-domain candidates");
for (const x of parameterTimeDomain) {
  console.log(JSON.stringify({
    ...shortResult(x.candidate),
    onsetS: x.onset.onsetTimeS,
    guardS: x.onset.guardTimeS,
    initialRmsMm: 1000 * x.onset.initialRmsM,
    thresholdMm: 1000 * x.onset.onsetThresholdM,
    maxRmsMm: 1000 * x.onset.maxObservedRmsM,
    fast: x.fast,
  }));
}

const baselineSystem = systemFromCase();
const curvatureCases = [];
for (const tipOffsetM of [0.008, 0.02, 0.04, 0.06]) {
  for (const tipRotationDeg of [0, 5, 10]) {
    const initialPerturbation = initialCurvaturePerturbationReduced(
      baselineSystem,
      {
        tipOffsetM,
        tipRotationRad: degToRad(tipRotationDeg),
        velocityAmplitudeMps: 0.02,
      },
    );
    const onset = simulateOnset(baselineSystem, {
      durationS: 8,
      initialPerturbation,
    });
    curvatureCases.push({
      tipOffsetM,
      tipRotationDeg,
      onset,
      fast: onset.onsetTimeS !== null
        && onset.onsetTimeS >= FAST_ONSET_TARGET.onsetMinS
        && onset.onsetTimeS <= FAST_ONSET_TARGET.onsetMaxS
        && (onset.guardTimeS === null || onset.guardTimeS > onset.onsetTimeS),
    });
  }
}

console.log("\n### H1-4A initial-curvature sensitivity at baseline parameters");
for (const x of curvatureCases) {
  console.log(JSON.stringify({
    tipOffsetMm: 1000 * x.tipOffsetM,
    tipRotationDeg: x.tipRotationDeg,
    onsetS: x.onset.onsetTimeS,
    guardS: x.onset.guardTimeS,
    initialRmsMm: 1000 * x.onset.initialRmsM,
    thresholdMm: 1000 * x.onset.onsetThresholdM,
    maxRmsMm: 1000 * x.onset.maxObservedRmsM,
    fast: x.fast,
  }));
}

// Boundary pulse endpoints must be kinematically smooth.
for (const t of [0, 0.35]) {
  const state = prescribedBasePulseState(t, {
    durationS: 0.35,
    translationAmplitudeM: 0.02,
    rotationAmplitudeRad: degToRad(5),
  });
  assert.ok(state.position.every((x) => Math.abs(x) < 1e-12));
  assert.ok(state.velocity.every((x) => Math.abs(x) < 1e-12));
  assert.ok(state.acceleration.every((x) => Math.abs(x) < 1e-12));
}

const boundaryCases = [];
for (const translationAmplitudeM of [0.01, 0.02, 0.04]) {
  for (const rotationDeg of [0, 5, 10]) {
    for (const durationS of [0.20, 0.35, 0.50]) {
      const onset = simulateOnset(baselineSystem, {
        durationS: 8,
        displacementAmplitudeM: 0,
        velocityAmplitudeMps: 0,
        basePulse: {
          durationS,
          translationAmplitudeM,
          rotationAmplitudeRad: degToRad(rotationDeg),
        },
      });
      boundaryCases.push({
        translationAmplitudeM,
        rotationDeg,
        durationS,
        onset,
        fast: onset.onsetTimeS !== null
          && onset.onsetTimeS >= FAST_ONSET_TARGET.onsetMinS
          && onset.onsetTimeS <= FAST_ONSET_TARGET.onsetMaxS
          && (onset.guardTimeS === null || onset.guardTimeS > onset.onsetTimeS),
      });
    }
  }
}

boundaryCases.sort((a, b) => (
  (a.onset.onsetTimeS ?? Infinity) - (b.onset.onsetTimeS ?? Infinity)
));

console.log("\n### H1-4A movable-boundary pulse sensitivity at baseline parameters");
for (const x of boundaryCases.slice(0, 12)) {
  console.log(JSON.stringify({
    baseTranslationMm: 1000 * x.translationAmplitudeM,
    baseRotationDeg: x.rotationDeg,
    pulseDurationS: x.durationS,
    onsetS: x.onset.onsetTimeS,
    guardS: x.onset.guardTimeS,
    thresholdMm: 1000 * x.onset.onsetThresholdM,
    maxRmsMm: 1000 * x.onset.maxObservedRmsM,
    fast: x.fast,
  }));
}

const parameterFast = parameterTimeDomain.some((x) => x.fast);
const curvatureFast = curvatureCases.some((x) => x.fast);
const boundaryFast = boundaryCases.some((x) => x.fast);

const decision = parameterFast
  ? "A_PARAMETER_ONLY"
  : ((curvatureFast || boundaryFast)
    ? "B_INITIAL_OR_BOUNDARY"
    : "C_LINEAR_INSUFFICIENT");

console.log("\nH1-4A_DECISION", JSON.stringify({
  decision,
  parameterFast,
  curvatureFast,
  boundaryFast,
  targetOnsetS: [
    FAST_ONSET_TARGET.onsetMinS,
    FAST_ONSET_TARGET.onsetMaxS,
  ],
}));

console.log(
  "H1-4A exploratory calibration complete; no real-product identification claimed.",
);
