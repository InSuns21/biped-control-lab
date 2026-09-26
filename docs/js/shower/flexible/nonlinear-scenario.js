import {
  createNonlinearRod,
  rodKinematics,
  solveStaticRodEquilibrium,
  stepNonlinearRodRK4,
} from "./nonlinear-rod.js";
import {
  createConveyingFlowGeneralizedForce,
} from "./nonlinear-flow.js";
import {
  DEFAULT_SHOWER_HEAD_PARAMS,
} from "./shower-head.js";
import {
  hoseAreaFromDiameterM,
  nonlinearShowerHeadTipLoad,
} from "./nonlinear-shower-head.js";

export const DEFAULT_NONLINEAR_SCENARIO_PARAMS = Object.freeze({
  flowLpm: 18,
  waterDensityKgM3: 997,
  hoseInnerDiameterM: 0.006,
  segmentCount: 12,
  lengthM: 1.2,
  flexuralRigidityNm2: 0.7,
  structuralMassPerM: 0.25,
  gravityMps2: 9.81,
  headMassKg: 0.20,
  headRotInertiaKgM2: 0.002,
  headComAxialOffsetM: 0.055,
  rayleighMassPerS: 0.08,
  rayleighStiffnessS: 0.0002,
});

export function createNonlinearShowerScenario(overrides = {}) {
  const params = {
    ...DEFAULT_NONLINEAR_SCENARIO_PARAMS,
    ...overrides,
  };
  const areaM2 = hoseAreaFromDiameterM(params.hoseInnerDiameterM);
  const fluidMassPerM = params.waterDensityKgM3 * areaM2;
  const flowRateM3s = params.flowLpm / 60000;
  const flowSpeedMps = flowRateM3s / areaM2;
  const system = createNonlinearRod({
    segmentCount: params.segmentCount,
    lengthM: params.lengthM,
    flexuralRigidityNm2: params.flexuralRigidityNm2,
    structuralMassPerM: params.structuralMassPerM,
    fluidMassPerM,
    gravityMps2: params.gravityMps2,
    headMassKg: params.headMassKg,
    headRotInertiaKgM2: params.headRotInertiaKgM2,
    headComAxialOffsetM: params.headComAxialOffsetM,
    rayleighMassPerS: params.rayleighMassPerS,
    rayleighStiffnessS: params.rayleighStiffnessS,
  });
  const head = {
    ...DEFAULT_SHOWER_HEAD_PARAMS,
    headMassKg: params.headMassKg,
    headRotInertiaAboutComKgM2: params.headRotInertiaKgM2,
    headComAxialOffsetM: params.headComAxialOffsetM,
    ...(params.head ?? {}),
  };

  const flowForce = createConveyingFlowGeneralizedForce({
    flowSpeedMps,
    fluidMassPerM,
  });
  const tipLoad = ({ state }) => nonlinearShowerHeadTipLoad(
    system,
    state.anglesRad,
    {
      flowRateM3s,
      waterDensityKgM3: params.waterDensityKgM3,
      hoseInnerDiameterM: params.hoseInnerDiameterM,
      head,
    },
  );

  return {
    params,
    system,
    head,
    areaM2,
    fluidMassPerM,
    flowRateM3s,
    flowSpeedMps,
    flowForce,
    tipLoad,
  };
}

export function solveNonlinearShowerEquilibrium(
  scenario,
  {
    initialAnglesRad = null,
    tolerance = 1e-8,
  } = {},
) {
  return solveStaticRodEquilibrium(
    scenario.system,
    {
      tipLoad: scenario.tipLoad,
      additionalGeneralizedForce: scenario.flowForce,
      initialAnglesRad,
      tolerance,
      maxIterations: 80,
    },
  );
}

export function perturbEquilibriumState(
  scenario,
  equilibriumAnglesRad,
  {
    tipAnglePerturbationRad = 0.02,
    velocityAmplitudeRadS = 0.03,
  } = {},
) {
  const count = scenario.system.params.segmentCount;
  const anglesRad = [...equilibriumAnglesRad];
  const angularRatesRadS = Array(count).fill(0);

  for (let i = 1; i < count; i += 1) {
    const x = i / (count - 1);
    const shape = x * x * (3 - 2 * x);
    anglesRad[i] += tipAnglePerturbationRad * shape;
    angularRatesRadS[i] = velocityAmplitudeRadS
      * x * x
      * Math.sin(2 * Math.PI * x);
  }
  anglesRad[0] = scenario.system.params.baseAngleRad;
  angularRatesRadS[0] = 0;

  return { anglesRad, angularRatesRadS };
}

export function geometricRmsFromEquilibrium(
  scenario,
  state,
  equilibriumKinematics,
) {
  const current = rodKinematics(
    scenario.system,
    state.anglesRad,
  );
  let sumSq = 0;
  let count = 0;
  let maxNodeDisplacementM = 0;

  for (let i = 1; i < current.nodes.length; i += 1) {
    const dx = current.nodes[i][0] - equilibriumKinematics.nodes[i][0];
    const dy = current.nodes[i][1] - equilibriumKinematics.nodes[i][1];
    const displacement = Math.hypot(dx, dy);
    sumSq += displacement * displacement;
    maxNodeDisplacementM = Math.max(
      maxNodeDisplacementM,
      displacement,
    );
    count += 1;
  }

  return {
    rmsM: Math.sqrt(sumSq / Math.max(1, count)),
    maxNodeDisplacementM,
    tipDisplacementM: Math.hypot(
      current.tip[0] - equilibriumKinematics.tip[0],
      current.tip[1] - equilibriumKinematics.tip[1],
    ),
    tip: current.tip,
    tipAngleRad: current.tipAngleRad,
  };
}

export function simulateNonlinearShowerOnset(
  scenario,
  equilibrium,
  {
    durationS = 8,
    dt = 0.001,
    onsetFactor = 3,
    absoluteOnsetM = 0.020,
    tipAnglePerturbationRad = 0.02,
    velocityAmplitudeRadS = 0.03,
    maxAbsAngleRad = 3.0,
  } = {},
) {
  let state = perturbEquilibriumState(
    scenario,
    equilibrium.anglesRad,
    {
      tipAnglePerturbationRad,
      velocityAmplitudeRadS,
    },
  );
  const initial = geometricRmsFromEquilibrium(
    scenario,
    state,
    equilibrium.kinematics,
  );
  const thresholdM = Math.max(
    absoluteOnsetM,
    onsetFactor * initial.rmsM,
  );

  let onsetTimeS = null;
  let maxRmsM = initial.rmsM;
  let maxTipDisplacementM = initial.tipDisplacementM;
  let numericalFailure = false;
  let largeRotationTimeS = null;

  const steps = Math.round(durationS / dt);
  for (let step = 0; step < steps; step += 1) {
    state = stepNonlinearRodRK4(
      scenario.system,
      state,
      dt,
      scenario.tipLoad,
      scenario.flowForce,
    );
    const timeS = (step + 1) * dt;

    if (
      !state.anglesRad.every(Number.isFinite)
      || !state.angularRatesRadS.every(Number.isFinite)
    ) {
      numericalFailure = true;
      break;
    }

    const metrics = geometricRmsFromEquilibrium(
      scenario,
      state,
      equilibrium.kinematics,
    );
    maxRmsM = Math.max(maxRmsM, metrics.rmsM);
    maxTipDisplacementM = Math.max(
      maxTipDisplacementM,
      metrics.tipDisplacementM,
    );

    if (
      onsetTimeS === null
      && metrics.rmsM >= thresholdM
    ) {
      onsetTimeS = timeS;
    }

    if (
      largeRotationTimeS === null
      && state.anglesRad.some(
        (angle) => Math.abs(angle) >= maxAbsAngleRad,
      )
    ) {
      largeRotationTimeS = timeS;
    }
  }

  return {
    initialRmsM: initial.rmsM,
    onsetThresholdM: thresholdM,
    onsetTimeS,
    maxRmsM,
    maxTipDisplacementM,
    largeRotationTimeS,
    numericalFailure,
    finalState: state,
  };
}
