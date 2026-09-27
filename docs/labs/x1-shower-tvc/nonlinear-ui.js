import {
  createNonlinearShowerScenario,
  geometricRmsFromEquilibrium,
  perturbEquilibriumState,
  solveNonlinearShowerEquilibrium,
} from "../../js/shower/flexible/nonlinear-scenario.js";
import {
  rodKinematics,
  stepNonlinearRodRK4,
} from "../../js/shower/flexible/nonlinear-rod.js";
import {
  nonlinearShowerHeadReaction,
} from "../../js/shower/flexible/nonlinear-shower-head.js";
import {
  createNonlinearView,
} from "./nonlinear-view.js";

const DT = 0.002;
const HISTORY_SECONDS = 8;
const HISTORY_SAMPLE_DT = 0.025;

const PRESETS = Object.freeze({
  baseline18: Object.freeze({
    label: "現基準 18 L/min",
    note: "現H1-4B既定パラメータ。非線形平衡は有限だが成長は比較的ゆっくり。",
    flowLpm: 18,
    lengthM: 1.2,
    flexuralRigidityNm2: 0.7,
    rayleighMassPerS: 0.08,
    rayleighStiffnessS: 0.0002,
    ciOnset: "6 s以内の20 mm閾値到達なし",
  }),
  fast22: Object.freeze({
    label: "Fast sensitivity 22 L/min",
    note: "教育用感度ケース。実製品同定ではない。CI onset ≈ 1.58 s。",
    flowLpm: 22,
    lengthM: 1.5,
    flexuralRigidityNm2: 0.25,
    rayleighMassPerS: 0.02,
    rayleighStiffnessS: 0.0002,
    ciOnset: "約1.58 s",
  }),
  high30: Object.freeze({
    label: "現基準 high-flow 30 L/min",
    note: "現基準材質のまま流量のみ増加した感度ケース。CI onset ≈ 1.82 s。",
    flowLpm: 30,
    lengthM: 1.2,
    flexuralRigidityNm2: 0.7,
    rayleighMassPerS: 0.08,
    rayleighStiffnessS: 0.0002,
    ciOnset: "約1.82 s",
  }),
});

function radToDeg(rad) {
  return rad * 180 / Math.PI;
}

function solveContinuation(preset, flowLpm) {
  const flowSteps = [
    0,
    flowLpm / 3,
    2 * flowLpm / 3,
    flowLpm,
  ];
  let angles = null;
  let scenario = null;
  let equilibrium = null;

  for (const q of flowSteps) {
    scenario = createNonlinearShowerScenario({
      segmentCount: 12,
      flowLpm: q,
      lengthM: preset.lengthM,
      flexuralRigidityNm2: preset.flexuralRigidityNm2,
      rayleighMassPerS: preset.rayleighMassPerS,
      rayleighStiffnessS: preset.rayleighStiffnessS,
      headMassKg: 0.20,
      headRotInertiaKgM2: 0.002,
    });
    equilibrium = solveNonlinearShowerEquilibrium(
      scenario,
      { initialAnglesRad: angles },
    );
    if (!equilibrium.converged) {
      throw new Error(
        `nonlinear equilibrium did not converge at Q=${q.toFixed(2)} L/min`,
      );
    }
    angles = [...equilibrium.anglesRad];
  }

  return { scenario, equilibrium };
}

export function mountNonlinearPhase(root) {
  const canvas = root.querySelector("#nlCanvas");
  const rmsChart = root.querySelector("#nlRmsChart");
  const tipChart = root.querySelector("#nlTipChart");
  const flow = root.querySelector("#nlFlow");
  const flowOut = root.querySelector("#nlFlowOut");
  const playback = root.querySelector("#nlPlayback");
  const showNodes = root.querySelector("#nlShowNodes");
  const pauseButton = root.querySelector("#nlPause");
  const resetButton = root.querySelector("#nlReset");
  const presetButtons = [...root.querySelectorAll("[data-nl-preset]")];

  const presetMetric = root.querySelector("#nlPresetMetric");
  const flowMetric = root.querySelector("#nlFlowMetric");
  const speedMetric = root.querySelector("#nlSpeedMetric");
  const materialMetric = root.querySelector("#nlMaterialMetric");
  const equilibriumMetric = root.querySelector("#nlEquilibriumMetric");
  const angleMetric = root.querySelector("#nlAngleMetric");
  const rmsMetric = root.querySelector("#nlRmsMetric");
  const onsetMetric = root.querySelector("#nlOnsetMetric");
  const ciMetric = root.querySelector("#nlCiMetric");
  const timeMetric = root.querySelector("#nlTimeMetric");
  const statusMetric = root.querySelector("#nlStatusMetric");
  const presetNote = root.querySelector("#nlPresetNote");

  const view = createNonlinearView({
    canvas,
    rmsChart,
    tipChart,
  });

  let presetId = "fast22";
  let preset = PRESETS[presetId];
  let scenario = null;
  let equilibrium = null;
  let state = null;
  let initialRmsM = 0;
  let onsetThresholdM = 0.020;
  let observedOnsetS = null;
  let history = [];
  let simTime = 0;
  let lastHistoryTime = -Infinity;
  let paused = false;
  let active = false;
  let accumulator = 0;
  let lastFrameMs = performance.now();
  let rafId = null;
  let stoppedReason = null;

  function reactionForState() {
    return nonlinearShowerHeadReaction(
      scenario.system,
      state.anglesRad,
      {
        flowRateM3s: scenario.flowRateM3s,
        waterDensityKgM3: scenario.params.waterDensityKgM3,
        hoseInnerDiameterM: scenario.params.hoseInnerDiameterM,
        head: scenario.head,
      },
    );
  }

  function configure({ preserveFlow = false } = {}) {
    preset = PRESETS[presetId];
    if (!preserveFlow) flow.value = String(preset.flowLpm);
    const flowLpm = Number(flow.value);
    flowOut.value = `${flowLpm.toFixed(1)} L/min`;
    presetNote.textContent = preset.note;

    const solved = solveContinuation(preset, flowLpm);
    scenario = solved.scenario;
    equilibrium = solved.equilibrium;
    state = perturbEquilibriumState(
      scenario,
      equilibrium.anglesRad,
      {
        tipAnglePerturbationRad: 0.02,
        velocityAmplitudeRadS: 0.03,
      },
    );

    const initial = geometricRmsFromEquilibrium(
      scenario,
      state,
      equilibrium.kinematics,
    );
    initialRmsM = initial.rmsM;
    onsetThresholdM = Math.max(0.020, 3 * initialRmsM);
    observedOnsetS = null;
    history = [];
    simTime = 0;
    lastHistoryTime = -Infinity;
    accumulator = 0;
    stoppedReason = null;
    paused = false;
    pauseButton.textContent = "一時停止";
    lastFrameMs = performance.now();
    recordHistory();
    render();
  }

  function dynamicMetrics() {
    return geometricRmsFromEquilibrium(
      scenario,
      state,
      equilibrium.kinematics,
    );
  }

  function recordHistory() {
    if (simTime - lastHistoryTime < HISTORY_SAMPLE_DT - 1e-9) return;
    lastHistoryTime = simTime;
    const metrics = dynamicMetrics();
    history.push({
      t: simTime,
      rmsMm: metrics.rmsM * 1000,
      tipMm: metrics.tipDisplacementM * 1000,
    });
    const cutoff = simTime - HISTORY_SECONDS;
    while (history.length > 2 && history[0].t < cutoff) history.shift();
  }

  function updateMetrics(metrics, current) {
    presetMetric.textContent = preset.label;
    flowMetric.textContent = `${scenario.params.flowLpm.toFixed(1)} L/min`;
    speedMetric.textContent = `${scenario.flowSpeedMps.toFixed(2)} m/s`;
    materialMetric.textContent =
      `EI ${scenario.params.flexuralRigidityNm2.toFixed(2)} N·m² / L ${scenario.params.lengthM.toFixed(2)} m`;
    equilibriumMetric.textContent =
      `x ${equilibrium.kinematics.tip[0].toFixed(3)} m / y ${equilibrium.kinematics.tip[1].toFixed(3)} m`;
    angleMetric.textContent =
      `${radToDeg(current.tipAngleRad).toFixed(1)}°`;
    rmsMetric.textContent = `${(metrics.rmsM * 1000).toFixed(1)} mm`;
    onsetMetric.textContent = observedOnsetS === null
      ? `未到達 / threshold ${(onsetThresholdM * 1000).toFixed(1)} mm`
      : `${observedOnsetS.toFixed(3)} s`;
    ciMetric.textContent = preset.ciOnset;
    timeMetric.textContent = `${simTime.toFixed(2)} s`;
    statusMetric.textContent = stoppedReason
      ? stoppedReason
      : (paused ? "一時停止" : "実行中");
    statusMetric.className = stoppedReason
      ? "status-danger"
      : "status-ok";
  }

  function render() {
    if (!scenario || !equilibrium || !state) return;
    const current = rodKinematics(
      scenario.system,
      state.anglesRad,
    );
    const metrics = dynamicMetrics();
    const reaction = reactionForState();

    view.render({
      currentKinematics: current,
      equilibriumKinematics: equilibrium.kinematics,
      reaction,
      showNodes: showNodes.checked,
      stoppedReason,
    });
    view.renderHistory(history);
    updateMetrics(metrics, current);
  }

  function step() {
    state = stepNonlinearRodRK4(
      scenario.system,
      state,
      DT,
      scenario.tipLoad,
      scenario.flowForce,
    );
    simTime += DT;

    if (
      !state.anglesRad.every(Number.isFinite)
      || !state.angularRatesRadS.every(Number.isFinite)
    ) {
      stoppedReason = "数値異常のため停止";
      paused = true;
      return;
    }

    const metrics = dynamicMetrics();
    if (
      observedOnsetS === null
      && metrics.rmsM >= onsetThresholdM
    ) {
      observedOnsetS = simTime;
    }

    const maxAngle = Math.max(
      ...state.anglesRad.map((angle) => Math.abs(angle)),
    );
    if (maxAngle > 3.0 || metrics.rmsM > 0.60) {
      stoppedReason = "H1-4B 2Dモデルの監査上限に到達";
      paused = true;
    }
  }

  function frame(now) {
    rafId = null;
    if (!active) return;

    const elapsed = Math.min((now - lastFrameMs) / 1000, 0.05);
    lastFrameMs = now;

    if (!paused && !stoppedReason) {
      accumulator += elapsed * Number(playback.value);
      let steps = 0;
      while (accumulator >= DT && steps < 40) {
        step();
        accumulator -= DT;
        steps += 1;
        if (stoppedReason) break;
      }
      if (steps >= 40) accumulator = 0;
      recordHistory();
    }

    render();
    rafId = requestAnimationFrame(frame);
  }

  presetButtons.forEach((button) => {
    button.addEventListener("click", () => {
      presetId = button.dataset.nlPreset;
      configure({ preserveFlow: false });
    });
  });

  flow.addEventListener("input", () => {
    flowOut.value = `${Number(flow.value).toFixed(1)} L/min`;
  });
  flow.addEventListener("change", () => {
    configure({ preserveFlow: true });
  });

  playback.addEventListener("change", () => {
    lastFrameMs = performance.now();
  });
  showNodes.addEventListener("change", render);

  pauseButton.addEventListener("click", () => {
    if (stoppedReason) return;
    paused = !paused;
    pauseButton.textContent = paused ? "再開" : "一時停止";
    lastFrameMs = performance.now();
  });

  resetButton.addEventListener("click", () => {
    configure({ preserveFlow: true });
  });

  configure({ preserveFlow: false });

  return {
    setActive(next) {
      active = next;
      if (active && rafId === null) {
        lastFrameMs = performance.now();
        rafId = requestAnimationFrame(frame);
      } else if (!active && rafId !== null) {
        cancelAnimationFrame(rafId);
        rafId = null;
      }
    },
  };
}
