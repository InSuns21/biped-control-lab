import {
  createNonlinearShowerScenario,
  perturbEquilibriumState,
  solveNonlinearShowerEquilibrium,
} from "../../js/shower/flexible/nonlinear-scenario.js";
import {
  nonlinearShowerHeadReaction,
} from "../../js/shower/flexible/nonlinear-shower-head.js";
import {
  handBoundaryDynamics,
  rodKinematicsWithHandBoundary,
  smoothHandPulse,
  stepRodWithHandBoundaryRK4,
  sumHandBoundaries,
  ZERO_HAND_BOUNDARY,
} from "../../js/shower/flexible/nonlinear-boundary.js";
import {
  createNonlinearView,
} from "./nonlinear-view.js";

const DT = 0.002;
const HISTORY_SECONDS = 8;
const HISTORY_SAMPLE_DT = 0.025;

const PRESETS = Object.freeze({
  baseline18: Object.freeze({
    label: "現基準 18 L/min",
    note: "非線形平衡は有限回転で成立。成長は比較的ゆっくり。",
    flowLpm: 18,
    lengthM: 1.2,
    flexuralRigidityNm2: 0.7,
    rayleighMassPerS: 0.08,
    rayleighStiffnessS: 0.0002,
    ciOnset: "6 s以内の20 mm閾値到達なし",
  }),
  fast22: Object.freeze({
    label: "Fast sensitivity 22 L/min",
    note: "CI onset ≈ 1.58 s。手元pulseで境界仕事との関係も確認できます。",
    flowLpm: 22,
    lengthM: 1.5,
    flexuralRigidityNm2: 0.25,
    rayleighMassPerS: 0.02,
    rayleighStiffnessS: 0.0002,
    ciOnset: "約1.58 s",
  }),
  high30: Object.freeze({
    label: "現基準 high-flow 30 L/min",
    note: "現基準材質のまま流量を増加。CI onset ≈ 1.82 s。",
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

function geometryMetrics(current, equilibrium) {
  let sumSq = 0;
  let maxNodeDisplacementM = 0;
  for (let i = 1; i < current.nodes.length; i += 1) {
    const dx = current.nodes[i][0] - equilibrium.nodes[i][0];
    const dy = current.nodes[i][1] - equilibrium.nodes[i][1];
    const displacement = Math.hypot(dx, dy);
    sumSq += displacement * displacement;
    maxNodeDisplacementM = Math.max(
      maxNodeDisplacementM,
      displacement,
    );
  }
  const tipDx = current.tip[0] - equilibrium.tip[0];
  const tipDy = current.tip[1] - equilibrium.tip[1];
  return {
    rmsM: Math.sqrt(
      sumSq / Math.max(1, current.nodes.length - 1),
    ),
    maxNodeDisplacementM,
    tipDisplacementM: Math.hypot(tipDx, tipDy),
  };
}

export function mountNonlinearPhase(root) {
  const canvas = root.querySelector("#nlCanvas");
  const rmsChart = root.querySelector("#nlRmsChart");
  const tipChart = root.querySelector("#nlTipChart");
  const flow = root.querySelector("#nlFlow");
  const flowOut = root.querySelector("#nlFlowOut");
  const playback = root.querySelector("#nlPlayback");
  const pulseDuration = root.querySelector("#nlPulseDuration");
  const showNodes = root.querySelector("#nlShowNodes");
  const pauseButton = root.querySelector("#nlPause");
  const resetButton = root.querySelector("#nlReset");
  const presetButtons = [...root.querySelectorAll("[data-nl-preset]")];
  const handPulseButtons = [
    ...root.querySelectorAll("[data-hand-pulse]"),
  ];

  const presetMetric = root.querySelector("#nlPresetMetric");
  const flowMetric = root.querySelector("#nlFlowMetric");
  const speedMetric = root.querySelector("#nlSpeedMetric");
  const materialMetric = root.querySelector("#nlMaterialMetric");
  const equilibriumMetric = root.querySelector("#nlEquilibriumMetric");
  const angleMetric = root.querySelector("#nlAngleMetric");
  const rmsMetric = root.querySelector("#nlRmsMetric");
  const onsetMetric = root.querySelector("#nlOnsetMetric");
  const ciMetric = root.querySelector("#nlCiMetric");
  const handMetric = root.querySelector("#nlHandMetric");
  const handReactionMetric = root.querySelector(
    "#nlHandReactionMetric",
  );
  const handPowerMetric = root.querySelector("#nlHandPowerMetric");
  const handWorkMetric = root.querySelector("#nlHandWorkMetric");
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
  let currentBoundary = { ...ZERO_HAND_BOUNDARY };
  let lastBoundaryDiagnostics = null;
  let activePulses = [];
  let cumulativeHandWorkJ = 0;
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

  function loadOptions() {
    return {
      tipLoad: scenario.tipLoad,
      additionalGeneralizedForce: scenario.flowForce,
      additionalCartesianResultant: scenario.flowResultant,
    };
  }

  function boundaryAtTime(timeS) {
    const values = activePulses.map((pulse) => smoothHandPulse(
      timeS,
      pulse,
    ));
    return values.length > 0
      ? sumHandBoundaries(values)
      : { ...ZERO_HAND_BOUNDARY };
  }

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

  function currentGeometry() {
    return rodKinematicsWithHandBoundary(
      scenario.system,
      state,
      currentBoundary,
    );
  }

  function currentMetrics(current = currentGeometry()) {
    return geometryMetrics(
      current,
      equilibrium.kinematics,
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

    currentBoundary = { ...ZERO_HAND_BOUNDARY };
    activePulses = [];
    cumulativeHandWorkJ = 0;
    lastBoundaryDiagnostics = handBoundaryDynamics(
      scenario.system,
      state,
      currentBoundary,
      loadOptions(),
    );

    const initial = currentMetrics();
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

  function recordHistory() {
    if (simTime - lastHistoryTime < HISTORY_SAMPLE_DT - 1e-9) return;
    lastHistoryTime = simTime;
    const metrics = currentMetrics();
    history.push({
      t: simTime,
      rmsMm: metrics.rmsM * 1000,
      tipMm: metrics.tipDisplacementM * 1000,
    });
    const cutoff = simTime - HISTORY_SECONDS;
    while (history.length > 2 && history[0].t < cutoff) {
      history.shift();
    }
  }

  function updateMetrics(metrics, current) {
    presetMetric.textContent = preset.label;
    flowMetric.textContent =
      `${scenario.params.flowLpm.toFixed(1)} L/min`;
    speedMetric.textContent =
      `${scenario.flowSpeedMps.toFixed(2)} m/s`;
    materialMetric.textContent =
      `EI ${scenario.params.flexuralRigidityNm2.toFixed(2)} N·m² / L ${scenario.params.lengthM.toFixed(2)} m`;
    equilibriumMetric.textContent =
      `x ${equilibrium.kinematics.tip[0].toFixed(3)} m / y ${equilibrium.kinematics.tip[1].toFixed(3)} m`;
    angleMetric.textContent =
      `${radToDeg(current.tipAngleRad).toFixed(1)}°`;
    rmsMetric.textContent =
      `${(metrics.rmsM * 1000).toFixed(1)} mm`;
    onsetMetric.textContent = observedOnsetS === null
      ? `未到達 / threshold ${(onsetThresholdM * 1000).toFixed(1)} mm`
      : `${observedOnsetS.toFixed(3)} s`;
    ciMetric.textContent = preset.ciOnset;

    handMetric.textContent =
      `x ${(1000 * currentBoundary.lateralPositionM).toFixed(1)} mm / θ ${radToDeg(currentBoundary.angleRad).toFixed(1)}°`;
    handReactionMetric.textContent =
      `F ${lastBoundaryDiagnostics.reactionForceXN.toFixed(2)} N / M ${lastBoundaryDiagnostics.reactionMomentNm.toFixed(3)} N·m`;
    handPowerMetric.textContent =
      `${lastBoundaryDiagnostics.handPowerW >= 0 ? "+" : ""}${lastBoundaryDiagnostics.handPowerW.toFixed(3)} W`;
    handWorkMetric.textContent =
      `${cumulativeHandWorkJ >= 0 ? "+" : ""}${cumulativeHandWorkJ.toFixed(4)} J`;

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
    const current = currentGeometry();
    const metrics = currentMetrics(current);
    const reaction = reactionForState();

    view.render({
      currentKinematics: current,
      equilibriumKinematics: equilibrium.kinematics,
      reaction,
      handBoundary: currentBoundary,
      handReaction: lastBoundaryDiagnostics,
      showNodes: showNodes.checked,
      stoppedReason,
    });
    view.renderHistory(history);
    updateMetrics(metrics, current);
  }

  function step() {
    const next = stepRodWithHandBoundaryRK4(
      scenario.system,
      state,
      simTime,
      DT,
      boundaryAtTime,
      loadOptions(),
    );

    cumulativeHandWorkJ += 0.5
      * (
        lastBoundaryDiagnostics.handPowerW
        + next.diagnostics.handPowerW
      )
      * DT;

    state = next.state;
    currentBoundary = next.boundary;
    lastBoundaryDiagnostics = next.diagnostics;
    simTime += DT;

    if (
      !state.anglesRad.every(Number.isFinite)
      || !state.angularRatesRadS.every(Number.isFinite)
      || !Number.isFinite(cumulativeHandWorkJ)
    ) {
      stoppedReason = "数値異常のため停止";
      paused = true;
      return;
    }

    const metrics = currentMetrics();
    if (
      observedOnsetS === null
      && metrics.rmsM >= onsetThresholdM
    ) {
      observedOnsetS = simTime;
    }

    const maxAngle = Math.max(
      ...state.anglesRad.map((angle) => Math.abs(angle)),
    );
    if (maxAngle > 3.0 || metrics.rmsM > 0.75) {
      stoppedReason = "H1-5-0 2Dモデルの監査上限に到達";
      paused = true;
    }
  }

  function triggerPulse({
    lateralAmplitudeM = 0,
    angleAmplitudeRad = 0,
  }) {
    if (stoppedReason) return;
    activePulses.push({
      startTimeS: simTime,
      durationS: Number(pulseDuration.value),
      lateralAmplitudeM,
      angleAmplitudeRad,
    });
    // Keep only active/recent entries.
    activePulses = activePulses.filter(
      (pulse) => simTime <= pulse.startTimeS + pulse.durationS,
    );
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

  handPulseButtons.forEach((button) => {
    button.addEventListener("click", () => {
      const action = button.dataset.handPulse;
      if (action === "left") {
        triggerPulse({ lateralAmplitudeM: -0.012 });
      } else if (action === "right") {
        triggerPulse({ lateralAmplitudeM: 0.012 });
      } else if (action === "ccw") {
        triggerPulse({ angleAmplitudeRad: -5 * Math.PI / 180 });
      } else if (action === "cw") {
        triggerPulse({ angleAmplitudeRad: 5 * Math.PI / 180 });
      } else if (action === "combo-left") {
        triggerPulse({
          lateralAmplitudeM: -0.010,
          angleAmplitudeRad: -5 * Math.PI / 180,
        });
      } else if (action === "combo-right") {
        triggerPulse({
          lateralAmplitudeM: 0.010,
          angleAmplitudeRad: 5 * Math.PI / 180,
        });
      }
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
