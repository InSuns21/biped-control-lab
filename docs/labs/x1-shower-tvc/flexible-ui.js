import {
  assembleShowerHeadConveyingBeam,
  showerHeadMomentumReaction2D,
} from "../../js/shower/flexible/shower-head.js";
import {
  flowSpeedFromRateM3s,
} from "../../js/shower/flexible/conveying-flow.js";
import {
  createNewmarkGeneralLinear,
} from "../../js/shower/flexible/integrator.js";
import {
  solveLinear,
} from "../../js/shower/flexible/linear-algebra.js";
import {
  H1_3_REFERENCE,
  classifyReferenceFlow,
  referenceInitialPerturbationReduced,
} from "../../js/shower/flexible/scenarios.js";
import {
  FAST_ONSET_TARGET,
  equilibriumValidity,
} from "../../js/shower/flexible/calibration.js";
import { createFlexibleView } from "./flexible-view.js";

const FIXED_DT = 0.002;
const HISTORY_SECONDS = 12;
const HISTORY_SAMPLE_DT = 0.05;
const lpmToM3s = (lpm) => lpm / 60000;
const radToDeg = (rad) => rad * 180 / Math.PI;

function rms(values) {
  if (values.length === 0) return 0;
  return Math.sqrt(
    values.reduce((sum, value) => sum + value * value, 0) / values.length,
  );
}

function vectorSubtract(a, b) {
  return a.map((value, i) => value - b[i]);
}

function dynamicNodeDisplacements(system, state, equilibrium) {
  const values = [0];
  for (let node = 1; node < system.nodeCount; node += 1) {
    const index = 2 * (node - 1);
    values.push(state.q[index] - equilibrium[index]);
  }
  return values;
}

function estimateShapeMode(system, state, equilibrium) {
  const values = dynamicNodeDisplacements(system, state, equilibrium);
  const maxAbs = Math.max(...values.map(Math.abs));
  if (maxAbs < 1e-6) return 1;

  const threshold = 0.04 * maxAbs;
  let lastSign = 0;
  let signChanges = 0;
  for (const value of values) {
    if (Math.abs(value) < threshold) continue;
    const sign = Math.sign(value);
    if (lastSign !== 0 && sign !== lastSign) signChanges += 1;
    lastSign = sign;
  }
  return Math.min(4, signChanges + 1);
}

function estimateGrowthRate(history) {
  const windowSamples = Math.round(1.5 / HISTORY_SAMPLE_DT);
  if (history.length < 2 * windowSamples + 2) return null;

  const recent = history.slice(-windowSamples);
  const previous = history.slice(-2 * windowSamples, -windowSamples);
  const recentMean = recent.reduce((sum, x) => sum + x.rmsMm, 0)
    / recent.length;
  const previousMean = previous.reduce((sum, x) => sum + x.rmsMm, 0)
    / previous.length;

  if (recentMean < 1e-5 || previousMean < 1e-5) return null;

  const recentTime = recent.reduce((sum, x) => sum + x.t, 0) / recent.length;
  const previousTime = previous.reduce((sum, x) => sum + x.t, 0)
    / previous.length;
  const deltaTime = recentTime - previousTime;
  if (!(deltaTime > 0)) return null;

  return Math.log(recentMean / previousMean) / deltaTime;
}

function flowStatusClass(regimeId) {
  if (regimeId === "flutter") return "status-danger";
  if (regimeId === "stable" || regimeId === "stationary") return "status-ok";
  return "";
}

export function mountFlexiblePhase(root) {
  const hoseCanvas = root.querySelector("#p1FlexibleCanvas");
  const tipChart = root.querySelector("#p1TipChart");
  const rmsChart = root.querySelector("#p1RmsChart");
  const flow = root.querySelector("#p1Flow");
  const flowOut = root.querySelector("#p1FlowOut");
  const deformationScale = root.querySelector("#p1DeformationScale");
  const playbackRate = root.querySelector("#p1PlaybackRate");
  const showNodes = root.querySelector("#p1ShowNodes");
  const pauseButton = root.querySelector("#p1Pause");
  const resetButton = root.querySelector("#p1Reset");
  const presetButtons = [...root.querySelectorAll("[data-flow-lpm]")];

  const regimeMetric = root.querySelector("#p1RegimeMetric");
  const flowMetric = root.querySelector("#p1FlowMetric");
  const speedMetric = root.querySelector("#p1SpeedMetric");
  const tipMetric = root.querySelector("#p1TipMetric");
  const angleMetric = root.querySelector("#p1AngleMetric");
  const rmsMetric = root.querySelector("#p1RmsMetric");
  const growthMetric = root.querySelector("#p1GrowthMetric");
  const modeMetric = root.querySelector("#p1ModeMetric");
  const timeMetric = root.querySelector("#p1TimeMetric");
  const validityMetric = root.querySelector("#p1ValidityMetric");
  const viewFitStatus = root.querySelector("#p1ViewFitStatus");

  const view = createFlexibleView({
    hoseCanvas,
    tipChart,
    rmsChart,
  });

  let system = null;
  let integrator = null;
  let equilibrium = null;
  let equilibriumCheck = null;
  let state = null;
  let headReaction = null;
  let history = [];
  let simTime = 0;
  let lastHistoryTime = -Infinity;
  let accumulator = 0;
  let lastTime = performance.now();
  let paused = false;
  let active = false;
  let rafId = null;
  let limitExceeded = false;

  function currentFlowLpm() {
    return Number(flow.value);
  }

  function updateFlowLabel() {
    const value = currentFlowLpm();
    flowOut.value = `${value.toFixed(1)} L/min`;
  }

  function buildSystem({ preserveState = false } = {}) {
    const flowLpm = currentFlowLpm();
    const flowSpeedMps = flowSpeedFromRateM3s(
      lpmToM3s(flowLpm),
      H1_3_REFERENCE.innerDiameterM,
    );

    const nextSystem = assembleShowerHeadConveyingBeam({
      flowSpeedMps,
      elementCount: H1_3_REFERENCE.elementCount,
      hoseInnerDiameterM: H1_3_REFERENCE.innerDiameterM,
    });
    const nextIntegrator = createNewmarkGeneralLinear(
      nextSystem.reduced,
      FIXED_DT,
    );
    const nextEquilibrium = solveLinear(
      nextSystem.reduced.stiffness,
      nextSystem.reduced.headForce0,
    );

    let q;
    let v;
    if (preserveState && state && state.q.length === nextEquilibrium.length) {
      q = [...state.q];
      v = [...state.v];
    } else {
      const perturbation = referenceInitialPerturbationReduced(nextSystem);
      q = nextEquilibrium.map(
        (value, i) => value + perturbation.q[i],
      );
      v = perturbation.v;
    }

    system = nextSystem;
    integrator = nextIntegrator;
    equilibrium = nextEquilibrium;
    equilibriumCheck = equilibriumValidity(
      nextSystem,
      nextEquilibrium,
      FAST_ONSET_TARGET,
    );
    state = integrator.initialize({
      q,
      v,
      force: system.reduced.headForce0,
    });
    headReaction = showerHeadMomentumReaction2D({
      flowSpeedMps: system.params.flowSpeedMps,
      fluidDensityKgM3: system.params.waterDensityKgM3,
      hoseAreaM2: system.flowAreaM2,
      tipAngleRad: state.q.at(-1),
      head: system.head,
    });

    if (!preserveState) {
      history = [];
      simTime = 0;
      lastHistoryTime = -Infinity;
      limitExceeded = false;
      accumulator = 0;
    }
  }

  function resetSimulation() {
    buildSystem({ preserveState: false });
    paused = false;
    pauseButton.textContent = "一時停止";
    lastTime = performance.now();
    render();
  }

  function checkLinearValidity() {
    const dynamicQ = vectorSubtract(state.q, equilibrium);
    let maxDisplacement = 0;
    let maxRotation = 0;
    for (let node = 1; node < system.nodeCount; node += 1) {
      const j = 2 * (node - 1);
      maxDisplacement = Math.max(maxDisplacement, Math.abs(dynamicQ[j]));
      maxRotation = Math.max(maxRotation, Math.abs(dynamicQ[j + 1]));
    }
    return maxDisplacement <= FAST_ONSET_TARGET.maxDynamicDisplacementM
      && maxRotation <= FAST_ONSET_TARGET.maxDynamicRotationRad;
  }

  function recordHistory() {
    if (simTime - lastHistoryTime < HISTORY_SAMPLE_DT - 1e-9) return;
    lastHistoryTime = simTime;

    const dynamicQ = vectorSubtract(state.q, equilibrium);
    const nodeDisplacements = dynamicNodeDisplacements(
      system,
      state,
      equilibrium,
    );
    const dynamicTipM = dynamicQ.at(-2);
    const rmsM = rms(nodeDisplacements);

    history.push({
      t: simTime,
      tipMm: dynamicTipM * 1000,
      rmsMm: rmsM * 1000,
    });
    const cutoff = simTime - HISTORY_SECONDS;
    while (history.length > 2 && history[0].t < cutoff) history.shift();
  }

  function updateMetrics() {
    const flowLpm = currentFlowLpm();
    const regime = classifyReferenceFlow(flowLpm);
    const dynamicQ = vectorSubtract(state.q, equilibrium);
    const nodeDisplacements = dynamicNodeDisplacements(
      system,
      state,
      equilibrium,
    );
    const growthRate = estimateGrowthRate(history);
    const mode = estimateShapeMode(system, state, equilibrium);

    regimeMetric.textContent = `${regime.label} — ${regime.detail}`;
    regimeMetric.className = flowStatusClass(regime.id);
    flowMetric.textContent = `${flowLpm.toFixed(1)} / ${H1_3_REFERENCE.criticalFlowLpm.toFixed(2)} L/min`;
    speedMetric.textContent = `${system.params.flowSpeedMps.toFixed(3)} m/s`;
    tipMetric.textContent = `${(state.q.at(-2) * 1000).toFixed(1)} mm`;
    angleMetric.textContent = `${radToDeg(state.q.at(-1)).toFixed(2)}°`;
    rmsMetric.textContent = `${(rms(nodeDisplacements) * 1000).toFixed(2)} mm`;
    growthMetric.textContent = growthRate === null
      ? "測定中"
      : `${growthRate >= 0 ? "+" : ""}${growthRate.toFixed(3)} /s`;
    growthMetric.className = growthRate === null
      ? ""
      : (growthRate > 0.02 ? "status-danger" : "status-ok");
    modeMetric.textContent = `形状モード ≈ ${mode}`;
    timeMetric.textContent = `${simTime.toFixed(1)} s`;
    if (!equilibriumCheck.valid) {
      validityMetric.textContent = "静的平衡が線形範囲外・参考表示";
      validityMetric.className = "status-danger";
    } else if (limitExceeded) {
      validityMetric.textContent = "動的線形範囲超過・停止";
      validityMetric.className = "status-danger";
    } else {
      validityMetric.textContent = "small-deflection 範囲内";
      validityMetric.className = "status-ok";
    }

    if (!dynamicQ.every(Number.isFinite)) {
      validityMetric.textContent = "数値異常・停止";
      validityMetric.className = "status-danger";
    }
  }

  function render() {
    headReaction = showerHeadMomentumReaction2D({
      flowSpeedMps: system.params.flowSpeedMps,
      fluidDensityKgM3: system.params.waterDensityKgM3,
      hoseAreaM2: system.flowAreaM2,
      tipAngleRad: state.q.at(-1),
      head: system.head,
    });

    const viewState = view.renderHose({
      system,
      state,
      equilibrium,
      deformationScale: Number(deformationScale.value),
      showNodes: showNodes.checked,
      headReaction,
      flowLpm: currentFlowLpm(),
      criticalFlowLpm: H1_3_REFERENCE.criticalFlowLpm,
      limitExceeded,
    });
    viewFitStatus.textContent = viewState.autoFitActive
      ? `横表示: 指定 ×${viewState.requestedScale.toFixed(0)} → 自動fit ×${viewState.effectiveScale.toFixed(2)}`
      : `横表示: ×${viewState.effectiveScale.toFixed(2)}（縦横同一縮尺）`;
    viewFitStatus.className = viewState.autoFitActive
      ? "view-fit-status status-warn"
      : "view-fit-status";
    view.renderCharts(history);
    updateMetrics();
  }

  function stepPhysics() {
    state = integrator.step(state, system.reduced.headForce0);
    simTime += FIXED_DT;

    if (!state.q.every(Number.isFinite) || !state.v.every(Number.isFinite)) {
      limitExceeded = true;
      paused = true;
      pauseButton.textContent = "再開不可・Reset";
      return;
    }

    if (!checkLinearValidity()) {
      limitExceeded = true;
      paused = true;
      pauseButton.textContent = "再開不可・Reset";
    }
  }

  function frame(now) {
    rafId = null;
    if (!active) return;

    const elapsed = Math.min((now - lastTime) / 1000, 0.05);
    lastTime = now;

    if (!paused && !limitExceeded) {
      accumulator += elapsed * Number(playbackRate.value);
      while (accumulator >= FIXED_DT) {
        stepPhysics();
        accumulator -= FIXED_DT;
        if (limitExceeded) break;
      }
      recordHistory();
    }

    render();
    rafId = requestAnimationFrame(frame);
  }

  flow.addEventListener("input", updateFlowLabel);
  flow.addEventListener("change", () => {
    buildSystem({ preserveState: false });
    render();
  });

  presetButtons.forEach((button) => {
    button.addEventListener("click", () => {
      flow.value = button.dataset.flowLpm;
      updateFlowLabel();
      resetSimulation();
    });
  });

  deformationScale.addEventListener("change", render);
  playbackRate.addEventListener("change", () => {
    lastTime = performance.now();
  });
  showNodes.addEventListener("change", render);

  pauseButton.addEventListener("click", () => {
    if (limitExceeded) return;
    paused = !paused;
    pauseButton.textContent = paused ? "再開" : "一時停止";
    lastTime = performance.now();
  });

  resetButton.addEventListener("click", resetSimulation);

  updateFlowLabel();
  buildSystem({ preserveState: false });
  recordHistory();
  render();

  return {
    setActive(next) {
      active = next;
      if (active && rafId === null) {
        lastTime = performance.now();
        rafId = requestAnimationFrame(frame);
      } else if (!active && rafId !== null) {
        cancelAnimationFrame(rafId);
        rafId = null;
      }
    },
  };
}
