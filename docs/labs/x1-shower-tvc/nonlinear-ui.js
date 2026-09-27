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
  actuatorBoundaryTrajectory,
  clampHandTarget,
  createHandActuatorState,
  DEFAULT_HAND_ACTUATOR_LIMITS,
  handActuatorLimitsLabel,
  pointerDeltaToHandTarget,
  scaleHandActuatorLimits,
  stepHandActuator,
} from "../../js/shower/flexible/hand-actuator.js";
import {
  createAimTarget,
  createDifficultyGameState,
  difficultyAimOffsetM,
  evaluateWaterAim,
  gameDifficultyById,
  gameHudSnapshot,
  updateGameState,
} from "../../js/shower/flexible/game.js";
import {
  controllerHandTarget,
  senseTipFeedback,
} from "../../js/shower/flexible/feedback-controller.js";
import {
  designFullStateLqr,
  lqrHandTarget,
} from "../../js/shower/flexible/state-feedback-controller.js";
import {
  automaticGameHandTarget,
  FIXED_STATE_FEEDBACK_GAIN_SCALE,
} from "../../js/shower/flexible/game-feedback-controller.js";
import {
  createGame3DView,
} from "./game-3d-view.js";
import {
  createNonlinearView,
} from "./nonlinear-view.js";

const DT = 0.002;
const HISTORY_SECONDS = 8;
const HISTORY_SAMPLE_DT = 0.025;

const PRESETS = Object.freeze({
  low12: Object.freeze({
    label: "Low Flow 12 L/min",
    note: "H1-5-2 low-flow stage。",
    flowLpm: 12,
    lengthM: 1.2,
    flexuralRigidityNm2: 0.7,
    rayleighMassPerS: 0.08,
    rayleighStiffnessS: 0.0002,
    ciOnset: "安定側",
  }),
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
    note: "CI onset ≈ 1.58 s。Pointer操作で手元境界から抑制・励起できます。",
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

function anySaturation(saturation) {
  return Object.values(saturation ?? {}).some(Boolean);
}

export function mountNonlinearPhase(root) {
  const canvas = root.querySelector("#nlCanvas");
  const game3dCanvas = root.querySelector("#nl3dCanvas");
  const game3dViewTab = root.querySelector("#nl3dViewTab");
  const debug2dViewTab = root.querySelector("#nl2dViewTab");
  const game3dViewPanel = root.querySelector("#nl3dViewPanel");
  const debug2dViewPanel = root.querySelector("#nl2dViewPanel");
  const game3dCamera = root.querySelector("#nl3dCamera");
  const rmsChart = root.querySelector("#nlRmsChart");
  const tipChart = root.querySelector("#nlTipChart");
  const handXChart = root.querySelector("#nlHandXChart");
  const handAngleChart = root.querySelector("#nlHandAngleChart");
  const flow = root.querySelector("#nlFlow");
  const flowOut = root.querySelector("#nlFlowOut");
  const playback = root.querySelector("#nlPlayback");
  const pulseDuration = root.querySelector("#nlPulseDuration");
  const showNodes = root.querySelector("#nlShowNodes");
  const pauseButton = root.querySelector("#nlPause");
  const resetButton = root.querySelector("#nlReset");
  const centerHandButton = root.querySelector("#nlCenterHand");
  const controlModeSelect = root.querySelector("#nlControlMode");
  const presetButtons = [...root.querySelectorAll("[data-nl-preset]")];
  const handPulseButtons = [
    ...root.querySelectorAll("[data-hand-pulse]"),
  ];
  const gameDifficultyButtons = [
    ...root.querySelectorAll("[data-game-difficulty]"),
  ];
  const gameRestartButton = root.querySelector("#nlGameRestart");
  const resultRestartButton = root.querySelector("#nlResultRestart");
  const gameOverlay = root.querySelector("#nlGameOverlay");
  const gameOverlayKicker = root.querySelector("#nlGameOverlayKicker");
  const gameOverlayTitle = root.querySelector("#nlGameOverlayTitle");
  const gameOverlayBody = root.querySelector("#nlGameOverlayBody");
  const gameOverlayScore = root.querySelector("#nlGameOverlayScore");

  const gameStageMetric = root.querySelector("#nlGameStage");
  const gameStatusMetric = root.querySelector("#nlGameStatus");
  const gameTimeMetric = root.querySelector("#nlGameTime");
  const gameScoreMetric = root.querySelector("#nlGameScore");
  const gameInsideMetric = root.querySelector("#nlGameInside");
  const gameHitMetric = root.querySelector("#nlGameHit");
  const gameEffortMetric = root.querySelector("#nlGameEffort");
  const gameTargetsMetric = root.querySelector("#nlGameTargets");
  const comparisonDifficultyMetric = root.querySelector(
    "#nlComparisonDifficulty",
  );
  const comparisonBody = root.querySelector("#nlComparisonBody");
  const comparisonResetButton = root.querySelector(
    "#nlComparisonReset",
  );

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
  const handTargetMetric = root.querySelector("#nlHandTargetMetric");
  const handReactionMetric = root.querySelector(
    "#nlHandReactionMetric",
  );
  const handPowerMetric = root.querySelector("#nlHandPowerMetric");
  const handWorkMetric = root.querySelector("#nlHandWorkMetric");
  const actuatorMetric = root.querySelector("#nlActuatorMetric");
  const pointerMetric = root.querySelector("#nlPointerMetric");
  const controlSenseMetric = root.querySelector(
    "#nlControlSenseMetric",
  );
  const controlCommandMetric = root.querySelector(
    "#nlControlCommandMetric",
  );
  const lqrStateMetric = root.querySelector("#nlLqrStateMetric");
  const lqrDesignMetric = root.querySelector("#nlLqrDesignMetric");
  const timeMetric = root.querySelector("#nlTimeMetric");
  const statusMetric = root.querySelector("#nlStatusMetric");
  const presetNote = root.querySelector("#nlPresetNote");

  const view = createNonlinearView({
    canvas,
    rmsChart,
    tipChart,
    handXChart,
    handAngleChart,
  });
  const game3dView = createGame3DView(game3dCanvas);

  let actuatorLimits = DEFAULT_HAND_ACTUATOR_LIMITS;
  let actuatorLabels = handActuatorLimitsLabel(actuatorLimits);

  let presetId = "fast22";
  let preset = PRESETS[presetId];
  let scenario = null;
  let equilibrium = null;
  let state = null;
  let currentBoundary = { ...ZERO_HAND_BOUNDARY };
  let lastBoundaryDiagnostics = null;
  let handActuator = createHandActuatorState();
  let handTarget = clampHandTarget({
    lateralPositionM: 0,
    angleRad: 0,
  });
  let lastActuatorSaturation = {};
  let activePulses = [];
  let cumulativeHandWorkJ = 0;
  let initialRmsM = 0;
  let onsetThresholdM = 0.020;
  let observedOnsetS = null;
  let manualControlUsed = false;
  let controlMode = "human";
  let lastControlSensing = null;
  let activeLqrDesign = null;
  let lastLqrStateNorm = null;
  let lqrDesignBusy = false;
  const lqrDesignCache = new Map();
  let lastAutomaticGameControl = null;
  const comparisonStats = new Map();
  let comparisonDifficultyId = null;
  let history = [];
  let simTime = 0;
  let lastHistoryTime = -Infinity;
  let paused = false;
  let active = false;
  let accumulator = 0;
  let lastFrameMs = performance.now();
  let rafId = null;
  let stoppedReason = null;
  let gameState = null;
  let aimReference = null;
  let aimTarget = null;
  let aimSample = null;
  let activeVisualMode = "3d";
  let initializationBusy = false;
  const solutionCache = new Map();
  let historyDirty = true;

  let pointerId = null;
  let pointerSurface = null;
  let pointerLastX = 0;
  let pointerLastY = 0;

  function loadOptions() {
    return {
      tipLoad: scenario.tipLoad,
      additionalGeneralizedForce: scenario.flowForce,
      additionalCartesianResultant: scenario.flowResultant,
    };
  }

  function pulseBoundaryAtTime(timeS) {
    const values = activePulses.map((pulse) => smoothHandPulse(
      timeS,
      pulse,
    ));
    return values.length > 0
      ? sumHandBoundaries(values)
      : { ...ZERO_HAND_BOUNDARY };
  }

  function reactionForAngles(anglesRad) {
    return nonlinearShowerHeadReaction(
      scenario.system,
      anglesRad,
      {
        flowRateM3s: scenario.flowRateM3s,
        waterDensityKgM3: scenario.params.waterDensityKgM3,
        hoseInnerDiameterM: scenario.params.hoseInnerDiameterM,
        head: scenario.head,
      },
    );
  }

  function reactionForState() {
    return reactionForAngles(state.anglesRad);
  }

  function nozzleOriginFor(current, reaction) {
    return [
      current.tip[0] + reaction.nozzleOffsetWorldM[0],
      current.tip[1] + reaction.nozzleOffsetWorldM[1],
    ];
  }

  function buildAimReference() {
    const equilibriumReaction = reactionForAngles(
      equilibrium.anglesRad,
    );
    return {
      nozzleOrigin: nozzleOriginFor(
        equilibrium.kinematics,
        equilibriumReaction,
      ),
      outletDirection: equilibriumReaction.outletDirection,
    };
  }

  function buildAimTarget(difficulty, elapsedS = 0) {
    if (!aimReference) {
      aimReference = buildAimReference();
    }
    return createAimTarget({
      nozzleOrigin: aimReference.nozzleOrigin,
      outletDirection: aimReference.outletDirection,
      distanceM: difficulty.aimDistanceM,
      normalOffsetM: difficultyAimOffsetM(
        difficulty,
        elapsedS,
      ),
      radiusM: difficulty.aimRadiusM,
    });
  }

  function updateScheduledAimTarget() {
    if (!gameState?.difficultyId) return;
    const difficulty = gameDifficultyById(
      gameState.difficultyId,
    );
    aimTarget = buildAimTarget(
      difficulty,
      gameState.elapsedS,
    );
  }

  function updateAimSample(current, reaction) {
    if (!aimTarget) {
      aimSample = null;
      return null;
    }
    aimSample = evaluateWaterAim({
      nozzleOrigin: nozzleOriginFor(current, reaction),
      outletDirection: reaction.outletDirection,
      target: aimTarget,
    });
    return aimSample;
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

  function solutionCacheKey(flowLpm) {
    return `${presetId}:${Number(flowLpm).toFixed(3)}`;
  }

  function lqrDesignCacheKey() {
    return [
      presetId,
      Number(flow.value).toFixed(3),
      scenario?.system?.params?.segmentCount ?? 0,
      DT.toFixed(6),
      actuatorLimits.lateralMaxSpeedMps.toFixed(4),
      actuatorLimits.lateralMaxAccelerationMps2.toFixed(4),
      actuatorLimits.angularMaxSpeedRadS.toFixed(4),
      actuatorLimits.angularMaxAccelerationRadS2.toFixed(4),
    ].join(":");
  }

  function invalidateActiveLqrDesign() {
    activeLqrDesign = null;
    lastLqrStateNorm = null;
    if (controlMode === "state" || controlMode === "lqr") {
      controlMode = "human";
      controlModeSelect.value = "human";
    }
  }

  async function ensureLqrDesign() {
    if (!scenario || !equilibrium) return null;
    const key = lqrDesignCacheKey();
    const cached = lqrDesignCache.get(key);
    if (cached) {
      activeLqrDesign = cached;
      return cached;
    }

    lqrDesignBusy = true;
    const wasPaused = paused;
    paused = true;
    controlModeSelect.disabled = true;
    statusMetric.textContent = "DESIGNING LQR…";
    statusMetric.className = "status-warn";
    lqrDesignMetric.textContent = "線形化 + DAREを計算中…";
    await nextPaint();

    try {
      const design = designFullStateLqr(
        scenario,
        equilibrium,
        {
          dt: DT,
          limits: actuatorLimits,
          lqrOptions: {
            tolerance: 1e-8,
            maxIterations: 4000,
          },
        },
      );
      lqrDesignCache.set(key, design);
      activeLqrDesign = design;
      return design;
    } catch (error) {
      console.error("LQR design failed", error);
      lqrDesignMetric.textContent = "LQR設計失敗";
      statusMetric.textContent = "LQR設計失敗";
      statusMetric.className = "status-danger";
      return null;
    } finally {
      lqrDesignBusy = false;
      paused = wasPaused;
      controlModeSelect.disabled = gameState?.status === "running";
      lastFrameMs = performance.now();
    }
  }

  function resetSimulationState() {
    state = perturbEquilibriumState(
      scenario,
      equilibrium.anglesRad,
      {
        tipAnglePerturbationRad: 0.02,
        velocityAmplitudeRadS: 0.03,
      },
    );

    handActuator = createHandActuatorState();
    handTarget = clampHandTarget({
      lateralPositionM: 0,
      angleRad: 0,
    }, actuatorLimits);
    currentBoundary = { ...ZERO_HAND_BOUNDARY };
    activePulses = [];
    cumulativeHandWorkJ = 0;
    lastActuatorSaturation = {};
    manualControlUsed = false;
    lastControlSensing = null;
    lastAutomaticGameControl = null;
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
    historyDirty = true;
    simTime = 0;
    lastHistoryTime = -Infinity;
    accumulator = 0;
    stoppedReason = null;
    paused = false;
    pauseButton.textContent = "一時停止";
    lastFrameMs = performance.now();
    recordHistory();
  }

  function applySolvedScenario(solved) {
    scenario = solved.scenario;
    equilibrium = solved.equilibrium;
    aimReference = null;
    invalidateActiveLqrDesign();
    resetSimulationState();
  }

  function preparePresetInputs({ preserveFlow = false } = {}) {
    preset = PRESETS[presetId];
    if (!preserveFlow) flow.value = String(preset.flowLpm);
    const flowLpm = Number(flow.value);
    flowOut.value = `${flowLpm.toFixed(1)} L/min`;
    presetNote.textContent = preset.note;
    return flowLpm;
  }

  function nextPaint() {
    return new Promise((resolve) => {
      requestAnimationFrame(() => requestAnimationFrame(resolve));
    });
  }

  function setInitializationBusy(next) {
    initializationBusy = next;
    const gameLocked = gameState?.status === "running";
    setGameControlLock(next || gameLocked);

    gameDifficultyButtons.forEach((button) => {
      button.disabled = next;
      button.setAttribute("aria-busy", String(next));
    });
    pauseButton.disabled = next;
    resetButton.disabled = next;
    centerHandButton.disabled = next || controlMode !== "human";
    gameRestartButton.setAttribute("aria-busy", String(next));

    if (next) {
      gameRestartButton.disabled = true;
      gameStatusMetric.textContent = "LOADING…";
      gameStatusMetric.className = "status-warn";
      statusMetric.textContent = "平衡解を計算中…";
      statusMetric.className = "status-warn";
    } else {
      updateGameHud();
    }
  }

  function showInitializationError(error) {
    initializationBusy = false;
    setGameControlLock(false);
    gameDifficultyButtons.forEach((button) => {
      button.disabled = false;
      button.setAttribute("aria-busy", "false");
    });
    pauseButton.disabled = false;
    resetButton.disabled = false;
    centerHandButton.disabled = controlMode !== "human";
    gameRestartButton.disabled = !gameState;
    gameStatusMetric.textContent = "ERROR";
    gameStatusMetric.className = "status-danger";
    statusMetric.textContent = "初期化に失敗";
    statusMetric.className = "status-danger";
    console.error("nonlinear shower initialization failed", error);
  }

  async function configureResponsive({
    preserveFlow = false,
  } = {}) {
    const flowLpm = preparePresetInputs({ preserveFlow });
    const key = solutionCacheKey(flowLpm);
    const cached = solutionCache.get(key);
    if (cached) {
      applySolvedScenario(cached);
      render();
      return true;
    }

    setInitializationBusy(true);
    paused = true;
    await nextPaint();

    try {
      const solved = solveContinuation(preset, flowLpm);
      solutionCache.set(key, solved);
      applySolvedScenario(solved);
    } catch (error) {
      showInitializationError(error);
      return false;
    }

    setInitializationBusy(false);
    render();
    return true;
  }

  function setGameControlLock(locked) {
    flow.disabled = locked;
    playback.disabled = locked;
    controlModeSelect.disabled = locked;
    presetButtons.forEach((button) => {
      button.disabled = locked;
    });
    handPulseButtons.forEach((button) => {
      button.disabled = locked;
    });
  }

  async function setControlMode(mode, {
    centerTarget = true,
  } = {}) {
    if (!["human", "p", "pd", "state", "lqr"].includes(mode)) {
      throw new RangeError(`unknown control mode: ${mode}`);
    }

    if (mode === "state" || mode === "lqr") {
      const design = await ensureLqrDesign();
      if (!design) {
        controlMode = "human";
        controlModeSelect.value = "human";
        centerHandButton.disabled = initializationBusy;
        render();
        return false;
      }
    }

    controlMode = mode;
    controlModeSelect.value = mode;
    lastControlSensing = null;
    lastLqrStateNorm = null;

    if (centerTarget) {
      handTarget = clampHandTarget(
        {
          lateralPositionM: 0,
          angleRad: 0,
        },
        actuatorLimits,
      );
    }

    centerHandButton.disabled = mode !== "human"
      || initializationBusy;
    render();
    return true;
  }

  function controlModeLabel(mode) {
    if (mode === "human") return "Human";
    if (mode === "p") return "P";
    if (mode === "pd") return "PD";
    if (mode === "state") return "State FB";
    if (mode === "lqr") return "LQR";
    return mode;
  }

  function comparisonKey(difficultyId, mode) {
    return `${difficultyId}:${mode}`;
  }

  function recordComparisonResult() {
    if (!gameState || gameState.status === "running") return;
    const mode = gameState.controlMode ?? controlMode;
    const key = comparisonKey(gameState.difficultyId, mode);
    const previous = comparisonStats.get(key) ?? {
      attempts: 0,
      successes: 0,
      bestScore: 0,
      lastScore: 0,
      lastHitFraction: 0,
      lastInsideFraction: 0,
      lastRmsMeanM: 0,
      lastEffortJ: 0,
      lastSaturationS: 0,
      lastStatus: "-",
    };
    const next = {
      attempts: previous.attempts + 1,
      successes:
        previous.successes
        + (gameState.status === "success" ? 1 : 0),
      bestScore: Math.max(previous.bestScore, gameState.score),
      lastScore: gameState.score,
      lastHitFraction: gameState.aimHitFraction,
      lastInsideFraction: gameState.insideFraction,
      lastRmsMeanM: gameState.rmsMeanM,
      lastEffortJ: gameState.effortJ,
      lastSaturationS: gameState.saturationS,
      lastStatus: gameState.status,
    };
    comparisonStats.set(key, next);
    comparisonDifficultyId = gameState.difficultyId;
    updateComparisonTable();
  }

  function updateComparisonTable() {
    if (!comparisonBody || !comparisonDifficultyMetric) return;
    const difficultyId = comparisonDifficultyId;
    comparisonDifficultyMetric.textContent = difficultyId
      ? gameDifficultyById(difficultyId).title
      : "未選択";

    const modes = ["human", "p", "pd", "state", "lqr"];
    comparisonBody.replaceChildren();

    for (const mode of modes) {
      const stats = difficultyId
        ? comparisonStats.get(
          comparisonKey(difficultyId, mode),
        )
        : null;
      const row = document.createElement("tr");
      const values = stats
        ? [
            controlModeLabel(mode),
            `${stats.successes}/${stats.attempts} (${Math.round(100 * stats.successes / stats.attempts)}%)`,
            String(stats.lastScore),
            String(stats.bestScore),
            `${(100 * stats.lastHitFraction).toFixed(0)}%`,
            `${(1000 * stats.lastRmsMeanM).toFixed(1)} mm`,
            `${stats.lastEffortJ.toFixed(2)} J`,
            `${stats.lastSaturationS.toFixed(2)} s`,
          ]
        : [
            controlModeLabel(mode),
            "-",
            "-",
            "-",
            "-",
            "-",
            "-",
            "-",
          ];
      for (const value of values) {
        const cell = document.createElement("td");
        cell.textContent = value;
        row.appendChild(cell);
      }
      comparisonBody.appendChild(row);
    }
  }

  function setVisualMode(mode) {
    activeVisualMode = mode;
    const is3d = mode === "3d";
    game3dViewTab.setAttribute("aria-selected", String(is3d));
    debug2dViewTab.setAttribute("aria-selected", String(!is3d));
    game3dViewTab.tabIndex = is3d ? 0 : -1;
    debug2dViewTab.tabIndex = is3d ? -1 : 0;
    game3dViewPanel.hidden = !is3d;
    debug2dViewPanel.hidden = is3d;
    render();
  }

  function hideGameOverlay() {
    gameOverlay.hidden = true;
  }

  function showGameOverlay({
    kicker,
    title,
    body,
    score = "",
    showRestart = false,
  }) {
    gameOverlayKicker.textContent = kicker;
    gameOverlayTitle.textContent = title;
    gameOverlayBody.textContent = body;
    gameOverlayScore.textContent = score;
    resultRestartButton.hidden = !showRestart;
    gameOverlay.hidden = false;
  }

  function showDifficultyIntro(difficulty, mode = controlMode) {
    showGameOverlay({
      kicker: `${difficulty.title} · ${difficulty.subtitle} · ${controlModeLabel(mode)}`,
      title: "STABILIZE + AIM",
      body:
        `RMSを抑えながら照準へ水を当てる。命中率 ${Math.round(100 * difficulty.minHitFraction)}% 以上でクリア。`,
      score: "",
      showRestart: false,
    });
  }

  function showGameResult() {
    if (!gameState || gameState.status === "running") return;
    const success = gameState.status === "success";
    const reason = gameState.failureReason === "aim ratio"
      ? "命中率不足"
      : (gameState.failureReason === "target ratio"
        ? "安定化滞在率不足"
        : (gameState.failureReason === "failure envelope"
          ? "failure envelope超過"
          : ""));
    pauseButton.disabled = true;
    recordComparisonResult();
    const runMode = gameState.controlMode ?? controlMode;
    showGameOverlay({
      kicker: `${gameState.stage.title} · ${controlModeLabel(runMode)}`,
      title: success ? "SUCCESS" : "FAILED",
      body: success
        ? `安定化と照準を両方達成。Hit ${Math.round(100 * gameState.aimHitFraction)}%、mean RMS ${(1000 * gameState.rmsMeanM).toFixed(1)} mm。`
        : `${reason}。Hit ${Math.round(100 * gameState.aimHitFraction)}%、mean RMS ${(1000 * gameState.rmsMeanM).toFixed(1)} mm。`,
      score: `SCORE ${gameState.score}`,
      showRestart: true,
    });
  }

  function updateGameHud() {
    if (initializationBusy) {
      gameStatusMetric.textContent = "LOADING…";
      gameStatusMetric.className = "status-warn";
      gameRestartButton.disabled = true;
      return;
    }

    const hud = gameHudSnapshot(gameState);
    if (!gameState) {
      gameStageMetric.textContent = "未開始";
      gameStatusMetric.textContent = hud.status;
      gameStatusMetric.className = "";
      gameTimeMetric.textContent = hud.timeLabel;
      gameScoreMetric.textContent = hud.scoreLabel;
      gameInsideMetric.textContent = hud.targetLabel;
      gameHitMetric.textContent = hud.hitLabel;
      gameEffortMetric.textContent = hud.effortLabel;
      gameTargetsMetric.textContent = "-";
      gameRestartButton.disabled = true;
      return;
    }

    const stage = gameState.stage;
    gameStageMetric.textContent = stage.title;
    gameStatusMetric.textContent = hud.status;
    gameStatusMetric.className = gameState.status === "success"
      ? "status-ok"
      : (gameState.status === "failed" ? "status-danger" : "");
    gameTimeMetric.textContent = hud.timeLabel;
    gameScoreMetric.textContent = hud.scoreLabel;
    gameInsideMetric.textContent = hud.targetLabel;
    gameHitMetric.textContent = hud.hitLabel;
    gameEffortMetric.textContent = hud.effortLabel;
    const aimOffsetMm = gameState.difficultyId
      ? 1000 * difficultyAimOffsetM(
        stage,
        gameState.elapsedS,
      )
      : 0;
    gameTargetsMetric.textContent =
      `RMS≤${(1000 * stage.targetRmsM).toFixed(0)}mm / Δθ≤${radToDeg(stage.targetTipAngleErrorRad).toFixed(0)}° / aim offset ${aimOffsetMm.toFixed(0)}mm / R=${(1000 * (stage.aimRadiusM ?? 0)).toFixed(0)}mm`;
    gameRestartButton.disabled = false;
  }

  function cancelGame() {
    gameState = null;
    aimTarget = null;
    aimSample = null;
    actuatorLimits = DEFAULT_HAND_ACTUATOR_LIMITS;
    actuatorLabels = handActuatorLimitsLabel(actuatorLimits);
    hideGameOverlay();
    pauseButton.disabled = false;
    setGameControlLock(false);
    updateGameHud();
  }

  async function startGame(difficultyId) {
    if (initializationBusy) return;
    const requestedMode = controlModeSelect.value;
    const difficulty = gameDifficultyById(difficultyId);

    gameState = null;
    aimTarget = null;
    aimSample = null;
    presetId = difficulty.presetId;
    playback.value = "1";
    actuatorLimits = scaleHandActuatorLimits(
      DEFAULT_HAND_ACTUATOR_LIMITS,
      difficulty.authorityScale,
    );
    actuatorLabels = handActuatorLimitsLabel(actuatorLimits);
    showDifficultyIntro(difficulty, requestedMode);

    const ready = await configureResponsive({
      preserveFlow: false,
    });
    if (!ready) return;

    const modeReady = await setControlMode(requestedMode);
    if (!modeReady) {
      hideGameOverlay();
      return;
    }

    aimReference = buildAimReference();
    aimTarget = buildAimTarget(difficulty, 0);
    const current = currentGeometry();
    const reaction = reactionForState();
    updateAimSample(current, reaction);
    gameState = {
      ...createDifficultyGameState(difficultyId),
      controlMode: requestedMode,
    };
    comparisonDifficultyId = difficultyId;
    updateComparisonTable();

    paused = true;
    pauseButton.disabled = false;
    pauseButton.textContent = "一時停止";
    setGameControlLock(true);
    updateGameHud();
    render();

    window.setTimeout(() => {
      if (
        gameState?.difficultyId === difficultyId
        && gameState.status === "running"
      ) {
        hideGameOverlay();
        paused = false;
        lastFrameMs = performance.now();
      }
    }, 650);
  }

  function flashRestartFeedback() {
    gameRestartButton.textContent = "Restarted ✓";
    window.setTimeout(() => {
      gameRestartButton.textContent = "Restart";
    }, 650);
  }

  function restartGame() {
    if (!gameState || initializationBusy) return;
    const difficultyId = gameState.difficultyId;
    const runMode = gameState.controlMode ?? controlMode;
    const difficulty = gameDifficultyById(difficultyId);

    resetSimulationState();
    aimReference = buildAimReference();
    aimTarget = buildAimTarget(difficulty, 0);
    updateAimSample(currentGeometry(), reactionForState());
    gameState = {
      ...createDifficultyGameState(difficultyId),
      controlMode: runMode,
    };
    controlMode = runMode;
    controlModeSelect.value = runMode;
    comparisonDifficultyId = difficultyId;

    paused = false;
    pauseButton.disabled = false;
    pauseButton.textContent = "一時停止";
    setGameControlLock(true);
    hideGameOverlay();
    updateGameHud();
    updateComparisonTable();
    render();
    flashRestartFeedback();
  }

  function recordHistory() {
    if (simTime - lastHistoryTime < HISTORY_SAMPLE_DT - 1e-9) return;
    lastHistoryTime = simTime;
    const metrics = currentMetrics();
    history.push({
      t: simTime,
      rmsMm: metrics.rmsM * 1000,
      tipMm: metrics.tipDisplacementM * 1000,
      handXmm: 1000 * currentBoundary.lateralPositionM,
      handTargetXmm: 1000 * handTarget.lateralPositionM,
      handAngleDeg: radToDeg(currentBoundary.angleRad),
      handTargetAngleDeg: radToDeg(handTarget.angleRad),
    });
    historyDirty = true;
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

    if (controlMode !== "human") {
      onsetMetric.textContent = observedOnsetS === null
        ? "feedback inputあり / 自励onset判定は参考"
        : `${observedOnsetS.toFixed(3)} s / feedback inputあり`;
    } else if (manualControlUsed) {
      onsetMetric.textContent = observedOnsetS === null
        ? "manual inputあり / 自励onset判定は参考"
        : `${observedOnsetS.toFixed(3)} s / manual inputあり`;
    } else {
      onsetMetric.textContent = observedOnsetS === null
        ? `未到達 / threshold ${(onsetThresholdM * 1000).toFixed(1)} mm`
        : `${observedOnsetS.toFixed(3)} s`;
    }
    ciMetric.textContent = preset.ciOnset;

    handMetric.textContent =
      `x ${(1000 * currentBoundary.lateralPositionM).toFixed(1)} mm / θ ${radToDeg(currentBoundary.angleRad).toFixed(1)}°`;
    handTargetMetric.textContent =
      `x* ${(1000 * handTarget.lateralPositionM).toFixed(1)} mm / θ* ${radToDeg(handTarget.angleRad).toFixed(1)}°`;
    handReactionMetric.textContent =
      `F ${lastBoundaryDiagnostics.reactionForceXN.toFixed(2)} N / M ${lastBoundaryDiagnostics.reactionMomentNm.toFixed(3)} N·m`;
    handPowerMetric.textContent =
      `${lastBoundaryDiagnostics.handPowerW >= 0 ? "+" : ""}${lastBoundaryDiagnostics.handPowerW.toFixed(3)} W`;
    handWorkMetric.textContent =
      `${cumulativeHandWorkJ >= 0 ? "+" : ""}${cumulativeHandWorkJ.toFixed(4)} J`;

    const saturated = anySaturation(lastActuatorSaturation);
    actuatorMetric.textContent = saturated
      ? "速度/加速度 limit作動"
      : `v≤${actuatorLabels.lateralMaxSpeedMps.toFixed(2)} m/s, ω≤${actuatorLabels.angularMaxSpeedDegS.toFixed(0)}°/s`;
    actuatorMetric.className = saturated
      ? "status-warn"
      : "status-ok";
    if (controlMode === "human") {
      pointerMetric.textContent = pointerId === null
        ? "待機: 横drag=x*, 縦drag=θ*"
        : "Pointer操作中";
    } else {
      pointerMetric.textContent =
        `${controlMode.toUpperCase()} feedbackがhand targetを生成`;
    }

    if (controlMode === "lqr" && activeLqrDesign) {
      controlSenseMetric.textContent =
        "full rod + hand actuator state";
      controlCommandMetric.textContent =
        `LQR -> x* ${(1000 * handTarget.lateralPositionM).toFixed(1)} mm / θ* ${radToDeg(handTarget.angleRad).toFixed(1)}°`;
      lqrStateMetric.textContent = lastLqrStateNorm === null
        ? "-"
        : `||x||₂ = ${lastLqrStateNorm.toExponential(3)}`;
      const diag = activeLqrDesign.controllability;
      lqrDesignMetric.textContent =
        `rank ${diag.rank}/${diag.dimension} (rod ${diag.rodRank}/${diag.rodDimension}) / residual ${activeLqrDesign.realization.residualNorm.toExponential(2)}`;
    } else if (lastControlSensing) {
      controlSenseMetric.textContent =
        `ex ${(1000 * lastControlSensing.tipLateralErrorM).toFixed(1)} mm / vx ${(1000 * lastControlSensing.tipLateralVelocityMps).toFixed(1)} mm/s / eθ ${radToDeg(lastControlSensing.tipAngleErrorRad).toFixed(1)}° / ω ${radToDeg(lastControlSensing.tipAngularRateRadS).toFixed(1)}°/s`;
      controlCommandMetric.textContent =
        `${controlMode.toUpperCase()} -> x* ${(1000 * handTarget.lateralPositionM).toFixed(1)} mm / θ* ${radToDeg(handTarget.angleRad).toFixed(1)}°`;
      lqrStateMetric.textContent = "-";
      lqrDesignMetric.textContent = "P/PD: tip local sensing";
    } else {
      controlSenseMetric.textContent = controlMode === "human"
        ? "Human mode: feedback sensor未使用"
        : "-";
      controlCommandMetric.textContent = controlMode === "human"
        ? "Pointer -> hand target"
        : "-";
      lqrStateMetric.textContent = "-";
      lqrDesignMetric.textContent = controlMode === "human"
        ? "Human mode"
        : (lqrDesignBusy ? "DESIGNING LQR…" : "-");
    }

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

    const renderPayload = {
      currentKinematics: current,
      equilibriumKinematics: equilibrium.kinematics,
      reaction,
      handBoundary: currentBoundary,
      handTarget,
      handReaction: lastBoundaryDiagnostics,
      aimTarget,
      aimSample,
      showNodes: showNodes.checked,
      stoppedReason,
    };
    if (activeVisualMode === "3d") {
      game3dView.render(renderPayload);
    } else {
      view.render(renderPayload);
    }
    if (historyDirty) {
      view.renderHistory(history);
      historyDirty = false;
    }
    updateMetrics(metrics, current);
    updateGameHud();
  }

  function step() {
    if (gameState?.status === "running") {
      updateScheduledAimTarget();
    }

    if (
      gameState?.status === "running"
      && controlMode !== "human"
    ) {
      const automatic = automaticGameHandTarget(
        controlMode,
        {
          system: scenario.system,
          rodState: state,
          boundary: currentBoundary,
          actuatorState: handActuator,
          equilibriumKinematics: equilibrium.kinematics,
          equilibriumReaction: reactionForAngles(
            equilibrium.anglesRad,
          ),
          aimTarget,
          lqrDesign: activeLqrDesign,
          limits: actuatorLimits,
          stateGainScale: FIXED_STATE_FEEDBACK_GAIN_SCALE,
        },
      );
      handTarget = automatic.target;
      lastAutomaticGameControl = automatic;
      lastControlSensing = automatic.sensing;
      lastLqrStateNorm = automatic.stateFeedback
        ? automatic.stateFeedback.errorNorm
        : null;
    } else if (
      controlMode === "state"
      && !gameState
    ) {
      const automatic = automaticGameHandTarget(
        "state",
        {
          system: scenario.system,
          rodState: state,
          boundary: currentBoundary,
          actuatorState: handActuator,
          equilibriumKinematics: equilibrium.kinematics,
          equilibriumReaction: reactionForAngles(
            equilibrium.anglesRad,
          ),
          aimTarget: null,
          lqrDesign: activeLqrDesign,
          limits: actuatorLimits,
          stateGainScale: FIXED_STATE_FEEDBACK_GAIN_SCALE,
        },
      );
      handTarget = automatic.target;
      lastAutomaticGameControl = automatic;
      lastControlSensing = null;
      lastLqrStateNorm = automatic.stateFeedback.errorNorm;
    } else if (controlMode === "lqr" && !gameState) {
      const result = lqrHandTarget(
        activeLqrDesign,
        state,
        handActuator,
        actuatorLimits,
      );
      handTarget = result.target;
      lastAutomaticGameControl = null;
      lastLqrStateNorm = result.stateNorm;
      lastControlSensing = null;
    } else if (
      (controlMode === "p" || controlMode === "pd")
      && !gameState
    ) {
      lastControlSensing = senseTipFeedback(
        scenario.system,
        state,
        currentBoundary,
        equilibrium.kinematics,
      );
      const feedbackTarget = controllerHandTarget(
        controlMode,
        lastControlSensing,
        { limits: actuatorLimits },
      );
      if (feedbackTarget) {
        handTarget = feedbackTarget;
      }
      lastAutomaticGameControl = null;
      lastLqrStateNorm = null;
    } else if (controlMode === "human") {
      lastAutomaticGameControl = null;
      lastControlSensing = null;
      lastLqrStateNorm = null;
    }

    const actuatorStart = handActuator;
    const actuatorStep = stepHandActuator(
      actuatorStart,
      handTarget,
      DT,
      actuatorLimits,
    );
    const actuatorTrajectory = actuatorBoundaryTrajectory(
      actuatorStart,
      actuatorStep,
      DT,
    );

    const boundaryAtStepTime = (absoluteTimeS) => {
      const actuatorBoundary = actuatorTrajectory(
        absoluteTimeS - simTime,
      );
      const pulseBoundary = pulseBoundaryAtTime(absoluteTimeS);
      return sumHandBoundaries([
        actuatorBoundary,
        pulseBoundary,
      ]);
    };

    const next = stepRodWithHandBoundaryRK4(
      scenario.system,
      state,
      simTime,
      DT,
      boundaryAtStepTime,
      loadOptions(),
    );

    cumulativeHandWorkJ += 0.5
      * (
        lastBoundaryDiagnostics.handPowerW
        + next.diagnostics.handPowerW
      )
      * DT;

    handActuator = actuatorStep.state;
    lastActuatorSaturation = actuatorStep.saturation;
    state = next.state;
    currentBoundary = next.boundary;
    lastBoundaryDiagnostics = next.diagnostics;
    simTime += DT;

    activePulses = activePulses.filter(
      (pulse) => simTime <= pulse.startTimeS + pulse.durationS,
    );

    if (
      !state.anglesRad.every(Number.isFinite)
      || !state.angularRatesRadS.every(Number.isFinite)
      || !Number.isFinite(cumulativeHandWorkJ)
    ) {
      stoppedReason = "数値異常のため停止";
      paused = true;
      return;
    }

    const current = currentGeometry();
    const metrics = currentMetrics(current);
    if (
      observedOnsetS === null
      && metrics.rmsM >= onsetThresholdM
    ) {
      observedOnsetS = simTime;
    }

    const reaction = reactionForState();
    updateAimSample(current, reaction);

    if (gameState?.status === "running") {
      gameState = updateGameState(
        gameState,
        {
          rmsM: metrics.rmsM,
          tipAngleErrorRad:
            current.tipAngleRad - equilibrium.kinematics.tipAngleRad,
          handPowerW: lastBoundaryDiagnostics.handPowerW,
          actuatorSaturated: anySaturation(lastActuatorSaturation),
          waterHit: Boolean(aimSample?.hit),
          waterMissDistanceM:
            aimSample?.missDistanceM ?? Infinity,
          aimQuality: aimSample?.aimQuality ?? 0,
        },
        DT,
      );

      if (gameState.status !== "running") {
        paused = true;
        pauseButton.textContent = "再開";
        setGameControlLock(false);
        showGameResult();
      }
    }

    const maxAngle = Math.max(
      ...state.anglesRad.map((angle) => Math.abs(angle)),
    );
    if (maxAngle > 3.0 || metrics.rmsM > 0.75) {
      stoppedReason = "H1-5 2Dモデルの監査上限に到達";
      paused = true;
    }
  }

  function triggerPulse({
    lateralAmplitudeM = 0,
    angleAmplitudeRad = 0,
  }) {
    if (stoppedReason) return;
    manualControlUsed = true;
    activePulses.push({
      startTimeS: simTime,
      durationS: Number(pulseDuration.value),
      lateralAmplitudeM,
      angleAmplitudeRad,
    });
  }

  function setPointerTargetFromDelta(
    controlCanvas,
    deltaXPx,
    deltaYPx,
  ) {
    const rect = controlCanvas.getBoundingClientRect();
    handTarget = pointerDeltaToHandTarget(
      handTarget,
      {
        deltaXPx,
        deltaYPx,
        widthPx: Math.max(1, rect.width),
        heightPx: Math.max(1, rect.height),
      },
      actuatorLimits,
    );
    manualControlUsed = true;
  }

  function finishPointer(event) {
    if (event.pointerId !== pointerId) return;
    if (pointerSurface?.hasPointerCapture?.(pointerId)) {
      pointerSurface.releasePointerCapture(pointerId);
    }
    pointerSurface?.classList.remove("pointer-active");
    pointerId = null;
    pointerSurface = null;
    render();
  }

  function installPointerControl(controlCanvas) {
    controlCanvas.addEventListener("pointerdown", (event) => {
      if (
        pointerId !== null
        || stoppedReason
        || initializationBusy
        || !scenario
        || controlMode !== "human"
      ) {
        return;
      }
      pointerId = event.pointerId;
      pointerSurface = controlCanvas;
      pointerLastX = event.clientX;
      pointerLastY = event.clientY;
      controlCanvas.setPointerCapture(event.pointerId);
      controlCanvas.classList.add("pointer-active");
      manualControlUsed = true;
      event.preventDefault();
      render();
    });

    controlCanvas.addEventListener("pointermove", (event) => {
      if (
        event.pointerId !== pointerId
        || pointerSurface !== controlCanvas
      ) {
        return;
      }
      const dx = event.clientX - pointerLastX;
      const dy = event.clientY - pointerLastY;
      pointerLastX = event.clientX;
      pointerLastY = event.clientY;
      setPointerTargetFromDelta(controlCanvas, dx, dy);
      render();
      event.preventDefault();
    });

    controlCanvas.addEventListener("pointerup", finishPointer);
    controlCanvas.addEventListener("pointercancel", finishPointer);
  }

  installPointerControl(game3dCanvas);
  installPointerControl(canvas);

  game3dViewTab.addEventListener(
    "click",
    () => setVisualMode("3d"),
  );
  debug2dViewTab.addEventListener(
    "click",
    () => setVisualMode("2d"),
  );
  game3dCamera.addEventListener("change", () => {
    game3dView.setCameraView(game3dCamera.value);
    if (activeVisualMode === "3d") render();
  });

  [game3dViewTab, debug2dViewTab].forEach(
    (tab, index, tabs) => {
      tab.addEventListener("keydown", (event) => {
        if (
          event.key !== "ArrowLeft"
          && event.key !== "ArrowRight"
        ) {
          return;
        }
        event.preventDefault();
        const delta = event.key === "ArrowRight" ? 1 : -1;
        const next = tabs[
          (index + delta + tabs.length) % tabs.length
        ];
        next.focus();
        next.click();
      });
    },
  );

  controlModeSelect.addEventListener("change", async () => {
    if (gameState) {
      await setControlMode("human");
      return;
    }
    await setControlMode(controlModeSelect.value);
  });

  presetButtons.forEach((button) => {
    button.addEventListener("click", async () => {
      if (initializationBusy) return;
      cancelGame();
      presetId = button.dataset.nlPreset;
      await configureResponsive({ preserveFlow: false });
    });
  });

  gameDifficultyButtons.forEach((button) => {
    button.addEventListener("click", async () => {
      await startGame(button.dataset.gameDifficulty);
    });
  });

  gameRestartButton.addEventListener("click", () => {
    restartGame();
  });
  resultRestartButton.addEventListener("click", () => {
    restartGame();
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

  centerHandButton.addEventListener("click", () => {
    if (controlMode !== "human") return;
    handTarget = clampHandTarget({
      lateralPositionM: 0,
      angleRad: 0,
    }, actuatorLimits);
    manualControlUsed = true;
    render();
  });

  flow.addEventListener("input", () => {
    flowOut.value = `${Number(flow.value).toFixed(1)} L/min`;
  });
  flow.addEventListener("change", async () => {
    if (initializationBusy) return;
    cancelGame();
    await configureResponsive({ preserveFlow: true });
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
    if (gameState) {
      restartGame();
    } else {
      resetSimulationState();
      render();
    }
  });

  function frame(now) {
    rafId = null;
    if (!active) return;

    const elapsed = Math.min((now - lastFrameMs) / 1000, 0.05);
    lastFrameMs = now;

    if (
      !initializationBusy
      && scenario
      && state
      && !paused
      && !stoppedReason
    ) {
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

  setVisualMode("3d");
  void setControlMode("human", { centerTarget: false });
  setGameControlLock(false);
  updateGameHud();
  void configureResponsive({ preserveFlow: false });

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
