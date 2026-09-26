import {
  DEFAULT_RIGID_BODY_PARAMS,
  createRigidBodyState,
  stepRigidBody3D,
} from "../../js/shower/rigid-body.js";
import {
  quatFromAxisAngle,
} from "../../js/shower/quaternion.js";
import {
  degToRad,
  lpmToM3s,
} from "../../js/shower/one-axis.js";

const canvas = document.querySelector("#scene");
const holdX = document.querySelector("#holdX");
const holdZ = document.querySelector("#holdZ");
const flow = document.querySelector("#flow");
const cameraView = document.querySelector("#cameraView");
const pauseButton = document.querySelector("#pause");
const neutralButton = document.querySelector("#neutral");
const resetButton = document.querySelector("#reset");

const holdXOut = document.querySelector("#holdXOut");
const holdZOut = document.querySelector("#holdZOut");
const flowOut = document.querySelector("#flowOut");
const rpyMetric = document.querySelector("#rpyMetric");
const downErrorMetric = document.querySelector("#downErrorMetric");
const omegaMetric = document.querySelector("#omegaMetric");
const flowMetric = document.querySelector("#flowMetric");
const holdMetric = document.querySelector("#holdMetric");
const thrustMetric = document.querySelector("#thrustMetric");
const waterTorqueMetric = document.querySelector("#waterTorqueMetric");
const holdTorqueMetric = document.querySelector("#holdTorqueMetric");
const saturationMetric = document.querySelector("#saturationMetric");

let view = null;
const params = DEFAULT_RIGID_BODY_PARAMS;
const fixedDt = 1 / 240;
const initialTilt = quatFromAxisAngle([1, 0, 1], degToRad(12));

let state = createRigidBodyState({ qBodyToWorld: initialTilt });
let diagnostics = null;
let paused = false;
let accumulator = 0;
let lastTime = performance.now();

const radToDeg = (rad) => rad * 180 / Math.PI;
const magnitude3 = (v) => Math.hypot(v[0], v[1], v[2]);

function quaternionToRpyDeg(q) {
  const [w, x, y, z] = q;
  const roll = Math.atan2(
    2 * (w * x + y * z),
    1 - 2 * (x * x + y * y),
  );
  const sinPitch = Math.max(
    -1,
    Math.min(1, 2 * (w * y - z * x)),
  );
  const pitch = Math.asin(sinPitch);
  const yaw = Math.atan2(
    2 * (w * z + x * y),
    1 - 2 * (y * y + z * z),
  );
  return [roll, pitch, yaw].map(radToDeg);
}

function readControl() {
  return {
    holdTiltXCommandRad: degToRad(Number(holdX.value)),
    holdTiltZCommandRad: degToRad(Number(holdZ.value)),
    flowRateM3s: lpmToM3s(Number(flow.value)),
  };
}

function updateControlLabels() {
  holdXOut.value = `${Number(holdX.value).toFixed(0)}°`;
  holdZOut.value = `${Number(holdZ.value).toFixed(0)}°`;
  flowOut.value = `${Number(flow.value).toFixed(1)} L/min`;
}

function updateMetrics() {
  const [roll, pitch, yaw] = quaternionToRpyDeg(state.qBodyToWorld);
  rpyMetric.textContent =
    `${roll.toFixed(1)}° / ${pitch.toFixed(1)}° / ${yaw.toFixed(1)}°`;
  omegaMetric.textContent = state.omegaBodyRadS
    .map((value) => value.toFixed(2))
    .join(" / ");

  if (!diagnostics) {
    downErrorMetric.textContent = "-";
    flowMetric.textContent = "-";
    holdMetric.textContent = "-";
    thrustMetric.textContent = "-";
    waterTorqueMetric.textContent = "-";
    holdTorqueMetric.textContent = "-";
    saturationMetric.textContent = "-";
    return;
  }

  downErrorMetric.textContent =
    `${radToDeg(diagnostics.jetDownErrorRad).toFixed(1)}°`;
  flowMetric.textContent =
    `${(diagnostics.flowRateM3s * 60000).toFixed(1)} L/min`;
  holdMetric.textContent =
    `${radToDeg(diagnostics.holdTiltXAppliedRad).toFixed(1)}° / `
    + `${radToDeg(diagnostics.holdTiltZAppliedRad).toFixed(1)}°`;
  thrustMetric.textContent = `${diagnostics.thrustN.toFixed(3)} N`;
  waterTorqueMetric.textContent =
    `${magnitude3(diagnostics.waterReactionTorqueBodyNm).toFixed(4)} N·m`;
  holdTorqueMetric.textContent =
    `${magnitude3(diagnostics.hoseHoldingTorqueBodyNm).toFixed(4)} N·m`;

  const saturated = diagnostics.flowSaturated
    || diagnostics.holdTiltXSaturated
    || diagnostics.holdTiltZSaturated;
  saturationMetric.textContent = saturated ? "SATURATED" : "OK";
  saturationMetric.className = saturated ? "status-danger" : "status-ok";
}

function resetAttitude() {
  state = createRigidBodyState({ qBodyToWorld: initialTilt });
  diagnostics = null;
  accumulator = 0;
  lastTime = performance.now();
}

function setPaused(next) {
  paused = next;
  pauseButton.textContent = paused ? "再開" : "一時停止";
  lastTime = performance.now();
}

[holdX, holdZ, flow].forEach((element) => {
  element.addEventListener("input", updateControlLabels);
});

cameraView.addEventListener("change", () => {
  view?.setCameraView(cameraView.value);
});

pauseButton.addEventListener("click", () => {
  setPaused(!paused);
});

neutralButton.addEventListener("click", () => {
  holdX.value = "0";
  holdZ.value = "0";
  updateControlLabels();
});

resetButton.addEventListener("click", resetAttitude);

function frame(now) {
  const elapsed = Math.min((now - lastTime) / 1000, 0.05);
  lastTime = now;

  if (!paused) {
    accumulator += elapsed;
    while (accumulator >= fixedDt) {
      const result = stepRigidBody3D(
        state,
        readControl(),
        fixedDt,
        params,
      );
      state = result.state;
      diagnostics = result.diagnostics;
      accumulator -= fixedDt;
    }
  }

  view.render(state, diagnostics);
  updateMetrics();
  requestAnimationFrame(frame);
}

function showRenderError(error) {
  canvas.hidden = true;
  const message = document.createElement("div");
  message.className = "callout status-danger";
  message.setAttribute("role", "alert");
  const detail = error instanceof Error ? error.message : String(error);
  message.innerHTML = "<strong>3D描画の初期化に失敗しました。</strong><br>"
    + "ページを再読み込みしても直らない場合は、Human Visual Audit の不具合として報告してください。"
    + "<br><code></code>";
  message.querySelector("code").textContent = detail;
  canvas.parentElement.append(message);
  console.error("Hanging shower renderer initialization failed", error);
}

async function bootstrap() {
  updateControlLabels();
  updateMetrics();

  try {
    const { createShowerScene } = await import("./scene.js");
    view = createShowerScene(canvas);
    requestAnimationFrame(frame);
  } catch (error) {
    showRenderError(error);
  }
}

bootstrap();
