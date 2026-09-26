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
import { createShowerScene } from "./scene.js";

const canvas = document.querySelector("#scene");
const deltaX = document.querySelector("#deltaX");
const deltaZ = document.querySelector("#deltaZ");
const flow = document.querySelector("#flow");
const cameraView = document.querySelector("#cameraView");
const pauseButton = document.querySelector("#pause");
const neutralButton = document.querySelector("#neutral");
const resetButton = document.querySelector("#reset");

const deltaXOut = document.querySelector("#deltaXOut");
const deltaZOut = document.querySelector("#deltaZOut");
const flowOut = document.querySelector("#flowOut");
const rpyMetric = document.querySelector("#rpyMetric");
const omegaMetric = document.querySelector("#omegaMetric");
const flowMetric = document.querySelector("#flowMetric");
const gimbalMetric = document.querySelector("#gimbalMetric");
const thrustMetric = document.querySelector("#thrustMetric");
const torqueMetric = document.querySelector("#torqueMetric");
const saturationMetric = document.querySelector("#saturationMetric");

const view = createShowerScene(canvas);
const params = DEFAULT_RIGID_BODY_PARAMS;
const fixedDt = 1 / 240;
const initialTilt = quatFromAxisAngle([1, 0, 1], degToRad(10));

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
    deltaXCommandRad: degToRad(Number(deltaX.value)),
    deltaZCommandRad: degToRad(Number(deltaZ.value)),
    flowRateM3s: lpmToM3s(Number(flow.value)),
  };
}

function updateControlLabels() {
  deltaXOut.value = `${Number(deltaX.value).toFixed(0)}°`;
  deltaZOut.value = `${Number(deltaZ.value).toFixed(0)}°`;
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
    flowMetric.textContent = "-";
    gimbalMetric.textContent = "-";
    thrustMetric.textContent = "-";
    torqueMetric.textContent = "-";
    saturationMetric.textContent = "-";
    return;
  }

  flowMetric.textContent =
    `${(diagnostics.flowRateM3s * 60000).toFixed(1)} L/min`;
  gimbalMetric.textContent =
    `${radToDeg(diagnostics.deltaXAppliedRad).toFixed(1)}° / `
    + `${radToDeg(diagnostics.deltaZAppliedRad).toFixed(1)}°`;
  thrustMetric.textContent = `${diagnostics.thrustN.toFixed(3)} N`;
  torqueMetric.textContent =
    `${magnitude3(diagnostics.jetTorqueBodyNm).toFixed(4)} N·m`;

  const saturated = diagnostics.flowSaturated
    || diagnostics.gimbalXSaturated
    || diagnostics.gimbalZSaturated;
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

[deltaX, deltaZ, flow].forEach((element) => {
  element.addEventListener("input", updateControlLabels);
});

cameraView.addEventListener("change", () => {
  view.setCameraView(cameraView.value);
});

pauseButton.addEventListener("click", () => {
  setPaused(!paused);
});

neutralButton.addEventListener("click", () => {
  deltaX.value = "0";
  deltaZ.value = "0";
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

updateControlLabels();
updateMetrics();
requestAnimationFrame(frame);
