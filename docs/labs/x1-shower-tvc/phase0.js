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

const radToDeg = (rad) => rad * 180 / Math.PI;
const magnitude3 = (v) => Math.hypot(v[0], v[1], v[2]);

function quaternionToRpyDeg(q) {
  const [w, x, y, z] = q;
  const roll = Math.atan2(
    2 * (w * x + y * z),
    1 - 2 * (x * x + y * y),
  );
  const sinPitch = Math.max(-1, Math.min(1, 2 * (w * y - z * x)));
  const pitch = Math.asin(sinPitch);
  const yaw = Math.atan2(
    2 * (w * z + x * y),
    1 - 2 * (y * y + z * z),
  );
  return [roll, pitch, yaw].map(radToDeg);
}

export function mountPhase0(root) {
  const canvas = root.querySelector("#p0Scene");
  const holdX = root.querySelector("#p0HoldX");
  const holdZ = root.querySelector("#p0HoldZ");
  const flow = root.querySelector("#p0Flow");
  const cameraView = root.querySelector("#p0CameraView");
  const pauseButton = root.querySelector("#p0Pause");
  const neutralButton = root.querySelector("#p0Neutral");
  const resetButton = root.querySelector("#p0Reset");

  const holdXOut = root.querySelector("#p0HoldXOut");
  const holdZOut = root.querySelector("#p0HoldZOut");
  const flowOut = root.querySelector("#p0FlowOut");
  const rpyMetric = root.querySelector("#p0RpyMetric");
  const downErrorMetric = root.querySelector("#p0DownErrorMetric");
  const omegaMetric = root.querySelector("#p0OmegaMetric");
  const flowMetric = root.querySelector("#p0FlowMetric");
  const holdMetric = root.querySelector("#p0HoldMetric");
  const thrustMetric = root.querySelector("#p0ThrustMetric");
  const waterTorqueMetric = root.querySelector("#p0WaterTorqueMetric");
  const holdTorqueMetric = root.querySelector("#p0HoldTorqueMetric");
  const saturationMetric = root.querySelector("#p0SaturationMetric");

  const view = createShowerScene(canvas);
  const params = DEFAULT_RIGID_BODY_PARAMS;
  const fixedDt = 1 / 240;
  const initialTilt = quatFromAxisAngle([1, 0, 1], degToRad(12));

  let state = createRigidBodyState({ qBodyToWorld: initialTilt });
  let diagnostics = null;
  let paused = false;
  let active = false;
  let accumulator = 0;
  let lastTime = performance.now();
  let rafId = null;

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
    render();
  }

  function render() {
    view.render(state, diagnostics);
    updateMetrics();
  }

  function frame(now) {
    rafId = null;
    if (!active) return;

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

    render();
    rafId = requestAnimationFrame(frame);
  }

  [holdX, holdZ, flow].forEach((element) => {
    element.addEventListener("input", updateControlLabels);
  });

  cameraView.addEventListener("change", () => {
    view.setCameraView(cameraView.value);
    render();
  });

  pauseButton.addEventListener("click", () => {
    paused = !paused;
    pauseButton.textContent = paused ? "再開" : "一時停止";
    lastTime = performance.now();
  });

  neutralButton.addEventListener("click", () => {
    holdX.value = "0";
    holdZ.value = "0";
    updateControlLabels();
  });

  resetButton.addEventListener("click", resetAttitude);

  updateControlLabels();
  updateMetrics();
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
