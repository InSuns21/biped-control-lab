import assert from "node:assert/strict";
import {
  DEFAULT_ONE_AXIS_PARAMS,
  DEFAULT_PD_GAINS,
  degToRad,
  gravityTorque1D,
  jetThrustFromFlow,
  jetTorque1D,
  lpmToM3s,
  stepOneAxis,
} from "../docs/js/shower/one-axis.js";

const params = DEFAULT_ONE_AXIS_PARAMS;
const nearly = (a, b, tol = 1e-12) => Math.abs(a - b) <= tol;

assert.equal(
  jetThrustFromFlow(0, params),
  0,
  "Q = 0 should produce zero jet thrust",
);

const thrust4Lpm = jetThrustFromFlow(lpmToM3s(4), params);
const thrust8Lpm = jetThrustFromFlow(lpmToM3s(8), params);
assert.ok(
  nearly(thrust8Lpm / thrust4Lpm, 4),
  "doubling flow should quadruple idealized jet thrust",
);

assert.ok(
  nearly(jetTorque1D(0, thrust8Lpm, params), 0),
  "zero gimbal angle should produce zero jet torque",
);

const positiveJetTorque = jetTorque1D(degToRad(10), thrust8Lpm, params);
const negativeJetTorque = jetTorque1D(degToRad(-10), thrust8Lpm, params);
assert.ok(
  nearly(positiveJetTorque, -negativeJetTorque),
  "reversing gimbal angle should reverse jet torque",
);

assert.ok(
  gravityTorque1D(degToRad(5), params) > 0,
  "positive tilt should receive destabilizing gravity torque",
);

const saturated = stepOneAxis(
  { thetaRad: degToRad(30), omegaRadS: 0 },
  { mode: "pd", kp: 4, kd: 1, flowRateM3s: lpmToM3s(20) },
  0.001,
  params,
);

assert.equal(
  saturated.diagnostics.gimbalSaturated,
  true,
  "large PD command should hit gimbal saturation",
);
assert.equal(
  saturated.diagnostics.flowSaturated,
  true,
  "flow command above Q_max should be clamped",
);
assert.ok(
  nearly(
    Math.abs(saturated.diagnostics.deltaAppliedRad),
    params.gimbalMaxRad,
  ),
  "applied gimbal angle should equal the limit",
);
assert.ok(
  saturated.diagnostics.deltaCommandRad
    !== saturated.diagnostics.deltaAppliedRad,
  "command and applied gimbal angle should remain distinguishable",
);

let pdState = { thetaRad: degToRad(8), omegaRadS: 0 };
let pdDiagnostics;
for (let t = 0; t < 6; t += 0.001) {
  const out = stepOneAxis(
    pdState,
    { mode: "pd", ...DEFAULT_PD_GAINS },
    0.001,
    params,
  );
  pdState = out.state;
  pdDiagnostics = out.diagnostics;
}

assert.ok(
  Math.abs(pdState.thetaRad) < degToRad(0.05),
  "default PD should recover a small tilt close to upright",
);
assert.ok(
  Math.abs(pdState.omegaRadS) < 0.002,
  "default PD should damp angular velocity close to zero",
);
assert.equal(
  pdDiagnostics.gimbalSaturated,
  false,
  "settled default PD state should be unsaturated",
);

let pState = { thetaRad: degToRad(8), omegaRadS: 0 };
for (let t = 0; t < 6; t += 0.001) {
  pState = stepOneAxis(
    pState,
    { mode: "p", kp: DEFAULT_PD_GAINS.kp },
    0.001,
    params,
  ).state;
}

assert.ok(
  Math.abs(pState.omegaRadS) > 0.05,
  "P-only case should retain appreciable oscillatory motion in this ideal model",
);

console.log(
  "Shower TVC X1-1 checks OK: jet scaling, torque signs, saturation, P/PD behavior",
);
