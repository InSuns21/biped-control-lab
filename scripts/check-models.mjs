import assert from "node:assert/strict";

const g = 9.81;

function simulateOpenLoop() {
  const l = 0.9;
  const initial = 3 * Math.PI / 180;
  let theta = initial, omega = 0;
  const dt = 0.0005;
  for (let t = 0; t < 1; t += dt) {
    omega += (g / l) * Math.sin(theta) * dt;
    theta += omega * dt;
  }
  assert.ok(theta > 3 * initial, "open-loop inverted pendulum should diverge from upright");
}

function simulatePdDefault() {
  const m = 12, l = 0.9, kp = 160, kd = 35, torqueLimit = 80;
  let theta = 7 * Math.PI / 180, omega = 0;
  const dt = 0.0005;
  const clamp = (x, a, b) => Math.max(a, Math.min(b, x));
  for (let t = 0; t < 5; t += dt) {
    const u = clamp(-kp * theta - kd * omega, -torqueLimit, torqueLimit);
    const alpha = (g / l) * Math.sin(theta) + u / (m * l * l);
    omega += alpha * dt;
    theta += omega * dt;
    assert.ok(Math.abs(theta) < 75 * Math.PI / 180, "default PD controller should not fall");
  }
  assert.ok(Math.abs(theta) < 0.002, "default PD controller should converge close to upright");
  assert.ok(Math.abs(omega) < 0.002, "default PD angular velocity should converge close to zero");
}

function verifyPeriodicLipm() {
  const h = 0.85, L = 0.36, T = 0.75;
  const w = Math.sqrt(g / h), a = L / 2;
  const q0 = -a;
  const v0 = a * w / Math.tanh(w * T / 2);
  const qT = q0 * Math.cosh(w * T) + (v0 / w) * Math.sinh(w * T);
  const vT = q0 * w * Math.sinh(w * T) + v0 * Math.cosh(w * T);
  assert.ok(Math.abs(qT - a) < 1e-12, "periodic LIPM should move COM from -a to +a in one step");
  assert.ok(Math.abs(vT - v0) < 1e-12, "symmetric periodic LIPM should recover the same velocity");
  const qAfterFootSwitch = qT - L;
  assert.ok(Math.abs(qAfterFootSwitch + a) < 1e-12, "foot switch should reset relative COM position to -a");
}

function verifyZmpInverse() {
  const h = 0.85, x = 0.04, desiredAcc = -0.3;
  const w2 = g / h;
  const p = x - desiredAcc / w2;
  const recoveredAcc = w2 * (x - p);
  assert.ok(Math.abs(recoveredAcc - desiredAcc) < 1e-12, "inverse ZMP equation should reproduce desired acceleration");
}

simulateOpenLoop();
simulatePdDefault();
verifyPeriodicLipm();
verifyZmpInverse();
console.log("Model regression checks OK: open-loop, PD, periodic LIPM, ZMP inverse");
