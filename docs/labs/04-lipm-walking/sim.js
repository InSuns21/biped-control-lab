const canvas = document.querySelector("#sim");
const ctx = canvas.getContext("2d");
const height = document.querySelector("#height");
const stepLimit = document.querySelector("#stepLimit");
const stepTime = document.querySelector("#stepTime");
const autoStep = document.querySelector("#autoStep");
const mode = document.querySelector("#mode");
const g = 9.81;
const halfFoot = 0.11;
const kp = 5.5;
const kd = 4.3;
const clamp = (z, a, b) => Math.max(a, Math.min(b, z));
let x, v, p, support, otherFoot, steps, simTime, nextStep;
let last = performance.now();

function omega() { return Math.sqrt(g / Number(height.value)); }
function reset() {
  simTime = 0;
  steps = 0;
  const L = Number(stepLimit.value);
  support = 0;
  otherFoot = -L;
  p = support;
  if (mode.value === "walk") {
    const w = omega();
    const T = Number(stepTime.value);
    const a = L / 2;
    x = support - a;
    v = a * w / Math.tanh(w * T / 2);
    nextStep = T;
  } else {
    x = 0;
    v = 0;
    nextStep = Infinity;
  }
}
function labels() {
  document.querySelector("#heightOut").value = `${Number(height.value).toFixed(2)} m`;
  document.querySelector("#stepOut").value = `${Number(stepLimit.value).toFixed(2)} m`;
  document.querySelector("#timeOut").value = `${Number(stepTime.value).toFixed(2)} s`;
}
function exactLipmStep(dt, zmp) {
  const w = omega();
  const q = x - zmp;
  const c = Math.cosh(w * dt);
  const s = Math.sinh(w * dt);
  x = zmp + q * c + (v / w) * s;
  v = q * w * s + v * c;
}
function emergencyStep(cp) {
  const delta = clamp(cp - support, -Number(stepLimit.value), Number(stepLimit.value));
  if (Math.abs(delta) < 0.04) return false;
  otherFoot = support;
  support += delta;
  p = support;
  steps++;
  nextStep = simTime + Number(stepTime.value);
  return true;
}
function performPlannedStep() {
  otherFoot = support;
  support += Number(stepLimit.value);
  p = support;
  steps++;
  nextStep += Number(stepTime.value);
}
function walkStep(dt) {
  const w = omega();
  const cp = x + v / w;
  if (autoStep.value === "on" && Math.abs(cp - support) > Number(stepLimit.value) + halfFoot) emergencyStep(cp);
  let remaining = dt;
  while (remaining > 1e-9) {
    const untilStep = nextStep - simTime;
    if (untilStep <= 1e-9) {
      performPlannedStep();
      continue;
    }
    const chunk = Math.min(remaining, untilStep);
    p = support;
    exactLipmStep(chunk, p);
    simTime += chunk;
    remaining -= chunk;
    if (Math.abs(simTime - nextStep) < 1e-8) performPlannedStep();
  }
}
function recoveryStep(dt) {
  const w = omega();
  const cp = x + v / w;
  const desiredAcc = -kp * (x - support) - kd * v;
  const desiredP = x - desiredAcc / (w * w);
  p = clamp(desiredP, support - halfFoot, support + halfFoot);
  if (autoStep.value === "on" && Math.abs(cp - support) > halfFoot + 0.035) emergencyStep(cp);
  const acc = w * w * (x - p);
  v += acc * dt;
  x += v * dt;
}
function step(dt) {
  if (mode.value === "walk") walkStep(dt);
  else { simTime += dt; recoveryStep(dt); }
}
function sx(world) { return canvas.width / 2 + (world - x) * 430; }
function draw() {
  const css = getComputedStyle(document.documentElement);
  const accent = css.getPropertyValue("--accent").trim();
  const danger = css.getPropertyValue("--danger").trim();
  const ok = css.getPropertyValue("--ok").trim();
  const muted = css.getPropertyValue("--muted").trim();
  const text = css.getPropertyValue("--text").trim();
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  const ground = 405;
  const comY = ground - Number(height.value) * 260;
  const w = omega();
  const cp = x + v / w;
  ctx.strokeStyle = muted; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(25, ground); ctx.lineTo(canvas.width - 25, ground); ctx.stroke();
  function foot(pos, active) {
    const left = sx(pos - halfFoot), right = sx(pos + halfFoot);
    ctx.fillStyle = active ? "rgba(70,130,255,.28)" : "rgba(130,130,130,.18)";
    ctx.strokeStyle = active ? accent : muted; ctx.lineWidth = 3;
    ctx.fillRect(left, ground - 13, right - left, 13); ctx.strokeRect(left, ground - 13, right - left, 13);
  }
  foot(otherFoot, false); foot(support, true);
  ctx.setLineDash([6, 5]); ctx.strokeStyle = muted; ctx.beginPath(); ctx.moveTo(sx(x), comY); ctx.lineTo(sx(x), ground); ctx.stroke(); ctx.setLineDash([]);
  ctx.strokeStyle = accent; ctx.lineWidth = 5; ctx.beginPath(); ctx.moveTo(sx(support), ground - 13); ctx.lineTo(sx(x), comY); ctx.stroke();
  ctx.fillStyle = accent; ctx.beginPath(); ctx.arc(sx(x), comY, 16, 0, Math.PI * 2); ctx.fill();
  ctx.fillStyle = ok; ctx.beginPath(); ctx.arc(sx(p), ground - 4, 8, 0, Math.PI * 2); ctx.fill();
  ctx.fillStyle = danger; ctx.beginPath(); ctx.moveTo(sx(cp), ground - 28); ctx.lineTo(sx(cp) - 9, ground - 45); ctx.lineTo(sx(cp) + 9, ground - 45); ctx.closePath(); ctx.fill();
  ctx.fillStyle = text; ctx.font = "15px system-ui"; ctx.fillText("COM", sx(x) + 20, comY + 5);
  ctx.fillStyle = ok; ctx.fillText("ZMP", sx(p) + 10, ground - 20);
  ctx.fillStyle = danger; ctx.fillText("CP", sx(cp) + 10, ground - 48);
  ctx.fillStyle = text; ctx.font = "17px system-ui"; ctx.fillText(mode.value === "walk" ? "周期LIPM歩行" : "立位ZMP制御", 24, 34);
  document.querySelector("#comMetric").textContent = `${x.toFixed(3)} m`;
  document.querySelector("#zmpMetric").textContent = `${p.toFixed(3)} m`;
  document.querySelector("#cpMetric").textContent = `${cp.toFixed(3)} m`;
  document.querySelector("#stepMetric").textContent = String(steps);
}
function frame(nowMs) {
  const elapsed = Math.min((nowMs - last) / 1000, 0.04);
  last = nowMs;
  const n = 5, dt = elapsed / n;
  for (let i = 0; i < n; i++) step(dt);
  draw();
  requestAnimationFrame(frame);
}
[height, stepLimit, stepTime].forEach(el => el.addEventListener("input", () => { labels(); reset(); }));
mode.addEventListener("change", reset);
document.querySelector("#push").addEventListener("click", () => { v += 0.55; });
document.querySelector("#pushBack").addEventListener("click", () => { v -= 0.55; });
document.querySelector("#reset").addEventListener("click", reset);
labels(); reset(); requestAnimationFrame(frame);
