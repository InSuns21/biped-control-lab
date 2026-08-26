const canvas = document.querySelector("#sim");
const ctx = canvas.getContext("2d");
const angle = document.querySelector("#angle");
const length = document.querySelector("#length");
const angleOut = document.querySelector("#angleOut");
const lengthOut = document.querySelector("#lengthOut");
const pauseButton = document.querySelector("#pause");
const g = 9.81;
let theta = 0;
let omega = 0;
let t = 0;
let paused = false;
let fallen = false;
let last = performance.now();

function reset() {
  theta = Number(angle.value) * Math.PI / 180;
  omega = 0;
  t = 0;
  fallen = false;
  paused = false;
  pauseButton.textContent = "一時停止";
}
function updateOutputs() {
  angleOut.value = `${Number(angle.value).toFixed(1)}°`;
  lengthOut.value = `${Number(length.value).toFixed(2)} m`;
}
function step(dt) {
  if (fallen) return;
  const l = Number(length.value);
  const alpha = (g / l) * Math.sin(theta);
  omega += alpha * dt;
  theta += omega * dt;
  t += dt;
  if (Math.abs(theta) > 75 * Math.PI / 180) fallen = true;
}
function draw() {
  const w = canvas.width, h = canvas.height;
  ctx.clearRect(0, 0, w, h);
  const style = getComputedStyle(document.documentElement);
  const text = style.getPropertyValue("--text").trim();
  const muted = style.getPropertyValue("--muted").trim();
  const danger = style.getPropertyValue("--danger").trim();
  const accent = style.getPropertyValue("--accent").trim();
  const pivot = { x: w / 2, y: h - 70 };
  const pxLen = 290 * Number(length.value) / 0.9;
  const tip = { x: pivot.x + pxLen * Math.sin(theta), y: pivot.y - pxLen * Math.cos(theta) };
  ctx.strokeStyle = muted; ctx.lineWidth = 3; ctx.beginPath(); ctx.moveTo(70, pivot.y + 8); ctx.lineTo(w - 70, pivot.y + 8); ctx.stroke();
  ctx.strokeStyle = fallen ? danger : accent; ctx.lineWidth = 9; ctx.lineCap = "round"; ctx.beginPath(); ctx.moveTo(pivot.x, pivot.y); ctx.lineTo(tip.x, tip.y); ctx.stroke();
  ctx.fillStyle = fallen ? danger : accent; ctx.beginPath(); ctx.arc(tip.x, tip.y, 20, 0, Math.PI * 2); ctx.fill();
  ctx.fillStyle = text; ctx.font = "18px system-ui"; ctx.fillText("直立 = 不安定平衡点", 24, 34);
  ctx.fillStyle = muted; ctx.font = "15px system-ui"; ctx.fillText("制御入力 u = 0", 24, 58);
  document.querySelector("#thetaMetric").textContent = `${(theta * 180 / Math.PI).toFixed(2)}°`;
  document.querySelector("#omegaMetric").textContent = `${omega.toFixed(2)} rad/s`;
  document.querySelector("#timeMetric").textContent = `${t.toFixed(2)} s`;
  const status = document.querySelector("#statusMetric");
  status.textContent = fallen ? "転倒" : "発散中";
  status.className = fallen ? "status-danger" : "";
}
function frame(now) {
  const elapsed = Math.min((now - last) / 1000, 0.05); last = now;
  if (!paused) {
    const n = 4, dt = elapsed / n;
    for (let i = 0; i < n; i++) step(dt);
  }
  draw(); requestAnimationFrame(frame);
}
angle.addEventListener("input", () => { updateOutputs(); reset(); });
length.addEventListener("input", () => { updateOutputs(); reset(); });
document.querySelector("#reset").addEventListener("click", reset);
pauseButton.addEventListener("click", () => { paused = !paused; pauseButton.textContent = paused ? "再開" : "一時停止"; });
updateOutputs(); reset(); requestAnimationFrame(frame);
