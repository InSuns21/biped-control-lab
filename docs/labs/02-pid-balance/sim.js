const canvas = document.querySelector("#sim"), ctx = canvas.getContext("2d");
const kp = document.querySelector("#kp"), kd = document.querySelector("#kd"), limit = document.querySelector("#limit");
const g = 9.81, m = 12, l = 0.9;
let theta, omega, torque, fallen, last = performance.now();
const clamp = (x, a, b) => Math.max(a, Math.min(b, x));
function reset() { theta = 7 * Math.PI / 180; omega = 0; torque = 0; fallen = false; }
function updateLabels() { document.querySelector("#kpOut").value = kp.value; document.querySelector("#kdOut").value = kd.value; document.querySelector("#limitOut").value = `${limit.value} N·m`; }
function step(dt) {
  if (fallen) return;
  torque = clamp(-Number(kp.value) * theta - Number(kd.value) * omega, -Number(limit.value), Number(limit.value));
  const alpha = (g/l) * Math.sin(theta) + torque/(m*l*l);
  omega += alpha * dt; theta += omega * dt;
  if (Math.abs(theta) > 75 * Math.PI / 180) fallen = true;
}
function draw() {
  const w=canvas.width,h=canvas.height,p={x:w/2,y:h-70},len=290;
  const tip={x:p.x+len*Math.sin(theta),y:p.y-len*Math.cos(theta)};
  const css=getComputedStyle(document.documentElement), accent=css.getPropertyValue("--accent").trim(), danger=css.getPropertyValue("--danger").trim(), muted=css.getPropertyValue("--muted").trim(), text=css.getPropertyValue("--text").trim();
  ctx.clearRect(0,0,w,h); ctx.strokeStyle=muted;ctx.lineWidth=3;ctx.beginPath();ctx.moveTo(60,p.y+8);ctx.lineTo(w-60,p.y+8);ctx.stroke();
  ctx.strokeStyle=fallen?danger:accent;ctx.lineWidth=9;ctx.lineCap="round";ctx.beginPath();ctx.moveTo(p.x,p.y);ctx.lineTo(tip.x,tip.y);ctx.stroke();ctx.fillStyle=fallen?danger:accent;ctx.beginPath();ctx.arc(tip.x,tip.y,20,0,Math.PI*2);ctx.fill();
  const maxBar=160; ctx.fillStyle=muted;ctx.fillRect(25,80,maxBar,12);ctx.fillStyle=torque>=0?accent:danger;ctx.fillRect(25,80,Math.abs(torque)/Number(limit.value)*maxBar,12);ctx.fillStyle=text;ctx.font="15px system-ui";ctx.fillText("制御トルク |u| / 上限",25,68);
  document.querySelector("#thetaMetric").textContent=`${(theta*180/Math.PI).toFixed(2)}°`;document.querySelector("#omegaMetric").textContent=`${omega.toFixed(2)} rad/s`;document.querySelector("#torqueMetric").textContent=`${torque.toFixed(1)} N·m`;
  const s=document.querySelector("#statusMetric"); s.textContent=fallen?"転倒":Math.abs(theta)<2*Math.PI/180&&Math.abs(omega)<.15?"安定":"制御中";s.className=fallen?"status-danger":s.textContent==="安定"?"status-ok":"";
}
function frame(now){const e=Math.min((now-last)/1000,.05);last=now;const n=5,dt=e/n;for(let i=0;i<n;i++)step(dt);draw();requestAnimationFrame(frame);}
[kp,kd,limit].forEach(el=>el.addEventListener("input",updateLabels));document.querySelector("#reset").addEventListener("click",reset);document.querySelector("#push").addEventListener("click",()=>{omega+=1.1;fallen=false;});document.querySelector("#pushBack").addEventListener("click",()=>{omega-=1.1;fallen=false;});updateLabels();reset();requestAnimationFrame(frame);
