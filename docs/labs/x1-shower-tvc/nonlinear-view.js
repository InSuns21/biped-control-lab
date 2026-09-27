function cssColor(name, fallback) {
  return getComputedStyle(document.documentElement)
    .getPropertyValue(name)
    .trim() || fallback;
}

function fitCanvas(canvas) {
  const ratio = Math.min(window.devicePixelRatio || 1, 2);
  const width = Math.max(1, Math.floor(canvas.clientWidth));
  const cssHeight = Number(canvas.dataset.cssHeight || 0);
  const height = Math.max(
    1,
    Math.floor(cssHeight > 0 ? cssHeight : canvas.clientHeight),
  );
  const rw = Math.floor(width * ratio);
  const rh = Math.floor(height * ratio);
  if (canvas.width !== rw || canvas.height !== rh) {
    canvas.width = rw;
    canvas.height = rh;
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  return { ctx, width, height };
}

export function computeNonlinearWorldFit(
  points,
  width,
  height,
  {
    paddingPx = 34,
    topReservePx = 24,
    bottomReservePx = 30,
  } = {},
) {
  const xs = points.map((p) => p[0]);
  const ys = points.map((p) => p[1]);
  let minX = Math.min(...xs);
  let maxX = Math.max(...xs);
  let minY = Math.min(...ys);
  let maxY = Math.max(...ys);

  const spanX0 = Math.max(0.25, maxX - minX);
  const spanY0 = Math.max(0.40, maxY - minY);
  minX -= 0.10 * spanX0;
  maxX += 0.10 * spanX0;
  minY -= 0.05 * spanY0;
  maxY += 0.08 * spanY0;

  const usableW = Math.max(1, width - 2 * paddingPx);
  const usableH = Math.max(
    1,
    height - topReservePx - bottomReservePx - 2 * paddingPx,
  );
  const scale = Math.min(
    usableW / Math.max(1e-9, maxX - minX),
    usableH / Math.max(1e-9, maxY - minY),
  );

  return {
    scale,
    map([x, y]) {
      return [
        paddingPx + (x - minX) * scale,
        topReservePx + paddingPx + (y - minY) * scale,
      ];
    },
  };
}

function drawArrow(ctx, from, vector, scale, color) {
  const mag = Math.hypot(vector[0], vector[1]);
  if (!(mag > 1e-10)) return;
  const ux = vector[0] / mag;
  const uy = vector[1] / mag;
  const to = [
    from[0] + ux * scale,
    from[1] + uy * scale,
  ];
  const head = 9;

  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = 2.5;
  ctx.beginPath();
  ctx.moveTo(from[0], from[1]);
  ctx.lineTo(to[0], to[1]);
  ctx.stroke();

  ctx.beginPath();
  ctx.moveTo(to[0], to[1]);
  ctx.lineTo(
    to[0] - ux * head - uy * head * 0.55,
    to[1] - uy * head + ux * head * 0.55,
  );
  ctx.lineTo(
    to[0] - ux * head + uy * head * 0.55,
    to[1] - uy * head - ux * head * 0.55,
  );
  ctx.closePath();
  ctx.fill();
}

function drawChart(canvas, history, {
  key,
  title,
  unit,
  minimumMax,
  color,
}) {
  const { ctx, width, height } = fitCanvas(canvas);
  const panel = cssColor("--panel", "#fff");
  const text = cssColor("--text", "#222");
  const muted = cssColor("--muted", "#666");
  const line = cssColor("--line", "#ddd");

  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = panel;
  ctx.fillRect(0, 0, width, height);

  const left = 48;
  const right = 12;
  const top = 26;
  const bottom = 28;
  const w = Math.max(1, width - left - right);
  const h = Math.max(1, height - top - bottom);

  ctx.fillStyle = text;
  ctx.font = "12px system-ui, sans-serif";
  ctx.fillText(title, 10, 17);

  if (history.length < 2) {
    ctx.fillStyle = muted;
    ctx.fillText("履歴を計測中…", left, top + 22);
    return;
  }

  const t0 = history[0].t;
  const t1 = history.at(-1).t;
  let maxY = minimumMax;
  for (const sample of history) {
    maxY = Math.max(maxY, sample[key]);
  }
  maxY *= 1.08;

  const mapX = (t) => left
    + (t - t0) / Math.max(1e-9, t1 - t0) * w;
  const mapY = (value) => top + h
    - value / Math.max(1e-9, maxY) * h;

  ctx.strokeStyle = line;
  ctx.lineWidth = 1;
  ctx.strokeRect(left, top, w, h);

  ctx.fillStyle = muted;
  ctx.fillText(`${maxY.toFixed(0)} ${unit}`, 4, top + 4);
  ctx.fillText(`${t1.toFixed(1)} s`, width - 55, height - 8);

  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  ctx.beginPath();
  history.forEach((sample, i) => {
    const x = mapX(sample.t);
    const y = mapY(sample[key]);
    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });
  ctx.stroke();
}

export function createNonlinearView({
  canvas,
  rmsChart,
  tipChart,
}) {
  const colors = {
    hose: cssColor("--text", "#eef"),
    equilibrium: cssColor("--muted", "#889"),
    line: cssColor("--line", "#445"),
    water: "#4db7e5",
    reaction: "#d68a00",
    node: "#7b5dc7",
    head: "#8b949e",
    accent: cssColor("--accent", "#79a8ff"),
    danger: cssColor("--danger", "#ff8787"),
  };

  function render({
    currentKinematics,
    equilibriumKinematics,
    reaction,
    showNodes,
    stoppedReason,
  }) {
    const { ctx, width, height } = fitCanvas(canvas);
    ctx.clearRect(0, 0, width, height);

    const current = currentKinematics.nodes;
    const equilibrium = equilibriumKinematics.nodes;
    const tip = currentKinematics.tip;
    const nozzle = [
      tip[0] + reaction.nozzleOffsetWorldM[0],
      tip[1] + reaction.nozzleOffsetWorldM[1],
    ];
    const waterEnd = [
      nozzle[0] + reaction.outletDirection[0] * 0.24,
      nozzle[1] + reaction.outletDirection[1] * 0.24,
    ];

    const fit = computeNonlinearWorldFit(
      [
        ...current,
        ...equilibrium,
        nozzle,
        waterEnd,
        [0, 0],
      ],
      width,
      height,
    );
    const map = fit.map;

    const base = map([0, 0]);
    ctx.fillStyle = colors.head;
    ctx.fillRect(base[0] - 22, base[1] - 6, 44, 12);
    ctx.fillStyle = colors.equilibrium;
    ctx.font = "12px system-ui, sans-serif";
    ctx.fillText("手元固定境界", base[0] + 10, base[1] - 10);

    ctx.strokeStyle = colors.equilibrium;
    ctx.lineWidth = 2;
    ctx.setLineDash([5, 5]);
    ctx.beginPath();
    equilibrium.forEach((point, i) => {
      const p = map(point);
      if (i === 0) ctx.moveTo(p[0], p[1]);
      else ctx.lineTo(p[0], p[1]);
    });
    ctx.stroke();
    ctx.setLineDash([]);

    ctx.strokeStyle = colors.hose;
    ctx.lineWidth = 8;
    ctx.lineCap = "round";
    ctx.lineJoin = "round";
    ctx.beginPath();
    current.forEach((point, i) => {
      const p = map(point);
      if (i === 0) ctx.moveTo(p[0], p[1]);
      else ctx.lineTo(p[0], p[1]);
    });
    ctx.stroke();

    if (showNodes) {
      ctx.fillStyle = colors.node;
      current.forEach((point) => {
        const p = map(point);
        ctx.beginPath();
        ctx.arc(p[0], p[1], 3.5, 0, 2 * Math.PI);
        ctx.fill();
      });
    }

    const tipPx = map(tip);
    const nozzlePx = map(nozzle);
    ctx.strokeStyle = colors.head;
    ctx.lineWidth = 13;
    ctx.beginPath();
    ctx.moveTo(tipPx[0], tipPx[1]);
    ctx.lineTo(nozzlePx[0], nozzlePx[1]);
    ctx.stroke();
    ctx.fillStyle = colors.head;
    ctx.beginPath();
    ctx.arc(nozzlePx[0], nozzlePx[1], 9, 0, 2 * Math.PI);
    ctx.fill();

    const waterPx = map(waterEnd);
    ctx.strokeStyle = colors.water;
    ctx.lineWidth = 5;
    ctx.beginPath();
    ctx.moveTo(nozzlePx[0], nozzlePx[1]);
    ctx.lineTo(waterPx[0], waterPx[1]);
    ctx.stroke();

    drawArrow(
      ctx,
      nozzlePx,
      reaction.forceXYN,
      48,
      colors.reaction,
    );

    ctx.fillStyle = colors.equilibrium;
    ctx.font = "12px system-ui, sans-serif";
    ctx.fillText("破線: 非線形静的平衡", 14, height - 14);

    if (stoppedReason) {
      ctx.fillStyle = "rgba(0,0,0,.58)";
      ctx.fillRect(0, 0, width, height);
      ctx.fillStyle = "#fff";
      ctx.textAlign = "center";
      ctx.font = "700 18px system-ui, sans-serif";
      ctx.fillText(stoppedReason, width / 2, height / 2);
      ctx.font = "13px system-ui, sans-serif";
      ctx.fillText("Reset または別プリセットで再開", width / 2, height / 2 + 26);
      ctx.textAlign = "start";
    }
  }

  function renderHistory(history) {
    drawChart(rmsChart, history, {
      key: "rmsMm",
      title: "非線形 centerline RMS（平衡形状との差）",
      unit: "mm",
      minimumMax: 20,
      color: colors.accent,
    });
    drawChart(tipChart, history, {
      key: "tipMm",
      title: "先端位置変化 |Δr_tip|",
      unit: "mm",
      minimumMax: 20,
      color: colors.node,
    });
  }

  return { render, renderHistory };
}
