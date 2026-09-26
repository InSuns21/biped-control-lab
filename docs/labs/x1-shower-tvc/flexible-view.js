function readCssColor(name, fallback) {
  const value = getComputedStyle(document.documentElement)
    .getPropertyValue(name)
    .trim();
  return value || fallback;
}

function fitCanvas(canvas) {
  const ratio = Math.min(window.devicePixelRatio || 1, 2);
  const width = Math.max(1, Math.floor(canvas.clientWidth));
  const cssHeight = Number(canvas.dataset.cssHeight || 0);
  const height = Math.max(
    1,
    Math.floor(cssHeight > 0 ? cssHeight : canvas.clientHeight),
  );
  const renderWidth = Math.floor(width * ratio);
  const renderHeight = Math.floor(height * ratio);
  if (canvas.width !== renderWidth || canvas.height !== renderHeight) {
    canvas.width = renderWidth;
    canvas.height = renderHeight;
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  return { ctx, width, height };
}

function hermiteTransverse(q1, theta1, q2, theta2, xi, lengthM) {
  const xi2 = xi * xi;
  const xi3 = xi2 * xi;
  const n1 = 1 - 3 * xi2 + 2 * xi3;
  const n2 = lengthM * (xi - 2 * xi2 + xi3);
  const n3 = 3 * xi2 - 2 * xi3;
  const n4 = lengthM * (-xi2 + xi3);
  return n1 * q1 + n2 * theta1 + n3 * q2 + n4 * theta2;
}

export function computeTransverseFit({
  width,
  axialPxPerM,
  requestedScale,
  minTransverseM,
  maxTransverseM,
  leftPx = 28,
  rightPx = 62,
  paddingPx = 18,
}) {
  const desiredPxPerM = axialPxPerM * requestedScale;
  const minM = Math.min(0, minTransverseM);
  const maxM = Math.max(0, maxTransverseM);
  const spanM = Math.max(1e-9, maxM - minM);
  const usableWidth = Math.max(1, width - leftPx - rightPx - 2 * paddingPx);
  const fitPxPerM = usableWidth / spanM;
  const transversePxPerM = Math.min(desiredPxPerM, fitPxPerM);
  const contentWidth = spanM * transversePxPerM;
  const freeWidth = Math.max(0, usableWidth - contentWidth);
  const axisX = leftPx + paddingPx + freeWidth / 2
    - minM * transversePxPerM;
  const fitRatio = desiredPxPerM > 0
    ? transversePxPerM / desiredPxPerM
    : 1;

  return {
    axisX,
    transversePxPerM,
    requestedScale,
    effectiveScale: transversePxPerM / axialPxPerM,
    fitRatio,
    leftBoundPx: axisX + minM * transversePxPerM,
    rightBoundPx: axisX + maxM * transversePxPerM,
  };
}

function sampleCenterlineTransverse(
  system,
  readNode,
  samplesPerElement = 12,
) {
  const values = [];
  for (let e = 0; e < system.params.elementCount; e += 1) {
    const a = readNode(e);
    const b = readNode(e + 1);
    for (let sample = 0; sample <= samplesPerElement; sample += 1) {
      const xi = sample / samplesPerElement;
      values.push(hermiteTransverse(
        a.y,
        a.theta,
        b.y,
        b.theta,
        xi,
        system.elementLengthM,
      ));
    }
  }
  return values;
}

function drawArrow(ctx, x0, y0, x1, y1, color) {
  const dx = x1 - x0;
  const dy = y1 - y0;
  const length = Math.hypot(dx, dy);
  if (length < 1e-6) return;

  const ux = dx / length;
  const uy = dy / length;
  const head = Math.min(10, Math.max(6, length * 0.22));

  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = 2.5;
  ctx.beginPath();
  ctx.moveTo(x0, y0);
  ctx.lineTo(x1, y1);
  ctx.stroke();

  ctx.beginPath();
  ctx.moveTo(x1, y1);
  ctx.lineTo(
    x1 - ux * head - uy * head * 0.55,
    y1 - uy * head + ux * head * 0.55,
  );
  ctx.lineTo(
    x1 - ux * head + uy * head * 0.55,
    y1 - uy * head - ux * head * 0.55,
  );
  ctx.closePath();
  ctx.fill();
}

function drawHistoryChart(canvas, history, {
  key,
  label,
  unit,
  color,
  signed = true,
  minimumRange = 1,
}) {
  const { ctx, width, height } = fitCanvas(canvas);
  const panel = readCssColor("--panel", "#fff");
  const text = readCssColor("--text", "#1b2430");
  const muted = readCssColor("--muted", "#5e6b78");
  const line = readCssColor("--line", "#d9dee7");

  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = panel;
  ctx.fillRect(0, 0, width, height);

  const left = 48;
  const right = 12;
  const top = 26;
  const bottom = 28;
  const plotW = Math.max(1, width - left - right);
  const plotH = Math.max(1, height - top - bottom);

  ctx.font = "12px system-ui, sans-serif";
  ctx.fillStyle = text;
  ctx.fillText(label, 10, 17);

  if (history.length < 2) {
    ctx.fillStyle = muted;
    ctx.fillText("履歴を計測中…", left, top + 24);
    return;
  }

  const tMin = history[0].t;
  const tMax = history.at(-1).t;
  let maxAbs = minimumRange;
  for (const sample of history) {
    maxAbs = Math.max(maxAbs, Math.abs(sample[key]));
  }
  const yMin = signed ? -maxAbs : 0;
  const yMax = maxAbs;

  const mapX = (t) => left + (t - tMin) / Math.max(1e-9, tMax - tMin) * plotW;
  const mapY = (value) => top + (yMax - value) / Math.max(1e-9, yMax - yMin) * plotH;

  ctx.strokeStyle = line;
  ctx.lineWidth = 1;
  ctx.strokeRect(left, top, plotW, plotH);

  if (signed && yMin < 0 && yMax > 0) {
    ctx.beginPath();
    ctx.moveTo(left, mapY(0));
    ctx.lineTo(left + plotW, mapY(0));
    ctx.stroke();
  }

  ctx.fillStyle = muted;
  ctx.fillText(`${yMax.toFixed(1)} ${unit}`, 4, top + 4);
  ctx.fillText(`${tMax.toFixed(1)} s`, Math.max(left, width - 55), height - 8);

  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  ctx.beginPath();
  history.forEach((sample, index) => {
    const x = mapX(sample.t);
    const y = mapY(sample[key]);
    if (index === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });
  ctx.stroke();
}

export function createFlexibleView({
  hoseCanvas,
  tipChart,
  rmsChart,
}) {
  const colors = {
    hose: readCssColor("--text", "#1b2430"),
    muted: readCssColor("--muted", "#5e6b78"),
    line: readCssColor("--line", "#d9dee7"),
    panel: readCssColor("--panel", "#fff"),
    accent: readCssColor("--accent", "#155eef"),
    water: "#4db7e5",
    reaction: "#d68a00",
    head: "#747f8c",
    node: "#7b5dc7",
    critical: "#c92a2a",
  };

  function renderHose({
    system,
    state,
    equilibrium,
    deformationScale,
    showNodes,
    headReaction,
    flowLpm,
    criticalFlowLpm,
    limitExceeded,
  }) {
    const { ctx, width, height } = fitCanvas(hoseCanvas);
    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle = colors.panel;
    ctx.fillRect(0, 0, width, height);

    const top = 36;
    const bottom = 28;
    const L = system.params.lengthM;
    const head = system.head;
    const waterStreamLengthM = 0.18;
    const headReachM = Math.hypot(
      head.nozzleAxialOffsetM,
      head.nozzleTransverseOffsetM,
    );
    const sceneAxialExtentM = L + headReachM + waterStreamLengthM + 0.05;
    const axialPxPerM = (height - top - bottom) / sceneAxialExtentM;

    const qAtNode = (node) => {
      if (node === 0) return { y: 0, theta: 0 };
      const j = 2 * (node - 1);
      return { y: state.q[j], theta: state.q[j + 1] };
    };

    const eqAtNode = (node) => {
      if (node === 0) return { y: 0, theta: 0 };
      const j = 2 * (node - 1);
      return { y: equilibrium[j], theta: equilibrium[j + 1] };
    };

    const tip = qAtNode(system.nodeCount - 1);
    const eqTip = eqAtNode(system.nodeCount - 1);
    const nozzleTransverseFromTip = (tipState) => (
      Math.sin(tipState.theta) * head.nozzleAxialOffsetM
      + Math.cos(tipState.theta) * head.nozzleTransverseOffsetM
    );
    const currentNozzleTransverse = tip.y + nozzleTransverseFromTip(tip);
    const equilibriumNozzleTransverse =
      eqTip.y + nozzleTransverseFromTip(eqTip);
    const outletAngle = tip.theta + head.outletAngleRad;
    const waterEndTransverse = currentNozzleTransverse
      + Math.sin(outletAngle) * waterStreamLengthM;

    const transverseValues = [
      ...sampleCenterlineTransverse(system, qAtNode),
      ...sampleCenterlineTransverse(system, eqAtNode),
      currentNozzleTransverse,
      equilibriumNozzleTransverse,
      waterEndTransverse,
    ];
    const minTransverseM = Math.min(...transverseValues);
    const maxTransverseM = Math.max(...transverseValues);
    const fit = computeTransverseFit({
      width,
      axialPxPerM,
      requestedScale: deformationScale,
      minTransverseM,
      maxTransverseM,
    });
    const centerX = fit.axisX;
    const transversePxPerM = fit.transversePxPerM;

    const mapPoint = (s, transverse) => ({
      x: centerX + transverse * transversePxPerM,
      y: top + s * axialPxPerM,
    });

    ctx.strokeStyle = colors.line;
    ctx.lineWidth = 1;
    ctx.setLineDash([5, 5]);
    ctx.beginPath();
    ctx.moveTo(centerX, top);
    ctx.lineTo(centerX, top + L * axialPxPerM);
    ctx.stroke();
    ctx.setLineDash([]);

    ctx.fillStyle = colors.muted;
    ctx.font = "12px system-ui, sans-serif";
    ctx.fillText("手元境界", centerX + 10, top - 12);
    ctx.fillText("基準軸", centerX + 8, top + 18);

    const handY = top;
    ctx.fillStyle = colors.head;
    ctx.fillRect(centerX - 22, handY - 6, 44, 12);

    ctx.strokeStyle = colors.line;
    ctx.lineWidth = 2;
    ctx.setLineDash([4, 4]);
    ctx.beginPath();
    for (let e = 0; e < system.params.elementCount; e += 1) {
      const a = eqAtNode(e);
      const b = eqAtNode(e + 1);
      for (let sample = 0; sample <= 10; sample += 1) {
        const xi = sample / 10;
        const s = (e + xi) * system.elementLengthM;
        const y = hermiteTransverse(
          a.y,
          a.theta,
          b.y,
          b.theta,
          xi,
          system.elementLengthM,
        );
        const p = mapPoint(s, y);
        if (e === 0 && sample === 0) ctx.moveTo(p.x, p.y);
        else ctx.lineTo(p.x, p.y);
      }
    }
    ctx.stroke();
    ctx.setLineDash([]);

    ctx.strokeStyle = colors.hose;
    ctx.lineWidth = 7;
    ctx.lineCap = "round";
    ctx.lineJoin = "round";
    ctx.beginPath();
    for (let e = 0; e < system.params.elementCount; e += 1) {
      const a = qAtNode(e);
      const b = qAtNode(e + 1);
      for (let sample = 0; sample <= 12; sample += 1) {
        const xi = sample / 12;
        const s = (e + xi) * system.elementLengthM;
        const y = hermiteTransverse(
          a.y,
          a.theta,
          b.y,
          b.theta,
          xi,
          system.elementLengthM,
        );
        const p = mapPoint(s, y);
        if (e === 0 && sample === 0) ctx.moveTo(p.x, p.y);
        else ctx.lineTo(p.x, p.y);
      }
    }
    ctx.stroke();

    if (showNodes) {
      ctx.fillStyle = colors.node;
      for (let node = 0; node < system.nodeCount; node += 1) {
        const n = qAtNode(node);
        const p = mapPoint(node * system.elementLengthM, n.y);
        ctx.beginPath();
        ctx.arc(p.x, p.y, 3.5, 0, 2 * Math.PI);
        ctx.fill();
      }
    }

    const tipPoint = mapPoint(L, tip.y);
    const theta = tip.theta;

    const ca = Math.cos(theta);
    const sa = Math.sin(theta);
    const nozzleAxial = ca * head.nozzleAxialOffsetM
      - sa * head.nozzleTransverseOffsetM;
    const nozzleTransverse = sa * head.nozzleAxialOffsetM
      + ca * head.nozzleTransverseOffsetM;
    const nozzle = {
      x: tipPoint.x + nozzleTransverse * transversePxPerM,
      y: tipPoint.y + nozzleAxial * axialPxPerM,
    };

    ctx.strokeStyle = colors.head;
    ctx.lineWidth = 13;
    ctx.lineCap = "round";
    ctx.beginPath();
    ctx.moveTo(tipPoint.x, tipPoint.y);
    ctx.lineTo(nozzle.x, nozzle.y);
    ctx.stroke();

    ctx.fillStyle = colors.head;
    ctx.beginPath();
    ctx.arc(nozzle.x, nozzle.y, 10, 0, 2 * Math.PI);
    ctx.fill();

    if (flowLpm > 1e-6) {
      const streamLength = waterStreamLengthM * axialPxPerM;
      const streamEnd = {
        x: nozzle.x + Math.sin(outletAngle) * streamLength,
        y: nozzle.y + Math.cos(outletAngle) * streamLength,
      };
      ctx.strokeStyle = colors.water;
      ctx.lineWidth = 5;
      ctx.beginPath();
      ctx.moveTo(nozzle.x, nozzle.y);
      ctx.lineTo(streamEnd.x, streamEnd.y);
      ctx.stroke();

      const reaction = headReaction.forceXYN;
      const forceMagnitude = Math.hypot(reaction[0], reaction[1]);
      if (forceMagnitude > 1e-8) {
        const reactionScale = 44 / Math.max(0.5, forceMagnitude);
        drawArrow(
          ctx,
          nozzle.x,
          nozzle.y,
          nozzle.x + reaction[1] * reactionScale,
          nozzle.y + reaction[0] * reactionScale,
          colors.reaction,
        );
      }
    }

    const barX = width - 24;
    const barTop = top;
    const barBottom = height - bottom;
    const criticalRatio = Math.min(1, criticalFlowLpm / 18);
    const flowRatio = Math.min(1, flowLpm / 18);
    ctx.strokeStyle = colors.line;
    ctx.lineWidth = 5;
    ctx.beginPath();
    ctx.moveTo(barX, barTop);
    ctx.lineTo(barX, barBottom);
    ctx.stroke();

    ctx.strokeStyle = flowLpm > criticalFlowLpm
      ? colors.critical
      : colors.accent;
    ctx.beginPath();
    ctx.moveTo(barX, barBottom);
    ctx.lineTo(barX, barBottom - flowRatio * (barBottom - barTop));
    ctx.stroke();

    ctx.fillStyle = colors.critical;
    const criticalY = barBottom - criticalRatio * (barBottom - barTop);
    ctx.fillRect(barX - 7, criticalY - 1, 14, 2);
    ctx.font = "11px system-ui, sans-serif";
    ctx.fillText("Qcr", barX - 35, criticalY + 4);

    if (limitExceeded) {
      ctx.fillStyle = "rgba(0,0,0,.58)";
      ctx.fillRect(0, 0, width, height);
      ctx.fillStyle = "#fff";
      ctx.font = "700 18px system-ui, sans-serif";
      ctx.textAlign = "center";
      ctx.fillText("線形 small-deflection 範囲を超えたため停止", width / 2, height / 2);
      ctx.font = "13px system-ui, sans-serif";
      ctx.fillText("Reset で再開してください", width / 2, height / 2 + 26);
      ctx.textAlign = "start";
    }

    return {
      autoFitActive: fit.fitRatio < 0.999,
      requestedScale: deformationScale,
      effectiveScale: fit.effectiveScale,
      fitRatio: fit.fitRatio,
    };
  }

  function renderCharts(history) {
    drawHistoryChart(tipChart, history, {
      key: "tipMm",
      label: "先端変位（静的平衡からの差）",
      unit: "mm",
      color: colors.accent,
      signed: true,
      minimumRange: 2,
    });
    drawHistoryChart(rmsChart, history, {
      key: "rmsMm",
      label: "ホース動的 RMS",
      unit: "mm",
      color: colors.node,
      signed: false,
      minimumRange: 2,
    });
  }

  return {
    renderHose,
    renderCharts,
  };
}
