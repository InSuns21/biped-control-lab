export const H1_3_REFERENCE = Object.freeze({
  criticalFlowSpeedMps: 8.3871,
  criticalFlowLpm: 14.23,
  innerDiameterM: 0.006,
  elementCount: 8,
});

export const FLOW_PRESETS = Object.freeze([
  Object.freeze({ id: "low", label: "低流量", flowLpm: 8.0 }),
  Object.freeze({ id: "near", label: "臨界付近", flowLpm: 14.0 }),
  Object.freeze({ id: "flutter", label: "Flutter", flowLpm: 18.0 }),
]);

export function classifyReferenceFlow(flowLpm) {
  const ratio = flowLpm / H1_3_REFERENCE.criticalFlowLpm;

  if (flowLpm <= 1e-9) {
    return {
      id: "stationary",
      label: "充水・流れなし",
      detail: "流れ由来の不安定化なし",
    };
  }
  if (ratio < 0.85) {
    return {
      id: "stable",
      label: "安定",
      detail: "摂動は減衰する領域",
    };
  }
  if (ratio < 0.98) {
    return {
      id: "approaching",
      label: "臨界接近",
      detail: "減衰が弱くなる領域",
    };
  }
  if (ratio <= 1.02) {
    return {
      id: "critical",
      label: "臨界付近",
      detail: "成長率が0付近",
    };
  }
  return {
    id: "flutter",
    label: "Flutter",
    detail: "微小摂動が自励的に成長",
  };
}


export function referenceInitialPerturbationReduced(
  system,
  {
    displacementAmplitudeM = 0.008,
    velocityAmplitudeMps = 0.02,
  } = {},
) {
  const q = [];
  const v = [];
  const L = system.params.lengthM;
  const lastNode = system.nodeCount - 1;

  for (let node = 1; node <= lastNode; node += 1) {
    const x = node / lastNode;

    // Smooth clamped-base displacement dominated by the first bending shape.
    const f = x * x * (3 - 2 * x);
    const dfDx = 6 * x * (1 - x);

    // One-time broadband velocity seed. x^2 preserves y_dot(0)=theta_dot(0)=0.
    // It is an initial condition only, never a continuing animation force.
    const phase = 2 * Math.PI * x;
    const g = x * x * Math.sin(phase);
    const dgDx = 2 * x * Math.sin(phase)
      + 2 * Math.PI * x * x * Math.cos(phase);

    q.push(displacementAmplitudeM * f);
    q.push(displacementAmplitudeM * dfDx / L);
    v.push(velocityAmplitudeMps * g);
    v.push(velocityAmplitudeMps * dgDx / L);
  }

  return { q, v };
}
