export const H1_3_REFERENCE = Object.freeze({
  criticalFlowSpeedMps: 8.3871,
  criticalFlowLpm: 14.23,
  innerDiameterM: 0.006,
  elementCount: 8,
});

export const FLOW_PRESETS = Object.freeze([
  Object.freeze({ id: "low", label: "低流量", flowLpm: 8.0 }),
  Object.freeze({ id: "near", label: "臨界付近", flowLpm: 14.0 }),
  Object.freeze({ id: "flutter", label: "Flutter", flowLpm: 16.0 }),
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
