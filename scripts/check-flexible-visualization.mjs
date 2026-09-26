import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import {
  assembleShowerHeadConveyingBeam,
} from "../docs/js/shower/flexible/shower-head.js";
import {
  flowSpeedFromRateM3s,
} from "../docs/js/shower/flexible/conveying-flow.js";
import {
  createNewmarkGeneralLinear,
} from "../docs/js/shower/flexible/integrator.js";
import {
  solveLinear,
} from "../docs/js/shower/flexible/linear-algebra.js";
import {
  H1_3_REFERENCE,
  classifyReferenceFlow,
  referenceInitialPerturbationReduced,
} from "../docs/js/shower/flexible/scenarios.js";

const dt = 0.002;
const lpmToM3s = (lpm) => lpm / 60000;

function runPreset(flowLpm, durationS = 12) {
  const flowSpeedMps = flowSpeedFromRateM3s(
    lpmToM3s(flowLpm),
    H1_3_REFERENCE.innerDiameterM,
  );
  const system = assembleShowerHeadConveyingBeam({
    flowSpeedMps,
    elementCount: H1_3_REFERENCE.elementCount,
    hoseInnerDiameterM: H1_3_REFERENCE.innerDiameterM,
  });
  const equilibrium = solveLinear(
    system.reduced.stiffness,
    system.reduced.headForce0,
  );
  const perturbation = referenceInitialPerturbationReduced(system);
  const q0 = equilibrium.map((value, i) => value + perturbation.q[i]);
  const integrator = createNewmarkGeneralLinear(system.reduced, dt);
  let state = integrator.initialize({
    q: q0,
    v: perturbation.v,
    force: system.reduced.headForce0,
  });

  const early = [];
  const late = [];
  const totalSteps = Math.round(durationS / dt);
  const windowSteps = Math.round(2 / dt);

  for (let i = 0; i < totalSteps; i += 1) {
    state = integrator.step(state, system.reduced.headForce0);
    const dynamicTip = state.q.at(-2) - equilibrium.at(-2);
    assert.ok(Number.isFinite(dynamicTip), "H1-4 preset must stay finite");
    if (i < windowSteps) early.push(dynamicTip);
    if (i >= totalSteps - windowSteps) late.push(dynamicTip);
  }

  const rms = (values) => Math.sqrt(
    values.reduce((sum, value) => sum + value * value, 0) / values.length,
  );
  return {
    earlyRms: rms(early),
    lateRms: rms(late),
  };
}

assert.equal(classifyReferenceFlow(8).id, "stable");
assert.equal(classifyReferenceFlow(14).id, "critical");
assert.equal(classifyReferenceFlow(18).id, "flutter");

const low = runPreset(8);
console.log("H1-4 low preset RMS ratio", low.lateRms / low.earlyRms);
assert.ok(
  low.lateRms < 0.35 * low.earlyRms,
  "H1-4 low-flow preset should visibly decay",
);

const flutter = runPreset(18);
console.log("H1-4 flutter preset RMS ratio", flutter.lateRms / flutter.earlyRms);
assert.ok(
  flutter.lateRms > 1.4 * flutter.earlyRms,
  "H1-4 18 L/min flutter preset should visibly grow",
);

const html = await readFile("docs/labs/x1-shower-tvc/index.html", "utf8");
for (const required of [
  'id="phase1Panel"',
  'id="phase0Panel"',
  'id="p1FlexibleCanvas"',
  'id="p1TipChart"',
  'id="p1RmsChart"',
  'data-flow-lpm="8.0"',
  'data-flow-lpm="14.0"',
  'data-flow-lpm="18.0"',
]) {
  assert.ok(html.includes(required), `missing H1-4 UI marker: ${required}`);
}
assert.ok(
  /id="phase0Panel"[^>]*hidden/.test(html),
  "Phase 0 must be hidden by default",
);
assert.ok(
  !/id="phase1Panel"[^>]*hidden/.test(html),
  "Phase 1 must be the default visible panel",
);

console.log(
  [
    "H1-4 visualization checks OK:",
    `8 L/min RMS ratio=${(low.lateRms / low.earlyRms).toFixed(3)}`,
    `18 L/min RMS ratio=${(flutter.lateRms / flutter.earlyRms).toFixed(3)}`,
    `Qcr=${H1_3_REFERENCE.criticalFlowLpm.toFixed(2)} L/min`,
  ].join(" "),
);
