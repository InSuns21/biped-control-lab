import assert from "node:assert/strict";
import {
  DEFAULT_SHOWER_HEAD_PARAMS,
} from "../docs/js/shower/flexible/shower-head.js";
import {
  createNonlinearRod,
  solveStaticRodEquilibrium,
} from "../docs/js/shower/flexible/nonlinear-rod.js";
import {
  hoseAreaFromDiameterM,
  nonlinearShowerHeadReaction,
  nonlinearShowerHeadTipLoad,
} from "../docs/js/shower/flexible/nonlinear-shower-head.js";

const degToRad = (deg) => deg * Math.PI / 180;
const rotate2 = ([x, y], angle) => [
  Math.cos(angle) * x - Math.sin(angle) * y,
  Math.sin(angle) * x + Math.cos(angle) * y,
];
const nearly = (a, b, tol = 1e-10) => Math.abs(a - b) <= tol;

const diameterM = 0.006;
const rho = 997;
const areaM2 = hoseAreaFromDiameterM(diameterM);
const fluidMassPerM = rho * areaM2;
const flow18M3s = 18 / 60000;

// Exact finite-angle covariance of the bent-head momentum correction.
const covarianceSystem = createNonlinearRod({
  segmentCount: 8,
  gravityMps2: 0,
  fluidMassPerM,
});
const straightAngles = Array(8).fill(0);
const turnedAngles = Array(8).fill(0);
turnedAngles[7] = degToRad(70);
const atZero = nonlinearShowerHeadReaction(
  covarianceSystem,
  straightAngles,
  {
    flowRateM3s: flow18M3s,
    waterDensityKgM3: rho,
    hoseInnerDiameterM: diameterM,
  },
);
const atTurn = nonlinearShowerHeadReaction(
  covarianceSystem,
  turnedAngles,
  {
    flowRateM3s: flow18M3s,
    waterDensityKgM3: rho,
    hoseInnerDiameterM: diameterM,
  },
);
const rotatedForce = rotate2(atZero.forceXYN, -degToRad(70));
assert.ok(nearly(atTurn.forceXYN[0], rotatedForce[0], 1e-10));
assert.ok(nearly(atTurn.forceXYN[1], rotatedForce[1], 1e-10));
assert.ok(
  nearly(atTurn.momentNm, atZero.momentNm, 1e-10),
  "co-rotating nozzle geometry should preserve the rod-frame scalar tip moment",
);

// Zero flow remains exactly zero load.
const zeroReaction = nonlinearShowerHeadReaction(
  covarianceSystem,
  turnedAngles,
  {
    flowRateM3s: 0,
    waterDensityKgM3: rho,
    hoseInnerDiameterM: diameterM,
  },
);
assert.ok(zeroReaction.forceXYN.every((value) => Math.abs(value) < 1e-14));
assert.ok(Math.abs(zeroReaction.momentNm) < 1e-14);

function staticAtFlow(segmentCount, outletAngleRad) {
  const system = createNonlinearRod({
    segmentCount,
    fluidMassPerM,
    gravityMps2: 9.81,
    headMassKg: 0.20,
    headRotInertiaKgM2: 0.002,
    headComAxialOffsetM: 0.055,
    bendingDampingNms: 0,
  });
  const head = {
    ...DEFAULT_SHOWER_HEAD_PARAMS,
    outletAngleRad,
  };
  const tipLoad = ({ state }) => nonlinearShowerHeadTipLoad(
    system,
    state.anglesRad,
    {
      flowRateM3s: flow18M3s,
      waterDensityKgM3: rho,
      hoseInnerDiameterM: diameterM,
      head,
    },
  );
  const result = solveStaticRodEquilibrium(system, { tipLoad });
  assert.ok(result.converged, "nonlinear bent-head equilibrium must converge");
  assert.ok(result.residualNorm < 1e-8);
  return { system, result };
}

const bent12 = staticAtFlow(12, DEFAULT_SHOWER_HEAD_PARAMS.outletAngleRad);
const bent24 = staticAtFlow(24, DEFAULT_SHOWER_HEAD_PARAMS.outletAngleRad);
const tip12 = bent12.result.kinematics.tip;
const tip24 = bent24.result.kinematics.tip;
const angle12 = bent12.result.kinematics.tipAngleRad;
const angle24 = bent24.result.kinematics.tipAngleRad;

assert.ok(
  Math.abs(tip24[0]) > 0.20,
  "18 L/min bent head should produce a clearly finite lateral equilibrium",
);
assert.ok(
  Math.abs(angle24) > degToRad(15),
  "18 L/min bent head should require finite rotation, not a small-angle pose",
);
assert.ok(
  Math.abs(tip12[0] - tip24[0]) / Math.abs(tip24[0]) < 0.06,
  "12 -> 24 segment nonlinear shower equilibrium should converge in tip x",
);
assert.ok(
  Math.abs(angle12 - angle24) / Math.abs(angle24) < 0.05,
  "12 -> 24 segment nonlinear shower equilibrium should converge in tip angle",
);

// Reversing the head bend should mirror the static solution.
const mirror = staticAtFlow(
  24,
  -DEFAULT_SHOWER_HEAD_PARAMS.outletAngleRad,
);
assert.ok(
  Math.abs(mirror.result.kinematics.tip[0] + tip24[0])
    / Math.abs(tip24[0]) < 1e-5,
  "reversing the bend should mirror the horizontal equilibrium",
);
assert.ok(
  Math.abs(mirror.result.kinematics.tipAngleRad + angle24)
    / Math.abs(angle24) < 1e-5,
  "reversing the bend should mirror the tip rotation",
);

console.log(
  [
    "H1-4B nonlinear shower-head checks OK:",
    `Q=18 L/min`,
    `tip x=${tip24[0].toFixed(3)} m`,
    `tip y=${tip24[1].toFixed(3)} m`,
    `tip angle=${(angle24 * 180 / Math.PI).toFixed(1)} deg`,
    `head force=${Math.hypot(...atZero.forceXYN).toFixed(3)} N`,
    `head moment=${atZero.momentNm.toFixed(3)} N m`,
  ].join(" "),
);
