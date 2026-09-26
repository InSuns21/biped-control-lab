import {
  DEFAULT_SHOWER_HEAD_PARAMS,
  showerHeadMomentumReaction2D,
} from "./shower-head.js";
import {
  generalizedTipLoad,
} from "./nonlinear-rod.js";

export const DEFAULT_NONLINEAR_SHOWER_FLOW_PARAMS = Object.freeze({
  waterDensityKgM3: 997,
  hoseInnerDiameterM: 0.006,
  flowRateM3s: 0,
});

export function hoseAreaFromDiameterM(innerDiameterM) {
  if (!(innerDiameterM > 0)) {
    throw new RangeError("innerDiameterM must be positive");
  }
  return Math.PI * innerDiameterM * innerDiameterM / 4;
}

export function nonlinearShowerHeadReaction(
  system,
  anglesRad,
  {
    waterDensityKgM3 = DEFAULT_NONLINEAR_SHOWER_FLOW_PARAMS.waterDensityKgM3,
    hoseInnerDiameterM = DEFAULT_NONLINEAR_SHOWER_FLOW_PARAMS.hoseInnerDiameterM,
    flowRateM3s = DEFAULT_NONLINEAR_SHOWER_FLOW_PARAMS.flowRateM3s,
    head = DEFAULT_SHOWER_HEAD_PARAMS,
  } = {},
) {
  if (!(flowRateM3s >= 0)) {
    throw new RangeError("flowRateM3s must be non-negative");
  }
  const areaM2 = hoseAreaFromDiameterM(hoseInnerDiameterM);
  const flowSpeedMps = flowRateM3s / areaM2;
  const tipAngleRad = anglesRad.at(-1);

  return showerHeadMomentumReaction2D({
    flowSpeedMps,
    fluidDensityKgM3: waterDensityKgM3,
    hoseAreaM2: areaM2,
    tipAngleRad,
    head,
  });
}

export function nonlinearShowerHeadGeneralizedForce(
  system,
  anglesRad,
  flowOptions = {},
) {
  const reaction = nonlinearShowerHeadReaction(
    system,
    anglesRad,
    flowOptions,
  );
  return {
    reaction,
    generalizedForce: generalizedTipLoad(
      system,
      anglesRad,
      {
        forceXYN: reaction.forceXYN,
        momentNm: reaction.momentNm,
      },
    ),
  };
}

export function nonlinearShowerHeadTipLoad(
  system,
  anglesRad,
  flowOptions = {},
) {
  const reaction = nonlinearShowerHeadReaction(
    system,
    anglesRad,
    flowOptions,
  );
  return {
    forceXYN: reaction.forceXYN,
    momentNm: reaction.momentNm,
    reaction,
  };
}
