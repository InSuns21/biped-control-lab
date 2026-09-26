import {
  generalizedSymmetricEigenvalues,
} from "./linear-algebra.js";

export function naturalFrequenciesHz(system, count = 6) {
  const eigenvalues = generalizedSymmetricEigenvalues(
    system.reduced.stiffness,
    system.reduced.mass,
  )
    .filter((value) => value > 0)
    .sort((a, b) => a - b);

  return eigenvalues
    .slice(0, count)
    .map((lambda) => Math.sqrt(lambda) / (2 * Math.PI));
}

export function cantileverAnalyticFirstFrequencyHz({
  lengthM,
  flexuralRigidityNm2,
  structuralMassPerM,
}) {
  const beta1 = 1.875104068711961;
  const omega1 = beta1 * beta1 * Math.sqrt(
    flexuralRigidityNm2
      / (structuralMassPerM * lengthM ** 4),
  );
  return omega1 / (2 * Math.PI);
}
