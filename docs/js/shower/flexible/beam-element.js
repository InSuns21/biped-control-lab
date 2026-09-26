function scaleMatrix(matrix, scale) {
  return matrix.map((row) => row.map((value) => value * scale));
}

export function hermiteBeamElementMatrices({
  lengthM,
  flexuralRigidityNm2,
  massPerLengthKgM,
}) {
  if (!(lengthM > 0)) throw new RangeError("lengthM must be positive");
  if (!(flexuralRigidityNm2 > 0)) {
    throw new RangeError("flexuralRigidityNm2 must be positive");
  }
  if (!(massPerLengthKgM > 0)) {
    throw new RangeError("massPerLengthKgM must be positive");
  }

  const l = lengthM;
  const l2 = l * l;

  const mass = scaleMatrix([
    [156, 22 * l, 54, -13 * l],
    [22 * l, 4 * l2, 13 * l, -3 * l2],
    [54, 13 * l, 156, -22 * l],
    [-13 * l, -3 * l2, -22 * l, 4 * l2],
  ], massPerLengthKgM * l / 420);

  const stiffness = scaleMatrix([
    [12, 6 * l, -12, 6 * l],
    [6 * l, 4 * l2, -6 * l, 2 * l2],
    [-12, -6 * l, 12, -6 * l],
    [6 * l, 2 * l2, -6 * l, 4 * l2],
  ], flexuralRigidityNm2 / (l * l * l));

  return { mass, stiffness };
}
