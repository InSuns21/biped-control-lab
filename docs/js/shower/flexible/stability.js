import {
  identity,
  solveLinear,
  zeros,
} from "./linear-algebra.js";

function solveMatrixColumns(matrix, rhsMatrix) {
  const rows = rhsMatrix.length;
  const cols = rhsMatrix[0].length;
  const out = zeros(rows, cols);

  for (let col = 0; col < cols; col += 1) {
    const rhs = rhsMatrix.map((row) => row[col]);
    const solution = solveLinear(matrix, rhs);
    for (let row = 0; row < rows; row += 1) {
      out[row][col] = solution[row];
    }
  }
  return out;
}

export function secondOrderStateMatrix(system) {
  const { mass, damping, stiffness } = system;
  const n = mass.length;
  const minvK = solveMatrixColumns(mass, stiffness);
  const minvC = solveMatrixColumns(mass, damping);
  const out = zeros(2 * n);
  const eye = identity(n);

  for (let i = 0; i < n; i += 1) {
    for (let j = 0; j < n; j += 1) {
      out[i][j + n] = eye[i][j];
      out[i + n][j] = -minvK[i][j];
      out[i + n][j + n] = -minvC[i][j];
    }
  }
  return out;
}
