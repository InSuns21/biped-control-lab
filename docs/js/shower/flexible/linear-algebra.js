export function zeros(rows, cols = rows) {
  return Array.from({ length: rows }, () => Array(cols).fill(0));
}

export function identity(size) {
  const out = zeros(size);
  for (let i = 0; i < size; i += 1) out[i][i] = 1;
  return out;
}

export function transpose(matrix) {
  const rows = matrix.length;
  const cols = matrix[0].length;
  const out = zeros(cols, rows);
  for (let i = 0; i < rows; i += 1) {
    for (let j = 0; j < cols; j += 1) out[j][i] = matrix[i][j];
  }
  return out;
}

export function multiplyMatrices(a, b) {
  const rows = a.length;
  const inner = a[0].length;
  const cols = b[0].length;
  if (b.length !== inner) throw new RangeError("matrix shape mismatch");

  const out = zeros(rows, cols);
  for (let i = 0; i < rows; i += 1) {
    for (let k = 0; k < inner; k += 1) {
      const aik = a[i][k];
      if (aik === 0) continue;
      for (let j = 0; j < cols; j += 1) out[i][j] += aik * b[k][j];
    }
  }
  return out;
}

export function matVec(matrix, vector) {
  if (matrix[0].length !== vector.length) {
    throw new RangeError("matrix/vector shape mismatch");
  }
  return matrix.map((row) => row.reduce(
    (sum, value, i) => sum + value * vector[i],
    0,
  ));
}

export function dot(a, b) {
  if (a.length !== b.length) throw new RangeError("vector shape mismatch");
  return a.reduce((sum, value, i) => sum + value * b[i], 0);
}

export function matrixLinearCombination(terms) {
  if (terms.length === 0) throw new RangeError("at least one matrix is required");
  const rows = terms[0].matrix.length;
  const cols = terms[0].matrix[0].length;
  const out = zeros(rows, cols);

  for (const { scale, matrix } of terms) {
    if (matrix.length !== rows || matrix[0].length !== cols) {
      throw new RangeError("matrix shape mismatch");
    }
    for (let i = 0; i < rows; i += 1) {
      for (let j = 0; j < cols; j += 1) {
        out[i][j] += scale * matrix[i][j];
      }
    }
  }
  return out;
}

export function choleskyFactor(matrix) {
  const n = matrix.length;
  const lower = zeros(n);

  for (let i = 0; i < n; i += 1) {
    for (let j = 0; j <= i; j += 1) {
      let sum = matrix[i][j];
      for (let k = 0; k < j; k += 1) {
        sum -= lower[i][k] * lower[j][k];
      }

      if (i === j) {
        if (!(sum > 0)) {
          throw new RangeError("matrix is not positive definite");
        }
        lower[i][j] = Math.sqrt(sum);
      } else {
        lower[i][j] = sum / lower[j][j];
      }
    }
  }

  return lower;
}

export function solveWithCholeskyFactor(lower, rhs) {
  const n = lower.length;
  if (rhs.length !== n) throw new RangeError("rhs shape mismatch");

  const y = Array(n).fill(0);
  for (let i = 0; i < n; i += 1) {
    let sum = rhs[i];
    for (let j = 0; j < i; j += 1) sum -= lower[i][j] * y[j];
    y[i] = sum / lower[i][i];
  }

  const x = Array(n).fill(0);
  for (let i = n - 1; i >= 0; i -= 1) {
    let sum = y[i];
    for (let j = i + 1; j < n; j += 1) sum -= lower[j][i] * x[j];
    x[i] = sum / lower[i][i];
  }

  return x;
}

export function solveSpd(matrix, rhs) {
  return solveWithCholeskyFactor(choleskyFactor(matrix), rhs);
}

export function invertLowerTriangular(lower) {
  const n = lower.length;
  const inverse = zeros(n);

  for (let col = 0; col < n; col += 1) {
    const rhs = Array(n).fill(0);
    rhs[col] = 1;

    for (let i = 0; i < n; i += 1) {
      let sum = rhs[i];
      for (let j = 0; j < i; j += 1) {
        sum -= lower[i][j] * inverse[j][col];
      }
      inverse[i][col] = sum / lower[i][i];
    }
  }

  return inverse;
}

export function jacobiEigenvaluesSymmetric(
  matrix,
  { tolerance = 1e-12, maxIterations = null } = {},
) {
  const n = matrix.length;
  const a = matrix.map((row) => [...row]);
  const limit = maxIterations ?? Math.max(50, 100 * n * n);

  for (let iteration = 0; iteration < limit; iteration += 1) {
    let p = 0;
    let q = 1;
    let largest = 0;

    for (let i = 0; i < n; i += 1) {
      for (let j = i + 1; j < n; j += 1) {
        const value = Math.abs(a[i][j]);
        if (value > largest) {
          largest = value;
          p = i;
          q = j;
        }
      }
    }

    if (largest <= tolerance) {
      return a.map((row, i) => row[i]).sort((x, y) => x - y);
    }

    const app = a[p][p];
    const aqq = a[q][q];
    const apq = a[p][q];
    const angle = 0.5 * Math.atan2(2 * apq, aqq - app);
    const c = Math.cos(angle);
    const s = Math.sin(angle);

    for (let k = 0; k < n; k += 1) {
      if (k === p || k === q) continue;
      const akp = a[k][p];
      const akq = a[k][q];
      const newKp = c * akp - s * akq;
      const newKq = s * akp + c * akq;
      a[k][p] = newKp;
      a[p][k] = newKp;
      a[k][q] = newKq;
      a[q][k] = newKq;
    }

    a[p][p] = c * c * app - 2 * s * c * apq + s * s * aqq;
    a[q][q] = s * s * app + 2 * s * c * apq + c * c * aqq;
    a[p][q] = 0;
    a[q][p] = 0;
  }

  throw new Error("Jacobi eigenvalue iteration did not converge");
}

export function generalizedSymmetricEigenvalues(stiffness, mass) {
  const lower = choleskyFactor(mass);
  const lowerInverse = invertLowerTriangular(lower);
  const transformed = multiplyMatrices(
    multiplyMatrices(lowerInverse, stiffness),
    transpose(lowerInverse),
  );

  // Remove tiny floating-point asymmetry before the symmetric eigensolve.
  const symmetric = transformed.map((row, i) => row.map(
    (value, j) => 0.5 * (value + transformed[j][i]),
  ));

  return jacobiEigenvaluesSymmetric(symmetric);
}

export function quadraticEnergy(matrix, vector) {
  return 0.5 * dot(vector, matVec(matrix, vector));
}


export function luFactor(matrix) {
  const n = matrix.length;
  if (!matrix.every((row) => row.length === n)) {
    throw new RangeError("LU factorization requires a square matrix");
  }

  const lu = matrix.map((row) => [...row]);
  const pivots = Array.from({ length: n }, (_, i) => i);

  for (let k = 0; k < n; k += 1) {
    let pivotRow = k;
    let pivotAbs = Math.abs(lu[k][k]);
    for (let i = k + 1; i < n; i += 1) {
      const candidate = Math.abs(lu[i][k]);
      if (candidate > pivotAbs) {
        pivotAbs = candidate;
        pivotRow = i;
      }
    }

    if (!(pivotAbs > 1e-14)) {
      throw new RangeError("matrix is singular to working precision");
    }

    if (pivotRow !== k) {
      [lu[k], lu[pivotRow]] = [lu[pivotRow], lu[k]];
      [pivots[k], pivots[pivotRow]] = [pivots[pivotRow], pivots[k]];
    }

    for (let i = k + 1; i < n; i += 1) {
      lu[i][k] /= lu[k][k];
      for (let j = k + 1; j < n; j += 1) {
        lu[i][j] -= lu[i][k] * lu[k][j];
      }
    }
  }

  return { lu, pivots };
}

export function solveWithLuFactor(factor, rhs) {
  const { lu, pivots } = factor;
  const n = lu.length;
  if (rhs.length !== n) throw new RangeError("rhs shape mismatch");

  const x = pivots.map((sourceIndex) => rhs[sourceIndex]);

  for (let i = 0; i < n; i += 1) {
    for (let j = 0; j < i; j += 1) {
      x[i] -= lu[i][j] * x[j];
    }
  }

  for (let i = n - 1; i >= 0; i -= 1) {
    for (let j = i + 1; j < n; j += 1) {
      x[i] -= lu[i][j] * x[j];
    }
    x[i] /= lu[i][i];
  }

  return x;
}

export function solveLinear(matrix, rhs) {
  return solveWithLuFactor(luFactor(matrix), rhs);
}
