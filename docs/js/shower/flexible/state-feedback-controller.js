import {
  actuatorBoundaryTrajectory,
  clampHandTarget,
  createHandActuatorState,
  DEFAULT_HAND_ACTUATOR_LIMITS,
  stepHandActuator,
} from "./hand-actuator.js";
import {
  stepRodWithHandBoundaryRK4,
} from "./nonlinear-boundary.js";
import {
  identity,
  multiplyMatrices,
  solveLinear,
  transpose,
  zeros,
} from "./linear-algebra.js";

export const DEFAULT_LQR_DESIGN_DT_S = 0.002;

function assertFiniteVector(vector, label) {
  if (
    !Array.isArray(vector)
    || !vector.every(Number.isFinite)
  ) {
    throw new RangeError(`${label} must be a finite vector`);
  }
}

function addMatrices(a, b) {
  if (
    a.length !== b.length
    || a[0].length !== b[0].length
  ) {
    throw new RangeError("matrix shape mismatch");
  }
  return a.map((row, i) => row.map(
    (value, j) => value + b[i][j],
  ));
}

function subtractMatrices(a, b) {
  if (
    a.length !== b.length
    || a[0].length !== b[0].length
  ) {
    throw new RangeError("matrix shape mismatch");
  }
  return a.map((row, i) => row.map(
    (value, j) => value - b[i][j],
  ));
}

function scaleMatrix(matrix, scale) {
  return matrix.map((row) => row.map(
    (value) => scale * value,
  ));
}

function symmetrize(matrix) {
  return matrix.map((row, i) => row.map(
    (value, j) => 0.5 * (value + matrix[j][i]),
  ));
}

function frobeniusNorm(matrix) {
  let sumSq = 0;
  for (const row of matrix) {
    for (const value of row) sumSq += value * value;
  }
  return Math.sqrt(sumSq);
}

function matrixMaxAbs(matrix) {
  let largest = 0;
  for (const row of matrix) {
    for (const value of row) {
      largest = Math.max(largest, Math.abs(value));
    }
  }
  return largest;
}

function solveMatrix(left, right) {
  if (left.length !== left[0].length) {
    throw new RangeError("left matrix must be square");
  }
  if (right.length !== left.length) {
    throw new RangeError("matrix shape mismatch");
  }
  const columns = right[0].length;
  const output = zeros(left.length, columns);
  for (let col = 0; col < columns; col += 1) {
    const rhs = right.map((row) => row[col]);
    const solution = solveLinear(left, rhs);
    for (let row = 0; row < solution.length; row += 1) {
      output[row][col] = solution[row];
    }
  }
  return output;
}

function matVec(matrix, vector) {
  return matrix.map((row) => row.reduce(
    (sum, value, index) => sum + value * vector[index],
    0,
  ));
}

function vectorDifference(a, b) {
  return a.map((value, i) => value - b[i]);
}

function vectorNorm(vector) {
  return Math.sqrt(vector.reduce(
    (sum, value) => sum + value * value,
    0,
  ));
}

function column(matrix, index) {
  return matrix.map((row) => row[index]);
}

function columnMatrix(columns) {
  if (columns.length === 0) return [];
  const rows = columns[0].length;
  return Array.from(
    { length: rows },
    (_, row) => columns.map((values) => values[row]),
  );
}

function multiplyMatrixVector(matrix, vector) {
  return matrix.map((row) => row.reduce(
    (sum, value, i) => sum + value * vector[i],
    0,
  ));
}

function loadOptions(scenario) {
  return {
    tipLoad: scenario.tipLoad,
    additionalGeneralizedForce: scenario.flowForce,
    additionalCartesianResultant: scenario.flowResultant,
  };
}

export function createFullStateDescriptor(
  system,
  equilibriumAnglesRad,
) {
  const segmentCount = system.params.segmentCount;
  if (
    !Array.isArray(equilibriumAnglesRad)
    || equilibriumAnglesRad.length !== segmentCount
  ) {
    throw new RangeError(
      "equilibriumAnglesRad must match segmentCount",
    );
  }

  const stateNames = [];
  const scales = [];

  for (let i = 1; i < segmentCount; i += 1) {
    stateNames.push(`dtheta_${i}`);
    scales.push(1e-6);
  }
  for (let i = 1; i < segmentCount; i += 1) {
    stateNames.push(`domega_${i}`);
    scales.push(1e-5);
  }

  stateNames.push(
    "hand_x",
    "hand_vx",
    "hand_theta",
    "hand_omega",
  );
  scales.push(1e-6, 1e-5, 1e-6, 1e-5);

  return {
    segmentCount,
    freeCount: segmentCount - 1,
    dimension: 2 * (segmentCount - 1) + 4,
    inputDimension: 2,
    stateNames,
    stateFiniteDifferenceScales: scales,
    inputFiniteDifferenceScales: [1e-5, 1e-5],
    equilibriumAnglesRad: [...equilibriumAnglesRad],
  };
}

export function encodeFullStateDeviation(
  descriptor,
  rodState,
  actuatorStateInput,
) {
  const actuator = createHandActuatorState(
    actuatorStateInput,
  );
  const {
    segmentCount,
    equilibriumAnglesRad,
  } = descriptor;

  if (
    rodState.anglesRad.length !== segmentCount
    || rodState.angularRatesRadS.length !== segmentCount
  ) {
    throw new RangeError("rod state does not match descriptor");
  }

  const vector = [];
  for (let i = 1; i < segmentCount; i += 1) {
    vector.push(
      rodState.anglesRad[i] - equilibriumAnglesRad[i],
    );
  }
  for (let i = 1; i < segmentCount; i += 1) {
    vector.push(rodState.angularRatesRadS[i]);
  }

  vector.push(
    actuator.lateralPositionM,
    actuator.lateralVelocityMps,
    actuator.angleRad,
    actuator.angularRateRadS,
  );
  return vector;
}

export function decodeFullStateDeviation(
  descriptor,
  vector,
) {
  assertFiniteVector(vector, "state vector");
  if (vector.length !== descriptor.dimension) {
    throw new RangeError("state vector dimension mismatch");
  }

  const {
    segmentCount,
    freeCount,
    equilibriumAnglesRad,
  } = descriptor;
  const anglesRad = [...equilibriumAnglesRad];
  const angularRatesRadS = Array(segmentCount).fill(0);

  for (let i = 0; i < freeCount; i += 1) {
    anglesRad[i + 1] += vector[i];
    angularRatesRadS[i + 1] = vector[freeCount + i];
  }

  const offset = 2 * freeCount;
  const actuatorState = createHandActuatorState({
    lateralPositionM: vector[offset],
    lateralVelocityMps: vector[offset + 1],
    angleRad: vector[offset + 2],
    angularRateRadS: vector[offset + 3],
  });

  anglesRad[0] = actuatorState.angleRad;
  angularRatesRadS[0] = actuatorState.angularRateRadS;

  return {
    rodState: { anglesRad, angularRatesRadS },
    actuatorState,
  };
}

export function fullStateOneStep(
  scenario,
  descriptor,
  stateVector,
  inputVector,
  {
    dt = DEFAULT_LQR_DESIGN_DT_S,
    limits = DEFAULT_HAND_ACTUATOR_LIMITS,
  } = {},
) {
  if (!(dt > 0)) throw new RangeError("dt must be positive");
  assertFiniteVector(inputVector, "input vector");
  if (inputVector.length !== 2) {
    throw new RangeError("input vector must have dimension 2");
  }

  const decoded = decodeFullStateDeviation(
    descriptor,
    stateVector,
  );
  const target = clampHandTarget(
    {
      lateralPositionM: inputVector[0],
      angleRad: inputVector[1],
    },
    limits,
  );

  const actuatorStep = stepHandActuator(
    decoded.actuatorState,
    target,
    dt,
    limits,
  );
  const trajectory = actuatorBoundaryTrajectory(
    decoded.actuatorState,
    actuatorStep,
    dt,
  );
  const result = stepRodWithHandBoundaryRK4(
    scenario.system,
    decoded.rodState,
    0,
    dt,
    trajectory,
    loadOptions(scenario),
  );

  return {
    stateVector: encodeFullStateDeviation(
      descriptor,
      result.state,
      actuatorStep.state,
    ),
    rodState: result.state,
    actuatorState: actuatorStep.state,
    boundary: result.boundary,
    diagnostics: result.diagnostics,
    saturation: actuatorStep.saturation,
    target,
  };
}

export function linearizeFullStateOneStep(
  scenario,
  equilibrium,
  {
    dt = DEFAULT_LQR_DESIGN_DT_S,
    limits = DEFAULT_HAND_ACTUATOR_LIMITS,
    stateScaleMultiplier = 1,
    inputScaleMultiplier = 1,
  } = {},
) {
  const descriptor = createFullStateDescriptor(
    scenario.system,
    equilibrium.anglesRad,
  );
  const n = descriptor.dimension;
  const m = descriptor.inputDimension;
  const zeroState = Array(n).fill(0);
  const zeroInput = Array(m).fill(0);

  const base = fullStateOneStep(
    scenario,
    descriptor,
    zeroState,
    zeroInput,
    { dt, limits },
  ).stateVector;

  const a = zeros(n, n);
  for (let col = 0; col < n; col += 1) {
    const epsilon =
      descriptor.stateFiniteDifferenceScales[col]
      * stateScaleMultiplier;
    const plus = [...zeroState];
    const minus = [...zeroState];
    plus[col] += epsilon;
    minus[col] -= epsilon;

    const fPlus = fullStateOneStep(
      scenario,
      descriptor,
      plus,
      zeroInput,
      { dt, limits },
    ).stateVector;
    const fMinus = fullStateOneStep(
      scenario,
      descriptor,
      minus,
      zeroInput,
      { dt, limits },
    ).stateVector;

    for (let row = 0; row < n; row += 1) {
      a[row][col] = (
        fPlus[row] - fMinus[row]
      ) / (2 * epsilon);
    }
  }

  const b = zeros(n, m);
  for (let col = 0; col < m; col += 1) {
    const epsilon =
      descriptor.inputFiniteDifferenceScales[col]
      * inputScaleMultiplier;
    const plus = [...zeroInput];
    const minus = [...zeroInput];
    plus[col] += epsilon;
    minus[col] -= epsilon;

    const fPlus = fullStateOneStep(
      scenario,
      descriptor,
      zeroState,
      plus,
      { dt, limits },
    ).stateVector;
    const fMinus = fullStateOneStep(
      scenario,
      descriptor,
      zeroState,
      minus,
      { dt, limits },
    ).stateVector;

    for (let row = 0; row < n; row += 1) {
      b[row][col] = (
        fPlus[row] - fMinus[row]
      ) / (2 * epsilon);
    }
  }

  return {
    descriptor,
    A: a,
    B: b,
    affineResidual: base,
    residualNorm: vectorNorm(base),
    dt,
    limits,
  };
}

export function continuousApproximation(realization) {
  const n = realization.A.length;
  const i = identity(n);
  return {
    A: scaleMatrix(
      subtractMatrices(realization.A, i),
      1 / realization.dt,
    ),
    B: scaleMatrix(realization.B, 1 / realization.dt),
    dt: realization.dt,
  };
}

export function controllabilityMatrix(a, b) {
  const n = a.length;
  let block = b.map((row) => [...row]);
  const blocks = [];

  for (let power = 0; power < n; power += 1) {
    blocks.push(block);
    block = multiplyMatrices(a, block);
  }

  const columns = [];
  for (const matrix of blocks) {
    for (let col = 0; col < matrix[0].length; col += 1) {
      columns.push(column(matrix, col));
    }
  }
  return columnMatrix(columns);
}

export function numericalColumnRank(
  matrix,
  {
    relativeTolerance = 1e-8,
    rowIndices = null,
  } = {},
) {
  if (matrix.length === 0 || matrix[0].length === 0) {
    return 0;
  }
  const working = rowIndices
    ? rowIndices.map((row) => [...matrix[row]])
    : matrix.map((row) => [...row]);

  const rowCount = working.length;
  const colCount = working[0].length;
  const basis = [];
  let largestOriginalNorm = 0;

  for (let colIndex = 0; colIndex < colCount; colIndex += 1) {
    const original = Array.from(
      { length: rowCount },
      (_, row) => working[row][colIndex],
    );
    const originalNorm = vectorNorm(original);
    largestOriginalNorm = Math.max(
      largestOriginalNorm,
      originalNorm,
    );
    if (!(originalNorm > 0)) continue;

    let v = original.map((value) => value / originalNorm);
    for (const q of basis) {
      const projection = q.reduce(
        (sum, value, i) => sum + value * v[i],
        0,
      );
      v = v.map(
        (value, i) => value - projection * q[i],
      );
    }
    // Re-orthogonalize once for long controllability matrices.
    for (const q of basis) {
      const projection = q.reduce(
        (sum, value, i) => sum + value * v[i],
        0,
      );
      v = v.map(
        (value, i) => value - projection * q[i],
      );
    }

    const residualNorm = vectorNorm(v);
    const scaledTolerance = relativeTolerance * Math.max(
      1,
      largestOriginalNorm / originalNorm,
    );
    if (residualNorm > scaledTolerance) {
      basis.push(v.map(
        (value) => value / residualNorm,
      ));
      if (basis.length === rowCount) break;
    }
  }

  return basis.length;
}

export function controllabilityDiagnostics(
  realization,
  options = {},
) {
  const matrix = controllabilityMatrix(
    realization.A,
    realization.B,
  );
  const n = realization.descriptor.dimension;
  const freeCount = realization.descriptor.freeCount;
  const rodRows = Array.from(
    { length: 2 * freeCount },
    (_, index) => index,
  );

  return {
    rank: numericalColumnRank(matrix, options),
    rodRank: numericalColumnRank(
      matrix,
      { ...options, rowIndices: rodRows },
    ),
    dimension: n,
    rodDimension: rodRows.length,
    inputDimension: realization.B[0].length,
  };
}

export function defaultLqrWeights(descriptor) {
  const n = descriptor.dimension;
  const q = zeros(n, n);
  const freeCount = descriptor.freeCount;

  for (let i = 0; i < freeCount; i += 1) {
    const x = (i + 1) / freeCount;
    q[i][i] = 55 + 95 * x * x;
    q[freeCount + i][freeCount + i] = 0.8 + 1.7 * x * x;
  }

  const offset = 2 * freeCount;
  q[offset][offset] = 35;
  q[offset + 1][offset + 1] = 0.45;
  q[offset + 2][offset + 2] = 18;
  q[offset + 3][offset + 3] = 0.32;

  return {
    Q: q,
    R: [
      [85, 0],
      [0, 24],
    ],
  };
}

export function solveDiscreteLqr(
  a,
  b,
  q,
  r,
  {
    tolerance = 1e-10,
    maxIterations = 5000,
  } = {},
) {
  const at = transpose(a);
  const bt = transpose(b);
  let p = q.map((row) => [...row]);
  let gain = zeros(b[0].length, a.length);
  let converged = false;
  let relativeChange = Infinity;
  let iteration = 0;

  for (; iteration < maxIterations; iteration += 1) {
    const btP = multiplyMatrices(bt, p);
    const s = addMatrices(
      r,
      multiplyMatrices(btP, b),
    );
    const btPA = multiplyMatrices(btP, a);
    gain = solveMatrix(s, btPA);

    const atPA = multiplyMatrices(
      multiplyMatrices(at, p),
      a,
    );
    const atPB = multiplyMatrices(
      multiplyMatrices(at, p),
      b,
    );
    const pNext = symmetrize(addMatrices(
      q,
      subtractMatrices(
        atPA,
        multiplyMatrices(atPB, gain),
      ),
    ));

    const delta = frobeniusNorm(
      subtractMatrices(pNext, p),
    );
    relativeChange = delta / Math.max(
      1,
      frobeniusNorm(pNext),
    );
    p = pNext;

    if (relativeChange <= tolerance) {
      converged = true;
      break;
    }
  }

  if (!converged) {
    throw new Error(
      `DARE iteration did not converge; relative change=${relativeChange}`,
    );
  }

  const btP = multiplyMatrices(bt, p);
  gain = solveMatrix(
    addMatrices(r, multiplyMatrices(btP, b)),
    multiplyMatrices(btP, a),
  );

  return {
    K: gain,
    P: p,
    iterations: iteration + 1,
    relativeChange,
    maxAbsGain: matrixMaxAbs(gain),
  };
}

export function designFullStateLqr(
  scenario,
  equilibrium,
  {
    dt = DEFAULT_LQR_DESIGN_DT_S,
    limits = DEFAULT_HAND_ACTUATOR_LIMITS,
    weights = null,
    linearizationOptions = {},
    lqrOptions = {},
  } = {},
) {
  const realization = linearizeFullStateOneStep(
    scenario,
    equilibrium,
    {
      dt,
      limits,
      ...linearizationOptions,
    },
  );
  const diagnostics = controllabilityDiagnostics(
    realization,
  );
  const selectedWeights = weights
    ?? defaultLqrWeights(realization.descriptor);
  const lqr = solveDiscreteLqr(
    realization.A,
    realization.B,
    selectedWeights.Q,
    selectedWeights.R,
    lqrOptions,
  );
  const continuous = continuousApproximation(realization);

  return {
    realization,
    controllability: diagnostics,
    weights: selectedWeights,
    lqr,
    continuous,
  };
}

export function lqrStateVector(
  design,
  rodState,
  actuatorState,
) {
  return encodeFullStateDeviation(
    design.realization.descriptor,
    rodState,
    actuatorState,
  );
}

export function lqrHandTarget(
  design,
  rodState,
  actuatorState,
  limits = DEFAULT_HAND_ACTUATOR_LIMITS,
) {
  const stateVector = lqrStateVector(
    design,
    rodState,
    actuatorState,
  );
  const raw = multiplyMatrixVector(
    design.lqr.K,
    stateVector,
  ).map((value) => -value);

  return {
    target: clampHandTarget(
      {
        lateralPositionM: raw[0],
        angleRad: raw[1],
      },
      limits,
    ),
    rawTarget: {
      lateralPositionM: raw[0],
      angleRad: raw[1],
    },
    stateVector,
    stateNorm: vectorNorm(stateVector),
  };
}

export function linearOneStepPrediction(
  realization,
  stateVector,
  inputVector,
) {
  const ax = multiplyMatrixVector(
    realization.A,
    stateVector,
  );
  const bu = multiplyMatrixVector(
    realization.B,
    inputVector,
  );
  return ax.map((value, i) => value + bu[i]);
}

export function realizationDifferenceMetrics(a, b) {
  if (
    a.A.length !== b.A.length
    || a.B.length !== b.B.length
  ) {
    throw new RangeError("realization shape mismatch");
  }

  const aDiff = subtractMatrices(a.A, b.A);
  const bDiff = subtractMatrices(a.B, b.B);
  return {
    aRelative: frobeniusNorm(aDiff)
      / Math.max(1e-12, frobeniusNorm(a.A)),
    bRelative: frobeniusNorm(bDiff)
      / Math.max(1e-12, frobeniusNorm(a.B)),
    aMaxAbs: matrixMaxAbs(aDiff),
    bMaxAbs: matrixMaxAbs(bDiff),
  };
}
