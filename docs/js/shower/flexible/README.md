# Phase 1 flexible hose — model contract (H1-0)

This document freezes the first 2D flexible-hose model before any interactive
rendering is added.

The goal is not CFD. The goal is the lowest-order distributed model that can
distinguish a dry/stable flexible hose from a hose whose internal flow changes
modal stability.

## 1. Reference geometry and signs

Use a straight reference centerline with material coordinate

```text
s in [0, L]
```

measured from the hand/base (`s = 0`) to the shower head (`s = L`).

The first model is planar. Its transverse displacement is

```text
y = y(s, t)
```

and positive rotation is

```text
theta = partial y / partial s
```

under the small-slope Euler–Bernoulli assumption.

This phase intentionally does **not** yet model the large 3D hanging shape.
Gravity, large rotations, torsion and geometric nonlinearity are deferred until
the linear conveying-flow mechanism is validated.

## 2. Continuum equation selected for H1-2

For a uniform Euler–Bernoulli hose conveying fluid at constant mean speed
`U`, the reference linear equation is

```text
(m_s + m_f) y_tt
+ c y_t
+ EI y_ssss
+ 2 m_f U y_st
+ m_f U^2 y_ss
= f_ext
```

where

- `m_s`: structural mass per unit length
- `m_f = rho A_hose`: internal-fluid mass per unit length
- `EI`: bending rigidity
- `c`: effective distributed damping
- `Q`: volume flow rate
- `U = Q / A_hose`: mean internal-flow speed

The `2 m_f U y_st` term is odd in `U`; the `m_f U^2 y_ss` term is even
in `U`. Those parity properties are mandatory H1-2 regression checks.

This is the standard small-deflection conveying-fluid beam structure: bending,
combined inertia, a Coriolis-type velocity coupling, and a flow-speed-squared
term.

## 3. H1-0 Galerkin convention

For one 2-node Euler–Bernoulli element of length `l_e`, use the Hermite field

```text
y_e(s,t) = N(s) q_e(t)

q_e = [y_1, theta_1, y_2, theta_2]^T
```

with normalized coordinate `xi = s / l_e` and

```text
N1 = 1 - 3 xi^2 + 2 xi^3
N2 = l_e (xi - 2 xi^2 + xi^3)
N3 = 3 xi^2 - 2 xi^3
N4 = l_e (-xi^2 + xi^3)
```

The element equations are kept as physically separate terms:

```text
(M_s,e + M_f,e) q_ddot
+ (C_struct,e + G_flow,e) q_dot
+ (K_bend,e + K_flow,e) q
= f_e
```

with

```text
M_s,e = m_s integral(N^T N ds)
M_f,e = m_f integral(N^T N ds)

K_bend,e = EI integral(N_ss^T N_ss ds)

G_flow,e = 2 m_f U integral(N^T N_s ds)

K_flow,e = m_f U^2 integral(N^T N_ss ds)
```

For the flow terms, H1-2 will use this **strong-form Galerkin volume
convention**. The `U` and `U^2` terms are not integrated by parts into an
extra hand-written follower force.

That choice is deliberate: it prevents adding the same outlet-momentum effect
once through the conveying-flow equation and again as an ad-hoc tip force.

If H1-2 later changes to a weak form with explicit outlet boundary terms, the
volume matrices and boundary residual must be changed together and the
parity/stability regressions must be updated.

## 4. Closed-form dry element matrices

For H1-1, fluid terms are disabled and only the structural consistent-mass and
bending-stiffness matrices are active.

```text
M_e = (m_s l_e / 420) *
[
  [156,      22 l_e,   54,      -13 l_e],
  [22 l_e,   4 l_e^2,  13 l_e,  -3 l_e^2],
  [54,       13 l_e,  156,      -22 l_e],
  [-13 l_e, -3 l_e^2, -22 l_e,   4 l_e^2]
]
```

```text
K_e = (EI / l_e^3) *
[
  [12,       6 l_e,   -12,       6 l_e],
  [6 l_e,    4 l_e^2, -6 l_e,    2 l_e^2],
  [-12,     -6 l_e,    12,      -6 l_e],
  [6 l_e,    2 l_e^2, -6 l_e,    4 l_e^2]
]
```

These matrices are implemented directly in `beam-element.js`.

## 5. Boundary conditions

### H1-1 dry validation boundary

The first structural validation problem is a clamped-free beam:

```text
y(0,t) = 0
theta(0,t) = 0
```

The two base DOFs are eliminated from the assembled system.

The free end carries optional lumped shower-head inertia:

```text
M_tip[y_L, y_L] += m_tip
M_tip[theta_L, theta_L] += J_tip
```

No tip spring is added in H1-1.

### Later controlled boundary

The actual game will replace the fixed values at the base with prescribed
`y_base(t)` and `theta_base(t)`. That is a boundary-control extension of the
same FEM model, not a separate fake force.

## 6. Damping convention

H1-1 uses Rayleigh damping only as a numerical/educational structural damping
model:

```text
C_struct = alpha_M M + beta_K K
```

The coefficients are explicit parameters.

Rayleigh damping is not claimed to be a measured material law for a real
shower hose.

## 7. Natural-frequency validation

With zero tip mass and zero damping, the clamped-free dry Euler–Bernoulli beam
has analytical frequencies

```text
omega_n = beta_n^2 sqrt(EI / (m_s L^4))
```

where the first root satisfies

```text
cos(beta_1) cosh(beta_1) = -1
beta_1 ~= 1.875104068711961
```

H1-1 must converge to this first frequency under mesh refinement.

Adding a positive tip mass must lower the first natural frequency.

## 8. Time integration for H1-1

H1-1 uses the average-acceleration Newmark method:

```text
beta = 1/4
gamma = 1/2
```

for

```text
M q_ddot + C q_dot + K q = f(t)
```

This replaces the Phase 0 semi-implicit Euler choice for the flexible system.

Required behavior:

- undamped dry system: mechanical energy remains nearly constant
- damped dry system: mechanical energy decays
- physics `dt` is independent of rendering fps

The integrator remains replaceable before H1-2 if the non-conservative
conveying-flow system requires a different numerical treatment.

## 9. Initial educational parameters

H1-1 defaults are numerical teaching parameters, not a product
identification:

```text
L = 1.2 m
EI = 0.7 N m^2
m_s = 0.25 kg/m
m_tip = 0.20 kg
J_tip = 0.002 kg m^2
elements = 8
```

The exact values will be revisited after H1-2 establishes a usable
below/near/above-critical flow sweep.

## 10. H1-2 stability definition and validated crossing

After flow terms are assembled, define the first-order state system from

```text
M q_ddot + C(U) q_dot + K(U) q = 0
```

and inspect its eigenvalues `lambda(U)`.

The project definition is

```text
max Re(lambda) < -eps  : stable
|max Re(lambda)| <= eps: near critical
max Re(lambda) > +eps  : unstable / growing mode
```

The critical flow is a model result found by a flow-speed sweep/root search. It
must not be hard-coded as a universal shower-hose number.

H1-2 now implements that calculation. The browser-side core assembles the
physical matrices and first-order state matrix in plain JavaScript; CI uses a
general real non-symmetric eigensolver only to inspect the resulting complex
eigenvalues.

For the current educational defaults

```text
L = 1.2 m
EI = 0.7 N m^2
m_s = 0.25 kg/m
inner diameter = 6 mm
rho = 997 kg/m^3
m_tip = 0.20 kg
J_tip = 0.002 kg m^2
elements = 8
```

CI finds

```text
U_cr ~= 9.4808 m/s
Q_cr ~= 16.08 L/min
Im(lambda_cr) ~= 14.797 rad/s
```

The nonzero imaginary part at the crossing is the regression guard that the
default loss of stability is oscillatory flutter rather than a static
divergence in this discretized model.

These values are **not** measurements of a real shower hose. They are
model-dependent educational values and will change when geometry, stiffness,
tip-head dynamics, gravity, or nonlinear effects are revised.

## 11. H1-2 discrete flow terms

H1-2 is implemented in `conveying-flow.js`.

For each Hermite element it keeps three fluid contributions separate:

```text
M_fluid
G_flow(U) = 2 m_f U integral(N^T N_s ds)
K_flow(U^2) = m_f U^2 integral(N^T N_ss ds)
```

The assembled equation is

```text
(M_struct + M_fluid + M_tip) q_ddot
+ (C_struct + G_flow) q_dot
+ (K_bend + K_flow) q = 0
```

A stationary **filled** hose at `U = 0` still has `M_fluid`. What vanishes
at zero flow are `G_flow` and `K_flow`. This distinction is part of the
regression suite.

Required parity checks are now executable:

```text
G_flow(-U) = -G_flow(U)
K_flow((-U)^2) = K_flow(U^2)
```

The effective matrices are non-symmetric, so H1-2 adds an LU-based linear
solve and a general Newmark average-acceleration path rather than incorrectly
using the H1-1 Cholesky-only path.

CI also verifies:

- low-flow eigenvalues have negative maximum real part
- a bisection search finds the zero crossing
- 4/6/8-element critical speeds are converged to the documented tolerance
- an above-critical time history grows without adding fake animation forcing
- halving the physics time step changes the selected RMS result by less than
  the regression tolerance

## 12. H1-0 / H1-1 / H1-2 scope boundary

H1-0 fixes the conventions above.

H1-1 implements only:

- dry Euler–Bernoulli element matrices
- global assembly
- clamped-base reduction
- tip mass / tip rotational inertia
- Rayleigh damping
- dry natural frequencies
- Newmark time integration

H1-1 by itself does **not** claim garden-hose flutter.

H1-2 has now demonstrated a flow-dependent oscillatory stability crossing and
a matching growing time-domain response for the current linear reference
model. The next step, H1-3, is to make the shower-head end condition and outlet
momentum bookkeeping more representative before exposing the model as the
interactive Phase 1 game.
