# H1-4A fast-onset calibration report

## Status

H1-4A is complete.

This is a **qualitative calibration / model-discrimination exercise**, not a
system identification of a commercial shower hose. The reference video/article
does not provide the hose stiffness, damping, inner diameter, flow rate, head
inertia, or a measured onset time.

The working visual target used for H1-4A was:

> can clearly growing motion appear in roughly 1--3 seconds without the current
> small-deflection model already being outside its own validity guard?

## Metrics

For each case the calibration records:

- `sigma = max Re(lambda)`
- predicted 3x / 5x modal growth times
- static-equilibrium displacement / rotation
- time-domain dynamic RMS
- time to the H1-4A visible-onset threshold
- time to the small-deflection guard

The current guard is an engineering/modeling guard, not a universal theorem:

```text
max dynamic displacement <= 0.18 m
max dynamic rotation     <= 0.35 rad
```

The static equilibrium is checked against the same scale before a case can be
called a valid linear fast-onset result.

## 1. Existing H1-4 / H1-3 reference

At the current H1-4 default high-flow reference:

```text
Q = 18 L/min
EI = 0.7 N m^2
L = 1.2 m
head mass = 0.20 kg

max static displacement ~= 1.079 m
max static rotation     ~= 98.27 deg
sigma                   ~= +0.1000 1/s
predicted 3x time       ~= 10.98 s
```

The important result is not only that the growth is slow. The linearized
static equilibrium is already far outside the intended small-deflection
regime.

Therefore the current high-flow H1-4 picture is useful as a **qualitative
linear extrapolation of the instability mechanism**, but it must not be
described as a quantitatively valid large-amplitude shower-hose shape.

## 2. Parameter-only sensitivity

One-at-a-time sweeps covered:

```text
EI:        0.15 ... 1.2 N m^2
Q:         8 ... 24 L/min
L:         0.8 ... 1.8 m
tip mass:  0.10 ... 0.35 kg
alpha_M:   0 ... 0.15 1/s
beta_K:    0 ... 5e-4 s
```

A small combination sweep was also run.

Parameter changes can make the eigenvalue growth extremely fast. For example,
one explored combination reached approximately

```text
Q = 24 L/min
EI = 0.25 N m^2
L = 1.6 m
sigma ~= +5.71 1/s
predicted 3x time ~= 0.19 s
```

but its static equilibrium was already approximately

```text
0.477 m
58.0 deg
```

and the time-domain trajectory hit the linear guard at about `0.84 s`.

Across the explored grid there was **no growing parameter-only case that also
kept the static equilibrium inside the current small-deflection guard**.

Changing parameters can therefore make the algebraic linear model "fast", but
does not solve the physical-validity problem.

## 3. Initial curvature / modal content

The initial-state sweep varied roughly

```text
tip offset:   8 ... 60 mm
tip rotation: 0 ... 10 deg
```

At the 18 L/min reference the static equilibrium is invalid regardless of the
initial perturbation.

A lower-flow probe at `Q = 5 L/min` has a valid reference equilibrium:

```text
max static displacement ~= 133.88 mm
max static rotation     ~= 10.17 deg
```

Within that valid low-flow case, changing initial curvature / mode content did
not produce a valid 1--3 s self-excited growth event under the H1-4A metric.

Initial curvature changes what modes are visible first, but it does not repair
an insufficient eigenvalue growth rate.

## 4. Movable hand boundary

H1-4A adds a prescribed hand boundary by partitioning the full FEM system:

```text
M_ff qdd_f + C_ff qd_f + K_ff q_f
  = f_f
  - M_fb qdd_b
  - C_fb qd_b
  - K_fb q_b
```

The first input is a single smooth pulse that returns the hand boundary to its
original position/angle. It is not continuing random forcing.

At the valid `Q = 5 L/min` probe, a small hand pulse can create clearly
visible motion quickly. Examples from CI:

```text
10 mm + 5 deg, 0.20 s pulse:
  visible onset ~= 1.0 s
  max dynamic RMS ~= 27.4 mm
  no linear guard hit

10 mm + 5 deg, 0.35 s pulse:
  visible onset ~= 1.0 s
  max dynamic RMS ~= 37.3 mm
  no linear guard hit

10 mm + 10 deg, 0.35 s pulse:
  visible onset ~= 1.0 s
  max dynamic RMS ~= 67.2 mm
  no linear guard hit
```

So a fast visible response **can** be produced in a valid linear regime when
the human hand supplies a transient boundary excitation.

This is different from proving that the water flow alone produces a 1--3 s
self-excited large-amplitude onset.

## 5. Decision

H1-4A records two distinct conclusions.

### Self-excited fast onset

```text
C — current linear small-deflection model is insufficient
```

No explored parameter-only growing case simultaneously satisfied the current
static small-deflection validity check.

### Human / boundary-excited fast onset

```text
B — movable hand boundary can reproduce fast visible motion
```

A small one-shot hand pulse creates 1 s-scale visible motion while a low-flow
reference remains inside the linear guard.

## 6. Consequence for the roadmap

H1-5 is **not** the next fidelity step.

Before claiming that the simulation reproduces the video's high-flow,
large-amplitude whipping, add:

> **H1-4B — geometrically nonlinear flexible hose**

Minimum target:

- finite rotation / finite curvature in 2D
- equilibrium under gravity + outlet momentum without small-angle geometry
- internal-flow coupling carried into the nonlinear configuration
- large-amplitude time-domain integration
- comparison against the H1-4A onset metrics

A 2D geometrically exact beam or planar Cosserat-rod formulation is preferred
over adding visual saturation to the present linear FEM.

The movable hand-boundary machinery from H1-4A is retained and should become
the input model for the later H1-5 game.

## 7. Non-claims

H1-4A does not establish:

- the real video's exact onset time
- the real hose's EI or damping
- the real flow rate
- that the observed motion is purely self-excited flutter
- that the current 2D model captures 3D twisting/contact

It establishes which mechanisms the current model can and cannot reproduce
without violating its own small-deflection assumptions.
