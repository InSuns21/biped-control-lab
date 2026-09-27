# H1-4B geometrically nonlinear hose report

## Status

H1-4B physics and browser visualization are implemented.
Human Visual Audit is still pending after deployment.

H1-4B was introduced because H1-4A found that the linear small-deflection
model could show flutter but its high-flow static equilibrium was already far
outside the linear geometry assumptions.

## Model

The H1-4B core uses an inextensible planar finite-rotation rod.

Each segment has a finite angle. Segment length is preserved exactly and the
centerline is reconstructed geometrically. Bending energy depends on adjacent
angle differences, so a rigid-body rotation does not create artificial bending.

The model includes:

- structural mass
- internal water mass
- finite-rotation gravity
- shower-head tip mass and rotational inertia
- exact-angle bent-head momentum reaction
- finite-angle Coriolis conveying-flow term
- finite-angle U^2 curvature term
- structural damping
- nonlinear static equilibrium
- nonlinear time-domain integration

## Small-amplitude consistency

The dry small-amplitude first mode converges toward the Euler-Bernoulli
cantilever reference:

```text
analytical first mode = 0.65026 Hz
32-segment rod        = 0.67122 Hz
relative error        = 3.22%
```

For the conveying-flow linearization about the straight configuration:

```text
Ucr(8)  = 10.048 m/s
Ucr(12) =  9.836 m/s
Ucr(16) =  9.739 m/s
H1-2 Hermite reference = 9.481 m/s
```

The instability remains oscillatory.

## Finite-rotation equilibrium

A dry 2 N lateral tip-load regression reaches approximately:

```text
tip x     = 0.412 m
tip angle = 29.3 deg
```

without invoking a small-angle geometry.

For the default bent shower head at 18 L/min, the nonlinear scenario gives:

```text
tip x       ~= -0.309 m
tip y       ~=  1.150 m
tip angle   ~= -19.5 deg
max Re(lambda) ~= +0.026 /s
```

This replaces the H1-4 linear extrapolation whose static rotation was about
98 deg.

## Fast-onset result

The default-material 30 L/min sensitivity case reaches the visible-onset
threshold in about:

```text
1.817 s
```

A softer/longer educational sensitivity case:

```text
Q       = 22 L/min
EI      = 0.25 N m^2
L       = 1.5 m
alpha_M = 0.02 1/s
```

reaches onset at about:

```text
1.583 s
```

Refinement:

```text
N=10, dt=.001 -> 1.592 s
N=12, dt=.001 -> 1.583 s
N=16, dt=.001 -> 1.585 s
N=12, dt=.002 -> 1.584 s
```

So the 1--3 s fast-onset time scale can exist in the nonlinear
conveying-flow model as a self-excited response.

## Interpretation

H1-4B changes the H1-4A conclusion in one important way.

H1-4A could only make parameter-only self-excited onset fast by leaving the
small-deflection regime.

H1-4B removes that particular geometric inconsistency. Large rotations are
part of the model rather than an immediate validity failure, and a 1--3 s
self-excited onset is obtained in finite-rotation sensitivity cases.

## Browser visualization

The Phase 1 default view is now H1-4B nonlinear.

It provides:

- finite-rotation centerline
- nonlinear static equilibrium overlay
- shower head
- outlet water direction
- bent-head reaction arrow
- dynamic RMS history
- tip-motion history
- observed onset time
- baseline 18 L/min preset
- fast 22 L/min sensitivity preset
- high-flow 30 L/min preset
- H1-4 linear model as a comparison tab

## Remaining work

The next control step is H1-5.

The nonlinear rod still uses a fixed hand boundary. H1-5-0 should introduce a
prescribed moving hand boundary and a work/energy diagnostic before turning it
into a manual-control game.
