# Side Lab X1 physical-model contract

This file defines the physical-model status of Side Lab X1.

## Canonical project structure

Side Lab X1 now has two intentionally different models.

### Phase 0 — rigid baseline

The existing hanging-shower rigid-body model is retained as a **comparison model**.

It includes:

- a held point above the shower head
- COM below the hand
- fixed downward water jet
- opposite reaction force
- rigid shower head
- rotational spring-damper support
- quaternion 3D attitude

This model is expected to settle because gravity is restoring and the support
contains damping. That behavior is no longer treated as the target phenomenon.

Phase 0 exists to answer:

> Why does a rigid-end model become calm even though the real flexible hose can
> keep whipping around?

Do not add ad-hoc oscillation to Phase 0 just to make it look more dramatic.

### Phase 1 — flexible hose with internal flow

This is the new primary model.

The target phenomenon is a flexible hose conveying water, with a shower head
at the free end, where internal flow can alter modal damping/stiffness and can
produce a flutter-like self-excited response above a critical flow regime.

The player acts at the hand / hose boundary, not by gimbaling the nozzle.

---

## Phase 0 contract

The current Phase 0 implementation remains valid as a frozen baseline.

World frame:

- right-handed
- +Y up
- gravity `[0, -g, 0]`

Rigid-body convention:

- `qBodyToWorld = [w, x, y, z]`
- body +Y points toward the held hose / hand
- body -Y points toward the hanging shower head
- water exits approximately body -Y
- the body receives the opposite momentum reaction

The COM lies below the held point, so small-angle gravity torque is restoring.

Phase 0 regression tests must continue to pass after Phase 1 is added.

---

## Phase 1 continuum contract

Phase 1 starts in 2D.

Use arc/axial coordinate

```text
s in [0, L]
```

and transverse hose displacement

```text
y = y(s, t)
```

with parameters:

- `L`: hose length
- `EI`: bending stiffness
- `m_s`: structural mass per unit length
- `A_hose`: internal flow area
- `rho`: water density
- `m_f = rho A_hose`: water mass per unit length
- `Q`: volume flow
- `U = Q / A_hose`: mean internal flow speed
- `c`: effective structural/material damping

The reference small-deflection conveying-fluid beam model has the conceptual
form

```text
(m_s + m_f) y_tt
+ c y_t
+ EI y_ssss
+ 2 m_f U y_st
+ m_f U^2 y_ss
= f_ext
```

The implementation must not assume these signs or element matrices from memory.
Before H1-1/H1-2 coding, derive the discrete form under one fixed coordinate
and boundary convention and lock it with regression tests.

The important model distinction is that the flow contributes distributed
velocity- and flow-speed-dependent terms. Phase 1 must therefore not be reduced
to a rigid body plus a single outlet force.

---

## Phase 1 discretization contract

First implementation: 2D Euler–Bernoulli beam FEM.

Per node:

```text
q_i = [y_i, theta_i]
```

Assemble the semi-discrete system as separate physical terms:

```text
M q_ddot + C(U) q_dot + K(U) q = f
```

Keep these contributions separately inspectable:

- structural mass
- internal-fluid mass
- structural damping
- bending stiffness
- velocity-dependent flow coupling
- `U^2` flow coupling
- boundary / shower-head loads

Do not hide all terms inside a single opaque update function.

The first mesh should be small enough for the browser and large enough for
low-mode convergence; roughly 8–16 beam elements is the initial target, not a
hard contract.

---

## Boundary conditions

### Hand end

The player acts at `s = 0`.

The first Phase 1 version uses prescribed hand-boundary motion:

- lateral hand position
- hand / hose angle

This is a boundary-control problem.

### Shower-head end

The free end at `s = L` may carry:

- shower-head tip mass
- tip rotational inertia
- head geometry
- outlet orientation

Outlet momentum must be handled consistently with the chosen continuum/FEM
formulation.

**Do not double count the same outlet/follower-force effect** once through the
flow matrices/boundary condition and again as an extra hand-written tip force.

If bent internal head plumbing creates an additional reaction not already
represented by the straight conveying-hose model, add that as a separate,
documented tip load.

---

## Numerical contract

Phase 1 is potentially stiff and non-conservative.

The Phase 0 semi-implicit Euler choice does not automatically carry over.

Candidate integrators include:

- Newmark-beta
- generalized-alpha
- state-space integration with a stable fixed step

The chosen method must be justified by regression tests, not animation quality.

Rendering frame rate and physics time step must be independent.

---

## Stability contract

Phase 1 is considered physically useful only if it can distinguish at least
three regimes as flow increases:

1. stable / decaying perturbations
2. near-critical weak decay
3. above-critical growing oscillation

Critical flow is defined from the discretized system eigenvalues, not from a
hard-coded animation threshold.

The expected regression concept is:

```text
max Re(lambda(U)) < 0   below critical
max Re(lambda(U)) ~= 0  near critical
max Re(lambda(U)) > 0   above critical
```

The exact value of the critical speed is model-parameter and mesh dependent.
Tests should verify convergence and sign changes rather than inventing a
universal shower-hose number.

---

## Required Phase 1 regressions

Before interactive visualization, the physics core must test:

- `U = 0` removes `G_flow` and `K_flow`, while a filled hose retains `M_fluid`
- reversing `U` flips the velocity-linear coupling sign
- reversing `U` does not flip the `U^2` coupling
- dry damped hose decays from an initial displacement
- tip mass lowers the first natural frequency
- low-mode natural frequencies converge under mesh refinement
- a low-flow case remains stable
- a flow sweep identifies a critical region
- a selected above-critical case has a growing mode
- time integration agrees qualitatively with eigenvalue stability
- no NaN / Inf
- result is not materially changed by small dt refinement
- outlet momentum is not counted twice

---

## Relationship to TVC and biped control

A flexible shower hose is not a rocket TVC system.

Comparison should be explicit:

- flexible hose: distributed flexible dynamics + internal-flow coupling +
  boundary control
- rocket TVC: thrust-direction actuation on a rigid body
- biped: contact-force / CoP constraints and hybrid support changes

The useful common control idea is:

> motion is controlled only through physically realizable external forces,
> moments, contacts, or boundary inputs.

Phase 1 adds another important lesson:

> the plant itself may contain lightly damped or unstable flexible modes, so a
> controller that works for a rigid approximation may fail badly on the real
> distributed system.

---

## Implementation status

- Phase 0 rigid baseline: implemented
- Phase 0 Human Visual Audit: comparison-model re-audit pending
- Phase 1 H1-0 model contract / derivation: implemented
- Phase 1 H1-1 dry flexible beam core: implemented
- Phase 1 H1-2 conveying-fluid coupling / flutter validation: implemented
- Phase 1 H1-3 shower-head boundary model: implemented
- straight H1-2 reference: `U_cr ~= 9.4808 m/s`, `Q_cr ~= 16.08 L/min`
- bent-head H1-3 reference: `U_cr ~= 8.3871 m/s`, `Q_cr ~= 14.23 L/min`
- straight/equal-area H1-3 limit is regression-locked to reproduce H1-2
  exactly, preventing duplicate outlet momentum loading
- Phase 1 H1-4 2D interactive visualization: implemented
- Phase 1 is now the default Side Lab surface; Phase 0 is a comparison tab
- H1-4 reference presets: 8 L/min stable, 14 L/min near critical,
  18 L/min clearly above critical
- H1-4 CI time histories: 8 L/min late/early RMS ~= 0.062,
  18 L/min ~= 2.702 over the inspection horizon
- next implementation step after visual approval: **H1-5 manual boundary-control game**
- Phase 1 Human Visual Audit is still pending
