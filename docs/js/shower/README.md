# Hanging Shower Control — X1 design contract

This file is the physical design contract for Side Lab X1.

The canonical setup is:

> the player holds the hose above, the shower head hangs below it, and water
> exits downward. The task is to keep the hanging head from swinging or
> twisting while keeping the water direction close to world down.

The original prototype incorrectly treated the shower as an inverted body with
a movable TVC nozzle. Human Visual Audit rejected that setup. The rules below
replace it.

## Frames and nominal pose

Use a right-handed world frame.

- +X: right
- +Y: up
- +Z: out of the initial front plane
- gravity: `[0, -g, 0]`
- quaternion `qBodyToWorld = [w, x, y, z]`
- identity attitude is the desired hanging pose

In the body frame:

- body +Y points from the head toward the held hose / hand
- body -Y points from the hand toward the hanging head
- the nominal water jet points along body -Y

Therefore identity attitude means the water jet points world down.

## Hand, center of mass and nozzle geometry

The hose joint / held end is the rotational pivot.

Default educational geometry:

- COM: `r_com = [0, -0.17, 0] m`
- resultant nozzle point: `r_nozzle = [0.045, -0.33, 0.02] m`
- mass: `m = 0.35 kg`
- diagonal body inertia:
  `I = diag(0.018, 0.009, 0.018) kg m^2`

The COM is below the hand. Gravity is therefore restoring, not destabilizing.

The nozzle resultant line is deliberately offset from the hose axis. That
represents a bent / asymmetric shower head where the summed outlet momentum
does not act through the held hose line.

These are educational effective parameters, not an identified commercial
shower-head model.

## Water momentum model

Flow rate is stored internally in SI `m^3/s`. UI code may display L/min.

The effective reaction-force magnitude is

```text
T = C_T rho Q^2 / A_eff
```

with defaults:

- `rho = 997 kg/m^3`
- `C_T = 0.85`
- `A_eff = 2.0e-5 m^2`
- nominal `Q = 8 L/min`
- maximum `Q = 10 L/min`

Water exits along body `-Y`, so the rigid body receives the opposite
reaction

```text
F_reaction_body = [0, +T, 0]
```

and the rotational effect about the held point is

```text
tau_water = r_nozzle x F_reaction
```

The first model does not solve the internal flow field. The offset resultant is
an effective representation of inlet turning, outlet momentum flux and the
shower-head geometry.

## Gravity torque

Gravity is defined in world coordinates and transformed to body coordinates.

```text
F_g_world = [0, -m g, 0]
tau_g_body = r_com_body x F_g_body
```

Because `r_com` lies below the hand, a small tilt receives a restoring
gravity torque.

For the one-axis +Z slice:

```text
tau_g = -m g l_com sin(theta)
```

The minus sign is a required regression check.

## Hand / hose holding model

The player does not gimbal the nozzle.

Instead, the controls specify the direction of the hose being held above the
head. Zero input means the held hose points world +Y.

Let

- `a`: current body +Y hose axis in world coordinates
- `h`: player-commanded held-hose direction in world coordinates

The flexible hose / wrist support is modeled as a rotational spring-damper:

```text
tau_hold_world = K_h (a x h)
tau_hold_body  = R(q)^T tau_hold_world - C_h omega
```

The default model damps X/Z tilt strongly and yaw weakly.

This is not intended to be a detailed hose finite-element model. It gives the
player a physically interpretable way to create a counter-torque by changing
how the hose is held.

## 3D rigid-body dynamics

Angular velocity and inertia are expressed in the body frame.

```text
I omega_dot + omega x (I omega)
  = tau_gravity + tau_water + tau_hold + tau_disturbance
```

Quaternion kinematics use

```text
q_dot = 1/2 q tensor_product [0, omega_body]
```

The numerical step is fixed-step semi-implicit Euler for angular velocity,
followed by quaternion integration and normalization.

## One-axis slice

For the +Z planar slice:

```text
I theta_ddot
  = -m g l_com sin(theta)
  + x_nozzle T
  + K_h sin(phi_hold - theta)
  - C_h theta_dot
  + tau_disturbance
```

where `phi_hold` is the direction in which the player holds the hose.

At nominal flow, a small counter-tilt of the held hose can balance the offset
water-reaction torque while the head remains nearly vertical.

## Input constraints

The first version enforces:

```text
0 <= Q <= 10 L/min
|hold_tilt_x| <= 25 deg
|hold_tilt_z| <= 25 deg
```

Commanded and applied values remain separate so saturation is visible in the
HUD and later game scoring.

## Relationship to TVC

A normal fixed-nozzle shower head is **not** thrust-vector control.

TVC remains useful later as a comparison:

- shower: the player changes the support / force balance of a hanging body
- rocket TVC: the actuator changes the thrust direction itself
- biped: the controller changes the realizable ground-reaction force / CoP

The common lesson is that attitude control depends on what external forces and
moments are physically realizable. The main X1 game must not present the shower
nozzle itself as a gimbaled rocket nozzle.

## Rendering contract

The renderer consumes the physics state and diagnostics; it does not rederive
the dynamics.

The 3D view must make these facts visually obvious:

- the hand / held hose is above
- the head and COM are below
- water exits downward
- reaction force points opposite the water
- gravity points world down
- water-reaction torque and hose-holding torque are distinguishable
- the target is a downward water direction
- quaternion state is authoritative; Euler angles are display-only

Human Visual Audit remains required after deployment.
