# Shower TVC X1 design contract

This file fixes the physical conventions used by X1-0, X1-1 and X1-2.
Rendering and game code must preserve these signs and units unless this contract
is deliberately revised together with its regression tests.

## Coordinate system

Use a right-handed world frame.

- +X: screen/right direction in the first one-axis visualization
- +Y: upward against gravity
- +Z: out of the XY plane
- gravity: `[0, -g, 0]`
- the upright shower body points along body +Y
- one-axis attitude `theta` is rotation about +Z

Because the reference body axis is +Y, positive `theta` rotates that axis toward
-X. This is a consequence of the right-hand rule and is intentional.

The first one-axis model and the X1-2 model both use a fixed virtual pivot below
the center of mass. They are attitude-control models, not yet free-flight
translation models.

## Geometry and inertia

The pivot is the origin. In the upright body frame:

- center of mass: `r_com = [0, l_com, 0]`
- nozzle force application point: `r_nozzle = [0, l_nozzle, 0]`
- default `l_com = 0.12 m`
- default `l_nozzle = 0.38 m`
- default mass `m = 0.35 kg`
- one-axis `I_z = 0.018 kg m^2`
- 3D diagonal inertia `I = diag(0.018, 0.009, 0.018) kg m^2`

These are effective educational parameters. They are not claimed to be an
identified inertia model of a commercial shower head.

## Gravity torque

For positive one-axis `theta`, gravity increases the tilt:

```text
tau_g = m g l_com sin(theta)
```

Therefore the upright equilibrium is open-loop unstable, matching the sign
convention used in the inverted-pendulum chapters.

In X1-2, gravity is defined in world coordinates and transformed to the body
frame before computing

```text
tau_g_body = r_com_body x F_g_body
```

so the same sign convention survives arbitrary 3D attitude.

## Jet and TVC model

Internally, flow rate `Q` is SI `m^3/s`. UI code may display L/min but must
convert at the boundary.

The model uses:

```text
T = C_T rho Q^2 / A_eff
```

with defaults:

- `rho = 997 kg/m^3`
- `C_T = 0.85`
- `A_eff = 2.0e-5 m^2`
- nominal `Q = 8 L/min`
- maximum `Q_max = 10 L/min`

`T` is the reaction force on the body, opposite to the outgoing water
momentum. At zero gimbal angle it points along body +Y.

The 2-axis convention is:

1. start with the reaction force along body +Y
2. rotate by `delta_x` about body +X
3. rotate by `delta_z` about body +Z

Thus, when the other gimbal angle is zero:

- positive `delta_x` creates positive body-X torque
- positive `delta_z` creates positive body-Z torque

The X1-1 scalar model is exactly the `delta_x = 0`, +Z rotational slice of
X1-2.

## One-axis dynamics

X1-1 integrates:

```text
I_z theta_ddot
  = m g l_com sin(theta)
  + l_nozzle T sin(delta)
  + tau_disturbance
```

The state is only:

```text
theta
omega = theta_dot
```

The integrator is fixed-step semi-implicit Euler:

```text
omega[k+1] = omega[k] + alpha[k] dt
theta[k+1] = theta[k] + omega[k+1] dt
```

Rendering code must not contain a second copy of these dynamics.

## X1-2 quaternion and rigid-body convention

Quaternion storage is

```text
q = [w, x, y, z]
```

and `qBodyToWorld` maps a vector expressed in body coordinates into world
coordinates. Body angular velocity is stored as

```text
omegaBodyRadS = [omega_x, omega_y, omega_z]
```

and is always expressed in the body frame.

For diagonal body-frame inertia, X1-2 evaluates Euler's rigid-body equation:

```text
I omega_dot + omega x (I omega) = tau_body
```

Quaternion kinematics use the same body-frame angular velocity:

```text
q_dot = 1/2 q tensor_product [0, omega_body]
```

The numerical step updates angular velocity first, then quaternion, and
normalizes the quaternion every step.

## Controller convention

Manual one-axis mode commands `delta` directly.

P mode:

```text
delta_cmd = -Kp theta
```

PD mode:

```text
delta_cmd = -Kp theta - Kd omega
```

Defaults for the educational one-axis model are:

- `Kp = 2.5`
- `Kd = 0.7 s`

Because `delta`, `theta` are radians and `omega` is radians/second,
`Kp` is dimensionless and `Kd` has units of seconds in this direct-angle
controller.

X1-2 deliberately stops at the 3D plant/actuator model. Quaternion attitude
feedback is added after the 3D plant is visually inspectable rather than being
hidden inside the physics core.

## Input constraints

X1-1 enforces:

```text
0 <= Q <= 10 L/min
|delta| <= 25 deg
```

X1-2 extends this independently to both axes:

```text
|delta_x| <= 25 deg
|delta_z| <= 25 deg
0 <= Q <= 10 L/min
```

The core preserves command and applied values separately. UI and graphs must not
silently replace requested values with saturated values.

Gimbal slew-rate limiting is deliberately deferred. When added, it must be a
separate actuator-state constraint and not be folded into the rigid-body
equation.

## Rendering contract for X1-3

The renderer must consume the physics state rather than recomputing it.

- world frame remains right-handed and Y-up
- body +Y remains the nominal reaction-force axis
- `delta_x = delta_z = 0` means the force line is collinear with body +Y
- positive axis rotations obey the right-hand rule
- force and torque diagnostics keep SI units
- command/applied actuator values remain distinct under saturation
- quaternion remains authoritative; Euler angles are display-only
