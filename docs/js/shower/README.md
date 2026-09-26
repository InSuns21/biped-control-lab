# Shower TVC X1 design contract

This file fixes the physical conventions used by X1-0 and the first one-axis core in X1-1.
The later 3D implementation must preserve these signs and units unless this contract is deliberately revised together with its tests.

## Coordinate system

Use a right-handed world frame.

- +X: screen/right direction in the first one-axis visualization
- +Y: upward against gravity
- +Z: out of the XY plane
- gravity: `[0, -g, 0]`
- the upright shower body points along body +Y
- one-axis attitude `theta` is rotation about +Z

Because the reference body axis is +Y, positive `theta` rotates that axis toward -X. This is a consequence of the right-hand rule and is intentional.

The first one-axis model uses a fixed virtual pivot below the center of mass. It is an educational attitude mode, not a free-flying rigid body. X1-2 will add the full 3D rigid-body state.

## Geometry and inertia

The pivot is the origin. In the upright body frame:

- center of mass: `r_com = [0, l_com, 0]`
- nozzle force application point: `r_nozzle = [0, l_nozzle, 0]`
- default `l_com = 0.12 m`
- default `l_nozzle = 0.38 m`
- default mass `m = 0.35 kg`
- default one-axis pivot inertia `I_z = 0.018 kg m^2`

The scalar `I_z` is an effective educational parameter. It is not claimed to be an identified inertia of a commercial shower head.

## Gravity torque

For positive `theta`, gravity increases the tilt:

```text
tau_g = m g l_com sin(theta)
```

Therefore the upright equilibrium is open-loop unstable, matching the sign convention used in the inverted-pendulum chapters.

## Jet and TVC model

Internally, flow rate `Q` is SI `m^3/s`. UI code may display L/min but must convert at the boundary.

The first model uses:

```text
T = C_T rho Q^2 / A_eff
```

with defaults:

- `rho = 997 kg/m^3`
- `C_T = 0.85`
- `A_eff = 2.0e-5 m^2`
- nominal `Q = 8 L/min`
- maximum `Q_max = 10 L/min`

`T` is the reaction force on the body, opposite to the outgoing water momentum. At zero gimbal angle it points along body +Y.

Positive gimbal angle `delta` rotates the reaction-force direction about body +Z. With the geometry above:

```text
tau_jet = l_nozzle T sin(delta)
```

so positive `delta` creates positive +Z torque.

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

## Controller convention

Manual mode commands `delta` directly.

P mode:

```text
delta_cmd = -Kp theta
```

PD mode:

```text
delta_cmd = -Kp theta - Kd omega
```

Defaults for the educational model are:

- `Kp = 2.5`
- `Kd = 0.7 s`

Because `delta`, `theta` are radians and `omega` is radians/second, `Kp` is dimensionless and `Kd` has units of seconds in this direct-angle controller.

## Input constraints

X1-1 enforces:

```text
0 <= Q <= 10 L/min
|delta| <= 25 deg
```

The core preserves both command and applied values. UI and graphs must not silently replace `delta_cmd` with the saturated `delta`.

Gimbal slew-rate limiting is deliberately deferred. When added, it must be a separate actuator-state constraint and not be folded into the rigid-body equation.

## Relation to X1-2

The 3D extension should preserve the same semantics:

- world frame remains right-handed and Y-up
- body +Y remains the nominal thrust axis
- `delta = 0` means the reaction-force line is collinear with the body axis
- positive axis rotations obey the right-hand rule
- force and torque diagnostics keep SI units
- command/applied actuator values remain distinct under saturation

The one-axis model is the +Z slice of that later 3D model.
