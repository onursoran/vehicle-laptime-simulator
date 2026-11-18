# Straight-line brake-bias study

For the separate front/rear preference, limited automatic left/right assistance,
and passive hydraulic model, see [BRAKE_CONTROL.md](BRAKE_CONTROL.md).

Run from the project folder:

```powershell
.\.venv\Scripts\python.exe brake_bias_study.py
```

This prints a comparison table, displays three plots, and saves the plots and
a CSV to `outputs/brake_bias`. To save without opening plot windows:

```powershell
.\.venv\Scripts\python.exe brake_bias_study.py --no-show
.\.venv\Scripts\python.exe -m unittest test_brake_model -v
```

Edit the illustrative vehicle defaults in `BrakingParameters` in
`brake_model.py` when measured parameters become available. This study runs
independently of `main.py` and its expensive G-G search.

## What bias means here

`front_braking_fraction`, beta, is front tire braking force divided by total
tire braking force. A value of 0.60 assigns 60% to the front and 40% to the
rear. It is not master-cylinder pressure split, pedal-force split, or adjuster
position. Mapping the existing hardware to tire force is a later stage.

## Equations and signs

Use positive deceleration magnitude d in m/s^2. The existing vehicle model
receives signed longitudinal acceleration -d. Let m be mass in kg, g = 9.81
m/s^2, a be CG-to-front distance, b CG-to-rear distance, L = a + b, and h CG
height; all distances are in m. mu is the dimensionless friction coefficient.

Axle normal loads (N):

```text
Fz_front = m*g*b/L + m*d*h/L
Fz_rear  = m*g*a/L - m*d*h/L
```

Braking-force demand magnitudes (N):

```text
B_front = beta*m*d
B_rear  = (1-beta)*m*d
```

Each axle must satisfy B <= mu*Fz. Utilization is B/(mu*Fz).
A value of 1 means saturation; above 1 means the demand is infeasible.
Demand is never silently clipped or reported as an achieved force.

With equal mu at both axles, equal utilization requires:

```text
beta_ideal = Fz_front / (m*g) = b/L + d*h/(g*L)
```

This gives a force allocation for a specified demand; it does not prove the
demand is achievable. At d = 0, it is the limiting split as braking begins.

For a fixed beta, the grip inequalities become:

```text
d*(beta*L - mu*h) <= mu*g*b
d*((1-beta)*L + mu*h) <= mu*g*a
```

A positive left coefficient gives an upper bound on d. A zero or negative
coefficient gives no upper bound from that inequality. The solver takes the
smallest bound, also capped by rear axle lift at d = g*a/h when h > 0.
It returns which constraints are active. The lift boundary is incipient
loss of rear contact; demands beyond it are rejected.

## Current scope

Flat-road, straight-line, quasi-static braking; common constant mu; no aero,
cornering, brake-torque limits, tire load sensitivity, or wheel rotation.
The maximum is the onset of saturation for a fixed force split. It does not
simulate post-lock behavior or ABS redistribution. Plot points above the
utilization limit are requested states, not achieved decelerations.

The example vehicle should reach 1.5 g at about 69.44% front force, where both
axles saturate together. This is an analytical model result, not a measured
car capability. Pressure/torque calibration for the real balance bar needs
hardware specifications. The four-wheel concept can be explored meanwhile
using explicit assumptions, as described below.

## Four-wheel concept study without hardware measurements

```powershell
.\.venv\Scripts\python.exe four_wheel_study.py
.\.venv\Scripts\python.exe -m unittest test_brake_model test_four_wheel_model -v
```

Use `--no-show` to save without opening windows. The study saves a PNG,
CSV, and the exact assumed parameters as JSON in `outputs/four_wheel`.
`FourWheelParameters` in `four_wheel_model.py` adds assumed front/rear track
widths of 1.20 m and a front roll-moment share of 0.50. Existing mass, CG,
geometry and friction assumptions come from `BrakingParameters`.

### Four separate wheels

Wheel order is front left, front right, rear left, rear right. Coordinates
are x forward and y left; positive lateral acceleration and yaw mean turning
left. During a left turn, the right wheels are outside and gain load.

This first model assigns the total lateral overturning moment m*ay*h between
the axles using a specified front share lambda. Load moved from one side to
the other is:

```text
transfer_front = lambda*m*ay*h/front_track
transfer_rear  = (1-lambda)*m*ay*h/rear_track
Fz_left  = axle_load/2 - transfer
Fz_right = axle_load/2 + transfer
```

Track widths and CG height are in m; lateral acceleration ay is in m/s^2;
transferred load is in N. This is a lumped quasi-static roll model. It does
not resolve suspension geometry, roll centers, unsprung masses, or transient
roll. Negative wheel loads are rejected, not clipped into apparently valid
states.

Lateral axle forces are prescribed to balance lateral demand and yaw:

```text
Fy_front = m*ay*b/L
Fy_rear  = m*ay*a/L
```

Within each axle, Fy is distributed in proportion to wheel normal load.
These are assumptions, not outputs of a solved steering/slip-angle model.
Each wheel's remaining braking-force capacity follows the friction circle:

```text
B_max = sqrt((mu*Fz)^2 - Fy^2)
```

Excessive lateral demand is infeasible. Braking uses signed Fx = -B, so total
yaw moment in N m is:

```text
Mz = sum(x*Fy - y*Fx) = sum(x*Fy + y*B)
```

Under the prescribed lateral forces, sum(x*Fy) is zero before braking.
More braking at a left wheel tends to turn left; more at a right wheel tends
to turn right. The study calculates force allocations, not the ensuing yaw
rate, steering angle, slip ratio, or trajectory.

### Four strategies and a fair comparison

1. Fixed 60% front, equal left/right forces on each axle.
2. Freely adjustable front/rear force split, still equal left/right.
3. Independent wheels constrained to zero added yaw.
4. Independent wheels with unrestricted yaw, a theoretical braking bound.

At each lateral target, the study scans braking demand at 0.0025 g spacing.
The curves report highest feasible sampled demands, not exact optimized
maxima. Missing solutions stay NaN rather than being replaced with zero.
Independent forces must add to the requested total and stay inside all four
friction circles. Requested yaw must lie within the achievable yaw interval.

The range of yaw at fixed braking is obtained by allocating braking first
to the lowest or highest y positions. Convex interpolation between those
extreme allocations reaches any intermediate yaw while preserving the total
braking force and individual wheel bounds. The algorithm does not pick a
unique optimum for brake temperatures or actuator effort.

With equal tracks and this proportional lateral-force assumption, the
adjustable-axle and independent-zero-yaw limits can coincide. Independent
control should not be credited with a gain unless the constraints support
it. Unrestricted-yaw braking can use more outside-wheel capacity, but that
can produce yaw opposing the turn. It is not a demonstrated handling or
lap-time improvement.

This concept model excludes actuator bandwidth, hydraulic restrictions,
tire load sensitivity, and post-lock behavior. Real specifications and
validation remain necessary to quantify the benefit on the actual car.
