# Comparative lap-time model

## What is implemented

The app and CLI use the **illustrative research vehicle** described under
*Physical and theoretical comparisons* below.

- Synthetic braking oval, mixed-direction technical loop, figure-eight and
  constant-radius circle, with mirrored and reversed geometry support.
- Approximate real Monza, Montreal and Barcelona-Catalunya centerlines from
  the TUM 2020 dataset, with source provenance exported alongside each lap.
- Fixed centerline, configurable road width and front/rear wheel tracks.
- Four-wheel longitudinal/lateral load transfer reused from the existing model.
- Coupled steering, sideslip and actuation equilibrium with combined tire grip.
- Conventional two-master braking, fixed rear/side passive valves, and limited
  independent assistance with separate front/rear preference and strength.
- Rear-wheel-drive force and power limits, optional drag/rolling resistance,
  bounded hydraulic supply, steering/sideslip limits and local road clearance.
- A closed-lap speed profile, segment times, configuration comparisons,
  diagnostic CSVs/plots and a local interactive viewer.

## Real circuits and braking zones

`--track monza`, `--track montreal` and `--track catalunya` select bundled
TUM racetrack-database layouts; `--track real` runs all three. The files are
pinned, unchanged source data derived from OpenStreetMap, with attribution
and licenses in [data/tracks/README.md](data/tracks/README.md). Catalunya has
the historical final chicane and old Turn 10. These are approximate 2020
layouts, not surveyed or confirmed current circuit geometry.

A periodic cubic spline gives continuous position, tangent and second
derivative (C2 continuity), including the closing seam. The loader integrates
arc length and samples the same spline at the requested spacing; coordinates
remain in metres without rescaling to an official published lap length.
Curvature comes from spline derivatives. Elevation and banking are omitted;
the driving line stays fixed on a flat plane.

Real circuits default to a 12 m uniform corridor and 4 m sample spacing;
synthetic tracks default to 3 m and 2 m. `--width` and `--spacing` override
these values. Source left/right width estimates are retained in the data
files but do not define the current symmetric corridor. Mirroring is a
geometric reflection for testing; it is not reverse travel on the original
circuit. The illustrative vehicle has a 25 m/s speed cap. Braking zones therefore depend on the car and
configuration rather than on published Formula 1 brake markers.

`braking_analysis.py` classifies each segment using its tangential
deceleration and midpoint hydraulic command. A segment is classified only
when deceleration exceeds 0.05 g and pushrod force exceeds 1 N. Light braking
is below 0.3 g, medium is 0.3 to below 0.7 g, and hard is at least 0.7 g.
These are explicit display thresholds, not physical controller switching
rules. Consecutive classified segments form zones, including zones crossing
the start/finish seam. The browser colors the track and speed plot with
these calculated categories, and reports peak deceleration and distance.

Each run exports `braking_zones.png`, per-configuration `*-braking.json`,
and braking-category CSV columns alongside the lap comparison. Default output
is `outputs/lap_time`; the real-track verification run uses
`outputs/real_tracks`. Imported data and interpolation code are included in
the source fingerprint. Older synthetic saved results must be regenerated
with `--track all --include-mirrors` before their fingerprint audit passes.

## Equations and frames

The vehicle axes remain x forward, y left, positive yaw left. Speed V is along
the prescribed path, sideslip beta is the velocity direction relative to the
body, kappa is signed path curvature (1/m), kappa' its distance derivative
(1/m²), and a is tangential acceleration (m/s²).

```text
u = V*cos(beta), v_body = V*sin(beta)
r = V*kappa
r_dot = a*kappa + V²*kappa'
ax_body = a*cos(beta) - V²*kappa*sin(beta)
ay_body = a*sin(beta) + V²*kappa*cos(beta)
```

These relations use locally constant sideslip. The model does not integrate
the changing sideslip/yaw states over a whole lap; they are quasi-steady
operating points, not a demonstrated transient trajectory.

Normal loads use signed body accelerations in the shared
`calculate_four_wheel_contacts` helper. Optional quadratic downforce,
split front/rear by a fixed fraction, is added to normal loads. It never adds
inertial mass: total normal load is m*g + downforce, while acceleration and
roll/yaw equations use the physical mass and inertia. The prescribed aero
balance omits unknown drag height and suspension/aero coupling.
Each wheel has local velocities
`u-r*y` and `v_body+r*x`. Front slip angle is steering minus local travel
direction; rear steering is zero. The existing linear lateral stiffness law
is reused. The existing axle stiffness of 20,000 N/rad is divided between
the two tires in proportion to normal load, preserving its axle meaning.

For specified longitudinal tire force Fx, lateral force is capped by:

```text
abs(Fy) <= sqrt((mu*Fz)²-Fx²)
```

This holds longitudinal force fixed rather than scaling the requested brake
force away. Longitudinal force above mu*Fz remains infeasible. Front forces
are transformed into body coordinates using the existing transformation.
The solver simultaneously adjusts actuation, beta and steering until:

```text
sum(Fx_body) = m*ax_body
sum(Fy_body) = m*ay_body
sum(x*Fy_body-y*Fx_body) = Iz*r_dot
```

Thus differential brake yaw is balanced by changed lateral tire forces and
steering, rather than counted as a free braking benefit or simply forbidden.
Curvature transitions and changing speed also require yaw acceleration.
Drive and brake actuation are separate nonnegative branches. Both are checked
when necessary at the coast boundary; constant path speed is not assumed to
always require drive, and light deceleration is not assumed to require brakes.

The local road check tests body and tire-outline corners against an
osculating-circle corridor. It is an approximation for a smoothly varying
track, not a polygonal track-boundary collision engine or racing-line optimizer.

The diagnostic `OperatingPoint.brake_yaw_moment_nm` is the controller's
unsteered left/right reference sum(y*B), rather than the total moment of all
steered brake-force vectors. The assistance cap applies to this reference;
front steering reduces its actual differential contribution by cos(steering).
`yaw_moment_nm` and the force/yaw residuals always use the full transformed
forces, including the longitudinal lever arms and steering effects.

## Physical and theoretical comparisons

`research_setup.py` defines the **illustrative** research inputs. Existing
mass/CG/friction/track/inertia values are reused. Missing inputs are labeled
research placeholders: two 19.05 mm master bores, tire-force gains of
0.0002 N/Pa per wheel, maximum bar input 4000 N, 40 kW drive power,
3000 N drive force, 25 m/s speed cap, 10-degree sideslip bound, and the
illustrated body/tire dimensions. Drag and rolling resistance default to zero.
If enabled, they are equivalent ground-level resistive losses in this simple
load-transfer model; drag height and its additional pitch moment are not resolved.
Downforce is optional and off by default for this vehicle.
The passive knee defaults to a **research** 20 bar, with a documented example
post-knee slope of 1/3; the knee is not identified from the actual installed car.

All inputs are exported in `comparison.json`. Values must be replaced or
swept before interpreting the actual car's performance.
The export also records the physical source fingerprint and NumPy version.
`validate_lap_results.py` audits all six track/direction comparisons against
the current fingerprint, declared limits, load conservation and assistance
front/rear invariance, and writes `validation_report.json`.

The conventional system uses two master cylinders and symmetric branches.
Its bar setting is calibrated to the requested front tire-force fraction at
a reference 2000 N bar force without valves. Passive variants retain that
same setting. Their actual front/rear fraction can change with pressure;
they do not secretly retune the bar each corner. Left/right installations
remain fixed. Pressure retention is rejected by the quasi-steady envelope
because it requires state/history; use the timed hydraulic model for that probe.

The adaptive benchmark preserves the driver-requested axle tire-force totals.
Its current-signal observer estimates lateral load distribution; the separate
assistance fraction scales its capacity-matched left/right differential.
The vehicle equilibrium then checks the actual resulting tire forces. Observer
estimates choose the command, not a second unrelated physical feasibility
constraint. This distinction is regression-tested at zero assistance.

The same master supply limits bound the adaptive demands. Required upstream
bar force is the largest per-wheel demand relative to its available supply;
individual outlet pressures cannot exceed the corresponding source. This
is ideal active pressure reduction with sufficient actuator authority, not
behavior provided by passive valves or by a motor on the front/rear bar alone.

The lap envelope uses memoryless assistance. Nonzero filter/slew dynamics are
explicitly rejected when constructing a lap strategy, rather than silently
ignored. Those effects remain in `brake_control_study.py`; fully coupling
them to a lap requires a transient integration/controller stage. The lap
controller receives current speed, lateral acceleration, steering, demand
and supply/estimated-grip information, with no track, future corner, or
distance argument. Track geometry is used by the offline driver/environment.

## Speed profile and numerical checks

The initial node speed limits come from feasible constant-speed equilibria.
Backward braking and forward acceleration passes reconcile all segments
including the last-to-first seam. For a segment of length ds:

```text
a = (V_end²-V_start²)/(2*ds)
dt = 2*ds/(V_start+V_end)
lap time = sum(dt)
```

Each segment is checked at its start, midpoint and end, with interpolated
curvature/width and the corresponding yaw-acceleration demand. A result is
returned only when the whole closed profile passes these checks. Exported
maximum equilibrium errors, grip use and steering include all three samples.
The CSV steering/load/force columns are explicitly midpoint quantities.

Normalized force and moment residual tolerance defaults to 2e-6, with force
scale m*g and moment scale m*g*L. A small corner-speed margin and default
speed-search resolution of 0.015 m/s make results conservative at numerical
boundaries. A converged feasible profile is not proof of a globally optimal
minimum-time solution; nonlinear feasibility can have multiple branches.

Default track spacing is 2 m for synthetic tracks and 4 m for real circuits.
Use 2 m, 1 m and 0.5 m refinement to assess numerical
sensitivity. Configuration differences can be much smaller than uncertainty
from tire, hardware, driver and suspension assumptions. The figure-eight
has a geometric crossing and no traffic/collision interaction.

Lengths, curvature gradients and segment geometry are prepared once per solve.
Identical drive-mode equilibria can be reused across brake strategies because
no brake setting enters that branch. Brake-mode solutions and fallbacks always
retain the requested strategy. Fixed-size caches include every relevant vehicle
and geometry input; they do not round floats or weaken feasibility checks.
The app defaults to two configurations and reports profiles incrementally;
its geometry-only preview never invokes the lap solver.

Tests cover analytical circle lap time with an independent analytical
performance envelope, acceleration/braking and closed-lap constraints, full
vehicle force/yaw closure, curvature-gradient yaw demand, baseline recovery
at zero assistance, mirror/valve-swap consistency, supply/power/road bounds,
and low-speed drive/coast/brake transitions on the technical loop.

## Unresolved research limitations

Real tire load sensitivity and nonlinear/post-peak tire data, Ackermann and
suspension geometry, transient roll/sideslip/yaw, wheel rotational slip and
lock recovery, hydraulic compliance/hysteresis/delay, driver steering effort
and confidence, thermal effects, actual hardware mass/packaging, and Formula
SAE Michigan 2027 compliance are not established. The linear tire model can
hold a saturated force plateau; it does not simulate a locked wheel.

The driver front/rear preference is held constant within each lap in this
first comparison, so both systems get equivalent control assumptions. The
underlying timed controller accepts changing driver preference, but this
planner does not give either system free per-corner retuning. The result is
a repeatable comparative research estimate, not a claim of real-world lap
time, stability, safe installation, or competition eligibility.
