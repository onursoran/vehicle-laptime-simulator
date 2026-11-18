# Separate driver controls and practical limits

This stage adds `adaptive_brake_control.py` (an active force-control benchmark)
and `brake_hydraulics.py` (two masters and fixed passive branch valves).
They are separate capabilities. Passive valves do not acquire automatic
corner detection, and neither model establishes competition legality.

## Run

```powershell
.\.venv\Scripts\python.exe brake_control_study.py
.\.venv\Scripts\python.exe -B -m unittest discover -v
```

Use `--no-show` to generate outputs without plot windows. Outputs are under
`outputs/brake_control`: two PNGs, maneuver CSV, sensitivity CSV, valve-curve
CSV, and the explicit research inputs JSON. Earlier studies and the existing
browser visualization remain available. The updated local interface and
quasi-steady lap comparison are described in [README.md](README.md) and
[LAP_TIME_MODEL.md](LAP_TIME_MODEL.md). The old standalone snapshot view is
retained separately.

To select 65% front force and 40% left/right assistance, compared with
assistance off at the same front/rear setting:

```powershell
.\.venv\Scripts\python.exe brake_control_study.py --front-bias 65 --strength 40
```

`--modulated-axles front` or `rear` isolates an axle. `--yaw-limit 80` sets an
80 N m research ceiling. The default maneuver changes front preference during
the run to verify recomputation; `--front-bias` holds it fixed. The sensitivity
CSV deliberately retains its documented full parameter grid, independently of
the selected maneuver's strength/front preference. Yaw-moment rate is included
in the maneuver CSV to inspect abrupt changes, without inventing a validated
threshold for driver confidence.

## Meaning of the two controls

The driver front/rear preference beta specifies the front fraction of total
tire braking-force demand. It is distinct from the balance-bar pushrod-force
fraction and from a master-cylinder pressure fraction.

For total braking demand B = m*d (N), where d is positive deceleration in
m/s² and m is mass in kg:

```text
B_front = beta*B
B_rear  = (1-beta)*B
```

Left/right authority is a separate dimensionless fraction alpha from 0 to 1.
The driver does not select a fixed left or right side. Each axle computes:

```text
C_left, C_right = remaining longitudinal grip, optionally limited by supply
q_peak = (C_right - C_left)/(C_right + C_left)
q_requested = alpha*q_peak
B_left  = B_axle*(1-q_applied)/2
B_right = B_axle*(1+q_applied)/2
```

Zero total capacity gives q_peak = 0 and any nonzero demand is infeasible.
At full authority this distribution equalizes use of remaining longitudinal
capacity within the axle. It minimizes the worst longitudinal demand/capacity
ratio for that axle's fixed force total. **Peak means this capacity-matched
reference, not maximum possible yaw, optimum handling, or optimum lap time.**
At a given setting, 50% strength permits half that reference differential,
before additional constraints. Assistance cannot exceed the driver-selected
fraction just to make an infeasible braking request appear feasible.

The algorithm recalculates wheel forces and yaw limits when beta changes.
It preserves axle totals even when the command exceeds grip: it reports the
infeasibility and the need to reduce total braking demand rather than silently
overriding the driver's front/rear preference. The next lap-time stage must
search for achievable deceleration using this feasibility result.

## Drivability measures implemented

- Configurable absolute axle-asymmetry and yaw-moment bounds.
- Front-only, rear-only, or combined assistance.
- Straight-line, low-speed, steering/lateral deadband, and countersteering
  gates. No track geometry or future signal is accepted by the controller.
- A first-order lateral-signal filter for gating and a slew limit on q.
  Raw current lateral demand remains in grip checks, so filtering cannot
  conceal a force-limit violation.
- A configurable reserve on remaining longitudinal capacity and optional
  per-wheel supply-force ceilings. Omitted supply limits mean ideal actuation,
  not an assertion that the real hydraulics can generate these forces.
- Combined tire utilization, per-wheel residual lateral capacity, yaw moment,
  and instantaneous yaw acceleration Mz/Iz.
- Repeatable application/release, opposite turns, changing front/rear
  preference, and sensitivity to common friction, vehicle mass, and axle
  selection. Supplied research limits are not validated safety thresholds.

Hard authority, gating, and yaw boundaries take precedence over the q slew
limit. A sudden authority reduction or corner reversal can therefore override
slew; the result reports `authority_limit_overrode_slew`. This is an explicit
control conflict, not proof of perfectly smooth behavior. q slew is not a
calibrated hydraulic pressure bandwidth; total driver demand can still change
wheel forces quickly. Filtering and rate limits have no stabilizing guarantee.

The yaw inertia default is 150 kg m², the value already used in `main.py`.
It is now configurable in `FourWheelParameters`. Snapshot yaw acceleration
uses the existing prescribed lateral-force distribution. It is not an
integrated yaw-rate response, stability margin, or driver steering correction.

## Passive hydraulic model

Two master-cylinder areas and four pressure-to-tire-force gains are required
inputs; there are no invented installed component values. Force gains include
caliper geometry, pad friction, effective rotor radius and rolling tire radius,
or can come from calibration. Bar input is pushrod force in N, after pedal
leverage. The pressure outputs are Pa, and unsaturated tire-force demands N.

Each normal branch is bypassed or uses a continuous knee curve:

```text
P_out = P_in                                      below the knee
P_out = P_knee + slope*(P_in-P_knee)              above the knee
```

The primary source is [Tilton instruction 98-1261](https://tiltonracing.com/wp-content/uploads/98-1261-Brake-Proportioning-Valve_.pdf):
it documents 1:1 below the knee, incremental slope 1/3 above it, and a
650 psi example knee setting. Those are examples for the documented valve,
not identification of the car's installed parts. The study plots that example
and uses a user-chosen research inlet range of 0–80 bar.

Valve locations are explicit FL, FR, RL, RR entries. A left-side valve remains
on the left during right turns and straight-line braking. Normal P_out is
between zero and P_in. The same quasi-static curve is used during application
and release; actual hysteresis, lag and compliance require measurements.

Reducing selected branch pressures can change the actual front/rear tire-force
fraction at a fixed bar setting. `solve_master_force_fraction` finds a bar-force
setting giving the desired force fraction at one pushrod load. This is an
offline setting/calibration calculation. Repeating it online would require an
active adjuster and its own hardware and rate constraints. It cannot make
fixed passive hardware preserve every requested force split automatically.

Failure probes cover blocked delivery from a depressurized branch, bypassed
proportioning, pressure retention, and complete loss of either hydraulic
circuit. A retention fault can hold earlier pressure above the current inlet
pressure; this is trapped pressure, not amplification. Circuit loss sets both
corresponding wheel outputs to zero. These deliberately simple probes omit
leak flow, fluid volume/compliance, topology-dependent trapped pressure, and
pedal travel/BOTS behavior; they are not safety or certification analyses.

## Remaining capability gaps

| Requirement | Current status |
|---|---|
| Separate front/rear preference and left/right strength | Implemented in the active benchmark |
| Pressure-dependent passive valves and actual force split | Implemented, component inputs required |
| Supply-constrained active command | Optional force ceilings; full pressure-controller coupling remains |
| Combined grip, yaw moment and snapshot yaw acceleration | Implemented using current quasi-static physics |
| Steering compensation | Quasi-steady steering/sideslip equilibrium in the lap model; transient handling and driver workload remain unresolved |
| Wheel lockup and recovery | Saturation is reported; wheel rotational dynamics remain |
| Yaw-rate response, stability and driver confidence | Not established by these studies |
| Lap times | Implemented as audited fixed-line quasi-steady estimates; transient corner-entry/stopping behavior remains unresolved |
| Thermal effects, hardware mass/packaging/failure risk | Await component data and separate engineering evaluation |
| Formula SAE Michigan 2027 compliance | Unresolved; verify current rules and applicable interpretation |

The earlier unrestricted-yaw braking curve remains an upper-bound experiment.
It must not be used as a lap-time braking limit without satisfying total yaw,
steering and path constraints. Signed left/right cases must be solved separately
for fixed asymmetric hardware; the symmetric G-G mirroring assumption does
not apply to those configurations. The new equilibrium/track solver carries
these distinctions into the comparisons; see LAP_TIME_MODEL.md for the
implemented constraints and remaining transient-model limitations.
