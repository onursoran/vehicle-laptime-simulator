# Inside-rear fiddle-brake comparison

Select **Conventional + inside-rear fiddle brake** in the browser's Compare
menu, then run a comparison. The figure-eight is a useful first test because
it contains both left and right turns. Orange wheel outlines and **Aux brake N**
show the selected auxiliary brake during playback. The ordinary front/rear
brakes remain available; only the *additional* brake acts at one rear wheel.

This experiment is inspired by McLaren's brake-steer concept. McLaren's
[heritage account](https://www.mclaren.com/racing/heritage/papaya-mirror-mclarens-greatest-what-ifs/)
describes the second brake pedal, and its
[owner's handbook, Driving Safety Systems / Brake-steer](https://mclaren-web.s3.eu-west-2.amazonaws.com/files/Europe-English.pdf)
describes applying the inside rear brake to assist turning. The simulator
uses an explicit experimental controller, not McLaren's proprietary logic
or a reconstruction of the original driver's pedal inputs.

## Command and limits

- Positive steering and lateral acceleration select rear-left; negative
  signals select rear-right. Opposing signals, straight travel, or speed
  below 2 m/s disable assistance. No future track information reaches this
  controller.
- Lateral demand ramps from zero at 0.2 m/s² to full demand 9.81 m/s² above
  that threshold. Steering demand ramps from zero at 0.003 rad to full demand
  0.03 rad above that threshold. Both ramps are clamped to [0, 1].
- Requested auxiliary yaw moment is `strength * yaw_limit * lateral_ramp *
  steering_ramp`. At 50% strength and a 120 N m limit, maximum requested
  auxiliary yaw is 60 N m. Wheel brake force is yaw moment divided by half
  the rear track width.
- Extra force is limited by the inside wheel's estimated remaining combined
  grip and remaining hydraulic supply after main-pedal braking. Full solved
  tire utilization, steering, footprint, force and yaw equilibrium checks
  still determine whether an operating point is accepted.

The main pedal retains its selected front/rear force split. Adding an
auxiliary rear brake shifts the **combined** braking split rearward. This is
intentional for this test; it differs from the independent-allocation mode,
which preserves total front/rear preference. Its supply ceiling is inherited
from the configured rear hydraulic authority. The extra actuator is idealized;
this is not a hydraulic plumbing design or a balance-bar-only implementation.

## Throttle overlap and interpretation

Assistance can operate during braking, steady cornering and acceleration.
Rear drive torque is divided equally, then the auxiliary brake is subtracted
at the selected wheel. The equilibrium must provide the extra propulsion
needed to overcome that drag, within the same motor force and power limits.
There is no free yaw moment. The net longitudinal tire force can remain
positive while that wheel's brake is applied, so playback highlights the
auxiliary brake separately from the net-force arrows. JSON and CSV exports
also include the four auxiliary forces.

The current-state lateral-force observer is approximate. Assistance follows
the signal ramps even without measured understeer: there is no closed-loop
yaw-error/understeer detector, wheel-slip dynamics, differential torque-bias
model, brake-temperature state or human second-pedal model. Do not interpret
a time difference as validated stability, driver benefit or an optimized
control law. Braking-zone colors continue to describe main-pedal deceleration;
use the separate auxiliary indication to inspect brake/throttle overlap.

## Reproduce

```powershell
.\.venv\Scripts\python.exe -B -m unittest test_fiddle_brake -v
.\.venv\Scripts\python.exe lap_time_study.py --profile illustrative --track figure-eight --configurations fiddle --output-dir outputs/fiddle_brake
```

The existing `active` two-profile study and `all` six-profile passive study
retain their previous strategy selections. The new `fiddle` study is a
separate pair with the same conventional baseline.
