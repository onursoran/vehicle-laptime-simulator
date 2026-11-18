# Vehicle lap-time predictor

Compare front/rear braking with independent-wheel assistance on synthetic
tracks and historical real circuit layouts, using an illustrative Formula
Student-style research vehicle. Its assumptions are documented in
[LAP_TIME_MODEL.md](LAP_TIME_MODEL.md). This is a quasi-steady research
comparison, not a validated real-car lap model.

## Open the simulator

Use Python 3.10 or newer (development and tests use Python 3.13). After cloning
the repository, run these commands from the project folder on Windows:

```powershell
py -3 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe simulator.py
```

On macOS or Linux:

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python simulator.py
```

For later launches, only the final command is needed. Generated results under
`outputs/`, temporary files and the local virtual environment are excluded
from Git. Track data and their original license notices are included.

Open **http://127.0.0.1:8765** and refresh after a code update. Stop an old
server with Ctrl+C before restarting it; two servers cannot share this port.
The server is local to this computer.

1. Choose a car profile and track. The track preview loads without calculating
   laps. Road width and front/rear wheel track remain adjustable.
2. Set front braking preference and left/right assistance strength separately.
3. Click **Run comparison**. The default compares front/rear braking with
   independent assistance. Select all six systems to include fixed passive valves.
4. Watch progress or cancel. The first completed profile appears while remaining
   profiles calculate. A failed configuration is labeled unavailable.
5. Select a calculated profile, seek along it, or play the lap. The map and
   speed plot show its light, medium and hard braking sections.

For the McLaren-inspired test, select **Conventional + inside-rear fiddle
brake** under Compare. Normal braking stays available, with additional
rear-left braking in left turns and rear-right braking in right turns.
Strength and yaw limit control the assistance; orange wheel outlines and
**Aux brake N** show its activity, including under throttle. Start with the
figure-eight to see both sides. See [FIDDLE_BRAKE.md](FIDDLE_BRAKE.md) for the
control law, limits and command-line test.

Changing inputs cancels obsolete calculations. Geometry changes request a new
preview; other changes mark existing results as stale. Completed laps are
reused when their physical inputs match. Changing assistance strength does
not recalculate the conventional baseline. Playback moves the car and cursor
without rebuilding the full circuit or speed graph.

The **illustrative** profile uses research placeholder values for mass,
geometry, brakes, tire grip and drivetrain. They are listed in
[LAP_TIME_MODEL.md](LAP_TIME_MODEL.md) and exported with every result.
Team-specific car data is not included in this public version.

Monza, Montreal and Barcelona-Catalunya use approximate **2020** centerlines;
Catalunya includes the old final chicane and Turn 10. They retain full metric
scale. Presets are 12 m uniform width / 4 m spacing for real circuits and
3 m / 2 m for synthetic tracks. Width is an experimental override, not a
surveyed boundary. Mirroring reflects the geometry; it does not reverse travel.
See [track provenance and licenses](data/tracks/README.md).

## Command-line runs and verification

```powershell
.\.venv\Scripts\python.exe lap_time_study.py --profile illustrative --track monza --configurations active --output-dir outputs/monza
.\.venv\Scripts\python.exe lap_time_study.py --profile illustrative --track real --output-dir outputs/real_tracks
.\.venv\Scripts\python.exe lap_time_study.py --profile illustrative --track all --include-mirrors
.\.venv\Scripts\python.exe -B -m unittest discover -v
.\.venv\Scripts\python.exe benchmark_simulator.py
```

The app benchmark requires the local server. It records preview latency,
time to first profile, full comparison time and cache reuse under
`outputs/performance`. See [PERFORMANCE.md](PERFORMANCE.md) for the changes.

The browser-script behavior test also runs independently with Node.js 22 or
newer: `node test_simulator_ui.mjs`. Its bundled fixture is for UI testing;
numerical validation runs through the Python tests above. Historical timing
and lap artifacts mentioned in the documentation are generated locally and
are not included in a fresh clone.

CLI output defaults to `outputs/lap_time`; use `--output-dir` to keep runs
separate. Each comparison includes inputs, provenance, lap summaries,
per-segment CSVs, braking-zone JSON and plots. Refine spacing before interpreting
small time differences. Saved results with an earlier source fingerprint must be regenerated before
the current audit can validate them.

Audit a selected export with:

```powershell
.\.venv\Scripts\python.exe validate_lap_results.py --comparison outputs/monza/monza-2020-dataset/comparison.json
```

The default validator expects the complete synthetic three-track/mirror suite.
Regenerate that legacy suite with the explicit illustrative command above.
[LAP_TIME_MODEL.md](LAP_TIME_MODEL.md) explains equations and limits;
[BRAKE_CONTROL.md](BRAKE_CONTROL.md) covers the separate timed controller study
and [BRAKE_BIAS.md](BRAKE_BIAS.md) the earlier experiments.

Runtime dependencies remain NumPy and Matplotlib. Car values and track data
are bundled locally; no PDF tools, network download, account or browser
framework is needed. `outputs/braking-on-track.html` is the earlier snapshot
viewer; use the local server for the current app.
