# Track loading and calculation performance

The old viewer waited for six complete lap calculations before displaying
any circuit. During playback it rebuilt the full SVG track, speed graph,
force inset and table every frame. Its solver also recreated whole-track
distance and curvature-gradient arrays during each segment probe.

The current implementation changes these paths:

- `/api/preview` returns geometry and car dimensions without a lap solve or
  compute lock. The initial page and track changes request this immediately.
- The page does not automatically start an expensive comparison. Its default
  comparison contains the conventional and independent systems; all six
  remain selectable.
- `/api/jobs` runs calculations on one background worker, publishing the first
  completed profile and progress while later configurations run. Versioned
  polling avoids repeatedly downloading unchanged profile arrays. Large
  responses use gzip when the browser supports it.
- Cancelled or superseded work stops cooperatively during speed searches and
  segment propagation. A slow or infeasible lap no longer hides track geometry.
- The server caches 32 completed laps by their physical inputs. A change to
  assistance strength reuses the conventional result. Exact request reuse is
  bounded to eight jobs; the pending queue is bounded to four.
- Track segment geometry is prepared once per solve. Drive-mode equilibria
  are shared across brake strategies using exact keys and a bounded 32,768-entry
  cache. Brake solutions and fallbacks always retain their actual strategy.
- Playback updates car transforms, the cursor and changed numerical cells.
  Static SVG paths are rebuilt only when data, selected profile or container
  width changes. Width-guarded resize handling and fixed SVG heights avoid
  repeated layout/redraw feedback. Distance lookup uses binary search.

No tolerance, physical feasibility check or start/midpoint/end sampling was
removed. Regression tests compare cached drive states with every brake
configuration and compare full lap outputs against the previous geometry
evaluation path. Tests also cover cancellation, stale responses, independent
preview access, baseline reuse, and aerodynamic load conservation.

## Measurements and reproduction

`outputs/performance/monza-solver.json` records the initial isolated geometry
optimization using the original illustrative car: median 2.537 s to 2.279 s
for one Monza profile (three alternating repeats), with exactly equal results.
This was measured before shared drive caching and before switching the app's
default car. It is not an overall speed comparison between car profiles.

`outputs/performance/app.json` records the latest live-server preview and job
timings with a team car profile that is not part of this public version. The first pre-drive-cache run is retained
as `app-before-drive-cache.json`; an interrupted attempt was excluded from
the final report. Timing varies with computer load and power/sleep state.
The app records the source fingerprint and actual settings with results.

The completed November 3 run used source fingerprint `5044c49013bb64b45454`,
that team car profile, 4 m spacing, 12 m road width and the default two systems:

| Circuit | Geometry preview | First lap available | Comparison complete |
|---|---:|---:|---:|
| Monza | 1.487 s | 9.75 s | 17.60 s |
| Montreal | 0.051 s | 15.92 s | 38.15 s |
| Catalunya | 0.063 s | 29.94 s | 49.64 s |

These are observed request times, not latency guarantees. All six lap times
exactly matched the pre-drive-cache report. Changing Monza assistance to 40%
reused the conventional lap and calculated the independent lap in 10.75 s;
repeating that exact request returned in 0.185 s. Circuit timings do not show
a uniform solver speedup, so the primary responsiveness improvement is that
geometry and controls no longer wait for all lap calculations.

Run the server, then:

```powershell
.\.venv\Scripts\python.exe benchmark_simulator.py
.\.venv\Scripts\python.exe benchmark_lap_performance.py --repeats 3 --profile
.\.venv\Scripts\python.exe -B -m unittest discover -v
```

Run timing checks without other numerical studies at the same time. The
geometry microbenchmark clears the shared drive cache before each timed
variant, so whichever runs second receives no cache advantage.

DOM-contract tests execute the real browser script at three container widths.
They verify that preview precedes calculation, partial results/progress and
errors are visible, superseded requests cannot overwrite current data, and
playback causes zero static SVG/table `innerHTML` rebuilds. These tests do
not measure browser frame rate or establish visual layout correctness.
The attempted live Chrome inspection was stopped by the computer-use tool
because it could not reliably identify the current URL. API checks and DOM
tests were completed independently; no visual-browser pass is claimed.
