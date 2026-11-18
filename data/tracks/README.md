# Real circuit source data

The three CSV files in `tumftm/` are unchanged copies from [TUM's racetrack database](https://github.com/TUMFTM/racetrack-database), pinned to commit `e59595d1f3573b30d1ded6a08984935b957688e0` and retrieved on November 2, 2026. TUM published these tracks in May 2020; the pinned 2021 commit updates documentation. Use the **2020 dataset** label. These files do not establish the current layout, a 2027 layout, or a Formula SAE Michigan course.

| Circuit | Source polyline length, including closure | Source order |
| --- | ---: | --- |
| Monza | 5790.202 m | Clockwise |
| Montreal / Circuit Gilles-Villeneuve | 4357.511 m | Clockwise |
| Barcelona-Catalunya | 4649.844 m | Clockwise |

The lengths above are calculated from the supplied coordinates. They are not substituted official lap lengths. All three have negative signed enclosed area in the source x/y coordinate system. Coordinates are in metres with the origin near the start/finish line. Each row is `x_m,y_m,w_tr_right_m,w_tr_left_m`; sampling is approximately 5 m. Catalunya includes the historical final chicane and old Turn 10. Preserve the supplied direction unless the user explicitly chooses a transformed test.

TUM created smoothed centerlines from [OpenStreetMap](https://www.openstreetmap.org) GPS points and estimated left/right widths from satellite imagery. Its documentation warns that accuracy varies and smoothing moves the line away from the exact road midpoint. These are approximate planar layouts, not surveyed racing lines. They contain no elevation, banking, grip map, kerb profile, braking telemetry, or measured brake markers. Tight-corner curvature depends on interpolation and sample spacing.

The simulator uses its stated interpolation and the user's **uniform road-width override**. It does not claim that width setting is the measured circuit width. Source width columns are preserved for provenance and future asymmetric-boundary support. Braking zones must be calculated for the simulated vehicle and settings; Formula 1 braking figures are not transferred into the Formula Student model. A full-scale circuit may contain many flat-out sections with this vehicle's speed limit.

## Attribution and license notices

Track data processing: **TUM, Institute of Automotive Technology**, repository contact Alexander Heilmeier. TUM credits Andressa de Paula Suiti for the satellite-image track-width extraction algorithm. Underlying map data: **Copyright OpenStreetMap contributors**, available under the [Open Database License (ODbL) 1.0](https://www.openstreetmap.org/copyright).

The upstream repository supplies **GNU LGPL version 3** in [tumftm/LICENSE](tumftm/LICENSE). That license incorporates GPL version 3; a complete copy is included at [tumftm/COPYING-GPL-3.0.txt](tumftm/COPYING-GPL-3.0.txt). The underlying OSM license is also included at [tumftm/COPYING-ODbL-1.0.txt](tumftm/COPYING-ODbL-1.0.txt). These notices apply to the imported data; no new license for the simulator code is declared here. Preserve these notices and source availability when redistributing data or derived layouts.

The original [TUM README](tumftm/README.md) is included unchanged. [manifest.json](tumftm/manifest.json) records pinned download URLs, retrieval date, byte-level SHA-256 hashes, layout notes, and computed source lengths. The original files remain separate from runtime interpolation or simulation outputs.
