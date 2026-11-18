# Browser behavior fixture

`simulator.json` is a captured figure-eight comparison used only to exercise
the browser script's rendering, inputs, progress, cancellation and playback.
The test fabricates additional presentation rows from it. These rows are not
numerical predictions, and the captured source fingerprint need not match
the current model. Python tests calculate fresh states for physics validation.

Keeping this fixture in Git makes the UI test independent of local benchmark
outputs, a running server or previously generated files.
