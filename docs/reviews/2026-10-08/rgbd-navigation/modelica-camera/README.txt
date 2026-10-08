Sequential replay using the actual Modelica synthetic camera
8 October 2026

The flight scenario runner now generates all pixel/depth observations through
SLAM.Simulation.LandmarkCamera exported with Rumoca 0.10.2. The C adapter only
copies inputs and outputs. Python chooses the trajectory, explicit error draws
and availability schedules, drives the Modelica camera and filters, and performs
independent offline evaluation. Projection, visibility, filter prediction,
correction, runtime NIS and fusion decisions all execute from Modelica sources.
No NavigationScore or navigationErrors implementation is added to the library.

The fresh qualification freezes both camera and navigation source/artifact
identities, then runs eight noise seeds 20271061 through 20271068 across GPS,
GPS denied and GPS loss/return. Every scenario retains the same two-second
camera outage as the original declaration. All 24 paired cases pass, all
144000 raw full15 covariance rows are symmetric and Cholesky-positive, all
available observations are accepted and no unavailable observation is fused.
Modelica camera visibility is checked before using every generated observation.

qualification.json, result.json and the sensor export/build logs retain the
exact source identities, generated-camera identity and all per-case outcomes.
Full captures and state/covariance traces remain under the owned scratch
directory rgbd-navigation/qualification-3. The earlier evidence in the parent
directory is retained with its original source identity and camera-generation
scope. It is not silently replaced by this run.

The scene is still an exactly known world-referenced four-landmark map, zero
transport delay, explicit constant biases and declared synthetic noise. It
does not represent uncertain-map SLAM or a calibrated real D435 flight. The
same observability, correlation and native-comparison limits documented in the
parent README apply. Near equality of sufficient loose and raw tight updates
does not demonstrate either coupling method's general superiority.
