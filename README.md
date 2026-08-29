# modelica_models

[![Modelica library checks](https://github.com/CogniPilot/modelica_models/actions/workflows/ci.yml/badge.svg)](https://github.com/CogniPilot/modelica_models/actions/workflows/ci.yml)

This project badge reports repository CI; it is not a Modelica Association
certification. The checked rules and their MSL rationale are documented in
[CONTRIBUTING.md](CONTRIBUTING.md).

Reusable Modelica building blocks for rigid-body simulation, estimation,
control, and verification.

This repository is the aerospace engineering workspace for CogniPilot vehicle
development. Vehicle physics, flight-control source models, named vehicle
parameterizations, avionics-facing plant interfaces, missions, and
qualification criteria live here. Firmware repositories consume exported
artifacts; they do not own alternate copies of these models.

## Layout

- `Avionics/`: transport-independent sensor, navigation, and estimator
  lifecycle contracts shared by drivers, estimation, and control.
- `Estimation/`: structured estimator prediction and correction functions,
  including the replaceable `StrapdownINS` ESKF and manifold UKF.
- `LinearAlgebra/`: dimension-generic matrix algorithms used by estimation
  and control code.
- `Polynomials/`: dimension-generic Hermite construction, derivative
  evaluation, and integrated derivative-cost matrices.
- `LieGroups/`: Lie groups including SO(2), SO(3), SE(2), SE(3), and SE_2(3),
  with replaceable quaternion, DCM, MRP, and Euler rotation representations.
  All 12 axis sequences are available in both body-fixed and space-fixed form,
  including `B232` and `S123`.
- `Geodesy/`: reusable local-frame and geodetic conversion helpers, including
  the East-North-Up projection and its inverse about a `GeodeticOrigin` used to
  turn global lat/lon/alt waypoints into local-frame references.
- `RigidBody/`: reusable six-degree-of-freedom rigid-body dynamics.
- `MathUtilities/`: shared clipping, filtering, angle, rate, and norm helpers.
- `Control/`: reusable control laws and sampled controller building blocks,
  including the multirotor SE_2(3)/SO(3) log-linear controller and the
  `Multirotor.RateLoop` and `Multirotor.Allocation` inner-loop building blocks.
- `Planning/`: forward-only bounded-curvature path planning, including all six
  classical Dubins path families.
- `Vehicles/Templates/`: parameterized fixed-wing and quadrotor plants.
- `Vehicles/Cubs2/` and `Vehicles/Rdd2/`: named parameterizations,
  flight-control models, avionics plant interfaces, and qualification missions.
  RDD2 includes both its cascaded sampled controller and a thin vehicle
  parameterization of the reusable log-linear controller.
- `tools/`: non-library Python orchestration for validation, qualification,
  export, CI caching, and reports. Nothing under this directory is part of the
  Modelica package API.

The repository keeps automation outside its Modelica package trees, following
the same separation used by the
[Modelica Standard Library](https://github.com/modelica/ModelicaStandardLibrary).
The structure check validates `within` declarations, `package.order` coverage,
and package metadata without requiring Nix.

## Navigation estimator boundary

`Avionics` is the pure-Modelica boundary between sensor drivers,
navigation algorithms, and control. Its IMU, motion-capture, GPS, and optical-
flow records use physical quantities with explicit ENU-world and FLU-body frame
conventions; transport adapters may map Synapse/FlatBuffers messages into these
records, but this library has no dependency on those message definitions.

Every estimator publishes `Avionics.NavigationEstimate`. It
contains only algorithm-independent physical state: position, velocity,
acceleration, body and world angular velocity, and mutually consistent
quaternion, direction-cosine-matrix, and roll-pitch-yaw attitude forms. Internal
bias states, tangent ordering, and covariance stay private because they are not
comparable across arbitrary filter implementations.

`Estimation.StrapdownINS.PartialEstimator` is the common aided inertial-
navigation interface. `StrapdownINS.ESKF` implements a 15-state local
right-error ESKF, while `StrapdownINS.UKF` implements a 31-point manifold UKF
over the same position, velocity, attitude, and IMU-bias state. The ESKF name
is intentional: additive bias states mean that calling it an invariant Kalman
filter would overstate its mathematics. Both filters accept identical sensor,
noise, initialization, and terrain-plane assumptions so their closed-loop
comparison is meaningful.

Sensor `timestamp_s` is capture time. `valid` may remain true while a usable
sample is held; `fresh` pulses for one estimator tick when a new sample arrives,
preventing a slow sensor value from being fused repeatedly. Latency compensation
belongs in a reusable fixed-lag wrapper over a private estimator-backend contract:
the wrapper buffers backend snapshots and IMU increments, corrects at the capture
time, and replays prediction to the present. This keeps replay logic shared while
allowing each backend to retain its own opaque state and uncertainty model.

The CUBS2 deployment boundary is intentionally narrower than the closed-loop
test model. `Vehicles.Cubs2.OuterLoop` is the deployable Modelica controller;
it sends pilot-style commands to an onboard inner-loop stabilizer whose
proprietary implementation is unavailable to this project. Closed-loop CUBS2
tests use `OnboardStabilizerSurrogate` only as an explicit simulation fixture.
Those tests verify the open outer loop and its assumed boundary behavior; they
do not establish equivalence to, or qualify, the unseen stabilizer.

The vehicle library is execution-neutral. Its names describe physical or
control meaning only. Tooling outside the library decides whether a model is
executed directly or exported for another runtime.

`Control.Multirotor.LogLinear` contains the alternate controller as one
documented package. It exposes the left-invariant state error, bounded position
integral, SE_2(3) outer loop, SO(3) attitude loop, and sampled controller model.
`Vehicles.Rdd2.LogLinearController` supplies only the RDD2 parameters. A
differentially flat trajectory connects directly through its world-frame
position, velocity, acceleration, and yaw-reference quaternion.

GPS waypoint navigation runs entirely on this local-frame controller. The
reusable `Planning.Bezier.waypointTrajectory` builds a piecewise-septic
differentially flat trajectory that passes through each waypoint at a commanded
velocity (zero for rest-to-rest), feeding world-frame position, velocity, and
acceleration to the log-linear controller; its collective thrust and body-rate
command pass through `Control.Multirotor.RateLoop` and
`Control.Multirotor.Allocation` onto the four rotors.
`Vehicles.Rdd2.Test.TruthWaypointMission` flies a truth-feedback comparison
baseline. `Vehicles.Rdd2.Test.WaypointMission` takes off, flies a box, and
lands in the local East-North-Up frame using the optical-flow-aided ESKF, and
`Vehicles.Rdd2.Test.GlobalWaypointMission` authors the same box in
latitude/longitude/altitude, projects it back through a fixed mission origin,
and flies it using the GPS-aided ESKF. Parallel `UkfWaypointMission` and
`UkfGlobalWaypointMission` models fly the same missions with the UKF. All four
aided tracks are compared with the truth-feedback baseline so controller/plant
failures can be distinguished from estimator or aiding failures.

The optical-flow sensor traces a normalized image feature grid against the
configured plane `n' p = d`; it therefore does not require that the observed
surface be aligned with local vertical. Raw line-of-sight flow contains both
translation and camera rotation. The compensated velocity and slant-range
measurement use the same plane geometry as each filter's observation model.
The default plane is horizontal because that is a common local-terrain
approximation, and an inclined-plane regression prevents that default from
becoming a hidden assumption.

## Vehicle development

An aerospace change follows one source path:

1. Edit a reusable plant under `Vehicles/Templates/`, or a named vehicle,
   controller, and mission under `Vehicles/Cubs2/` or `Vehicles/Rdd2/`.
2. Run the named vehicle qualification. Each mission exercises the controller
   and plant together and writes reviewable traces and reports beneath
   `artifacts/vehicles/`.
3. Export the same controller source as eFMI Production Code.
4. Export the same avionics plant interface as FMI 3 Co-Simulation.
5. Let the consuming firmware repository integrate those generated artifacts.

Landing-gear contact is intentionally eventful. Direct simulation preserves
the touchdown relations. The current source-FMU profile rejects plant exports
that contain event/action partitions; it must continue to fail closed until
the FMI execution kernel preserves those partitions and its conformance tests
cover them.

Install the cross-platform repository command from the checkout:

```bash
python -m pip install -e .
modelica-models doctor
```

Python owns the task definitions and process orchestration. Run qualifications
and exports with the same command on Linux, macOS, or Windows:

```bash
modelica-models qualify cubs2
modelica-models qualify rdd2
modelica-models qualify all

modelica-models export cubs2-controller
modelica-models export cubs2-plant
modelica-models export rdd2-controller
modelica-models export rdd2-estimator
modelica-models export rdd2-plant
```

Rumoca's CLI and matching Python bindings are required for qualification;
OpenModelica or a working Docker installation is required for the full test
suite. Tool locations can be overridden with `MODELICA_MODELS_RUMOCA`,
`MODELICA_MODELS_OMC`, and `MODELICA_MODELS_DOCKER`. Set
`MODELICA_MODELS_ROOT` when running outside the checkout.

Nix is optional. It only provides pinned compilers, Python dependencies, and
environment variables; it does not define tasks or task-specific wrappers:

```bash
nix develop
modelica-models qualify rdd2

# One command without entering the shell:
nix run . -- test
```

CI runs the CUBS2 and RDD2 qualifications as independent jobs. RDD2 flies the
truth-feedback baseline plus optical-flow- and GPS-aided ESKF and UKF missions. Each
job uploads its traces, PNG plots, and a self-contained HTML report, with a
direct artifact link in the GitHub Actions job summary. The two jobs use
`fail-fast: false`, so both vehicles are qualified even if one fails.

RDD2 sensor noise is deterministic and compiler-portable. Each Gaussian sample
is produced by a Box-Muller transform of indexed uniform fixtures, then scaled
from the exact measurement covariance or continuous IMU/bias-walk spectral
density supplied to the selected filter. This is deliberately not the stateful
Modelica Standard Library/OpenModelica Xorshift sequence. Qualification checks
the empirical variance and cross-correlation against the declared assumptions;
`enableSensorNoise=false` provides a noiseless diagnostic run.

Arguments after `--` are forwarded to the selected qualification, exporter,
or report implementation.

### RDD2 optical-flow and GPS mission trace reports

These commands render the explicit optical-flow and GPS mission models and
write a PNG, self-contained HTML report, JSON observation report, and selected
CSV under `artifacts/vehicles/rdd2/mission-plots/<mode>/`. They report model
traces; they do not qualify a physical aircraft or issue a flight-acceptance
verdict.

Invoke the reporter through the Python orchestration command:

```bash
modelica-models mission-report optical -- --engine omc
modelica-models mission-report gps -- --engine omc
```

Live simulation refuses tracked modifications or untracked files among
Modelica sources, scenario TOML, `Resources/`, and `package.order`, and checks
again after simulation so a report cannot label dirty inputs with the clean
revision. Provenance records resolved compiler and Python paths and their
SHA-256 digests. Exact receipted compiler and model revisions are mandatory
whether the tools came from Nix or a native installation.

The optical command always selects `Vehicles.Rdd2.Test.WaypointMission`; the
GPS command always selects `Vehicles.Rdd2.Test.GlobalWaypointMission`. Defaults
match the collected model evidence: 45 seconds with a 5 ms output step. The
GPS report shows the existing 0.5 m model navigation-error criterion as a
diagnostic. The optical report shows absolute-position drift without applying
a threshold because velocity-only optical aiding does not observe absolute
position.

The frozen Rumoca flight compiler revision `9860c307` is not available from the
current optional environment, so live Rumoca reporting fails closed instead of
silently using a different compiler. A receipted Rumoca CSV can still be
replayed:

```bash
modelica-models mission-report optical -- \
  --engine rumoca --rumoca-csv /path/to/trace.csv \
  --rumoca-receipt /path/to/trace.receipt.json
modelica-models mission-report gps -- \
  --engine rumoca --rumoca-csv /path/to/trace.csv \
  --rumoca-receipt /path/to/trace.receipt.json
```

Replay refuses a CSV without a sidecar whose JSON fields exactly identify its
mode, model, compiler, model revision, duration, cadence, and SHA-256 digest.
The receipt schema is:

```json
{
  "schema": "rdd2-mission-trace-receipt-v1",
  "engine": "rumoca",
  "mode": "gps",
  "model": "Vehicles.Rdd2.Test.GlobalWaypointMission",
  "compiler": {
    "name": "Rumoca",
    "identity": "rdd2-flight-freeze-2-9860c307",
    "revision": "9860c30781242ff65dfcf47b136385ac5ecf4350"
  },
  "model_revision": "a9e5037ab3e57b3fac6ca783c0bdbfdd2b6dd98e",
  "stop_time_s": 45.0,
  "step_s": 0.005,
  "csv_sha256": "<exactly 64 lowercase hexadecimal characters>"
}
```

OpenModelica replay uses the same schema with engine `omc`, identity
`OpenModelica/a96aa1a682c463b0fd2d285b486c09a8b7fe496d`, and revision
`a96aa1a682c463b0fd2d285b486c09a8b7fe496d`. Trace validation also requires
finite samples, time zero, strict ordering, exact cadence, and complete
requested coverage. Use `--stop-time` and `--step` to match a shorter receipted
trace. These commands are currently Linux-only.

## Mission trajectory logs

Named vehicle missions write a canonical, execution-neutral trajectory log:

```text
time_s,x_m,y_m,z_m,roll_rad,pitch_rad,yaw_rad
```

The trajectory comparison tool treats one such log as the gold standard,
interpolates any number of named candidate logs at its timestamps, renders
full trajectory/component/error plots, and enforces duration, position,
altitude, and attitude limits. It intentionally knows nothing about the system
that produced a log:

```sh
modelica-models trajectory-compare -- --help
```

Qualification artifacts remain ignored build outputs under
`artifacts/vehicles/`. A downstream repository can compare generated, bench,
or flight logs by adapting them to this seven-column contract; no downstream
execution terminology or log decoder belongs in this library.

## Testing

All mathematical test definitions are Modelica classes under `Tests/`. The
suite uses Modelica `assert` equations for Lie-group identities, generic
linear-algebra systems, geometric error-state filters, Dubins paths, and analytic rigid-body
trajectories.
No Python source generation or numerical oracle is involved.

Run the complete suite with the installed Python command:

```bash
modelica-models test
```

Individual checks are available through the same interface:

```bash
modelica-models check-structure
modelica-models test omc
modelica-models test rumoca
modelica-models test plots
```

From an uninstalled source checkout, replace `modelica-models` with
`python -m tools.modelica_models_cli`. The runner uses the declared
OpenModelica container when Docker is operational, then falls back to a local
`omc`. It handles paths, temporary directories, process failures, and artifact
validation without platform-specific scripts. Python does not compute test
oracles: mathematical pass/fail checks remain Modelica assertions executed by
OpenModelica and, where currently supported, Rumoca.

The CI command also simulates reproducible planning examples and writes CSV
data plus PNG plots under `artifacts/planning/`. The primary aircraft figure is
a closed figure eight whose two crossing headings share exactly one center
position. Redundant identical poses are coalesced, leaving seven waypoint
occurrences and six nonzero Dubins legs. The initial flight plan is overlaid with
its flyable, continuous-curvature Dubins-polynomial smoothing. A separate plot
checks curvature and curvature-derivative continuity across every leg and
waypoint junction, and a mission-wide signed bank-angle plot shows the
coordinated left/right command. Generated artifacts are ignored by Git and
uploaded by the GitHub Actions workflow.

The Dubins-polynomial coefficients are obtained with a fixed-cost analytical
procedure: one joint unconstrained quadratic solve over the free junction
derivatives, followed by one closed-form true-spatial-derivative repair pass.
The default derivative weights are `{0.3, 1, 0, 50}` through third order. The
CI gallery also renders four paper-style comparisons with the Dubins reference,
optimized path, and nominal segment junctions on common axes.

The plotted example also checks a stated aircraft flight envelope in Modelica:
minimum turn radius, coordinated bank angle, and coordinated bank rate at the
declared flight speed. Thus "flyable" means that those explicit assumptions
pass assertions; it is not inferred from geometric smoothness alone.

To regenerate only the planning artifacts:

```bash
modelica-models test plots
```

CI uses the same Python commands. Its optional Nix layer pins compiler binaries
and caches only the Rumoca runtime when that pin changes. OpenModelica runs the complete
assertion simulation using the pinned container image, with a local `omc` as
the fallback when Docker is unavailable. Rumoca independently parses, resolves, flattens, and lowers
the same `Tests.All` model to DAE form, then executes SO(2) and SE(2) assertion
smoke models with its simulation backend. The complete assertion model remains
the OpenModelica simulation authority because Rumoca does not yet structurally
lower every aggregate static assertion and array-valued dynamic equation.

## Replaceable rotation representations

`SE3.Generic` and `SE23.Generic` accept any package implementing
`SO3.Interfaces.PartialRotation`. Concrete quaternion, MRP, DCM, and Euler-B321
packages are provided. A specific Euler sequence can be selected directly:

```modelica
package VehicleState = LieGroups.SE23.Generic(
  redeclare package Rotation = LieGroups.SO3.EulerSequences.B232);
```

Quaternion and DCM representations are generally preferable for global
simulation. Euler representations are useful at interfaces where a named
sequence is required. Proofs using local log coordinates should explicitly
establish the chart-domain guard supplied by `LieGroups.Analysis`.

## Verification-oriented APIs

The library keeps simulation equations and proof expressions aligned:

- `RigidBody.stateDerivative` is the pure vector field used by
  `RigidBody6DOF`.
- `RigidBody.mechanicalEnergy`, `wrenchPower`, and `powerBalanceResidual`
  expose the lossless passivity identity.
- `LieGroups.Analysis` supplies coordinate-independent attitude potential,
  principal-log domain checks, and dimension-generic tangent-space quadratic
  candidates.
- `LinearAlgebra.contractionResidual`, `quadraticForm`, and
  `quadraticFormDerivative` provide dimension-generic certificate expressions.
- The ESKF exposes its nominal discrete map, tangent transition, process
  covariance, measurement map, measurement covariance, and local residual
  as separate pure functions for reachable-set propagation.

These functions produce certificate quantities; they do not claim a proof by
themselves. Bounds on chart validity, disturbances, numerical integration, and
the sign of the resulting matrix inequalities remain explicit proof
obligations.

## Dubins path planning

`Planning.Dubins` computes the globally shortest member of the six classical
forward-only, constant-turn-radius path families: LSL, RSR, LSR, RSL, RLR, and
LRL. Positions are Cartesian x/y coordinates and headings are in radians.

The classical `plan` search includes RLR/LRL by default and accepts
`allowThreeTurnPaths=false` when those reversal-heavy paths are inappropriate.
Aircraft trajectory examples disable them by default and automatically select
the shortest feasible LSL, RSR, LSR, or RSL path, so callers do not select a
family manually.

```modelica
Planning.Dubins.Path path;
Planning.Dubins.Pose midpoint;
equation
  path = Planning.Dubins.plan(
    {0.0, 0.0}, 0.0,
    {10.0, 10.0}, 0.5 * 2.0 * asin(1.0),
    5.0);
  midpoint = Planning.Dubins.evaluate(path, 0.5);
```

The path record exposes its selected family, physical length, turn radius, and
three normalized segment lengths. Turn segment lengths are angles in radians;
straight segment lengths are distance divided by turn radius. This makes the
piecewise curvature and segment domains explicit for tracking and reachability
analysis.

### Smooth transverse-polynomial paths

`Planning.DubinsPolynomial` represents a smooth trajectory as a piecewise
polynomial left-normal offset from an underlying Dubins path. Polynomial
abscissas use physical nominal path distance rather than normalized segment
coordinates, keeping derivative costs dimensionally consistent between arcs
and straight segments.

The evaluator accepts any coefficient count and returns:

- position and heading;
- true unit-speed spatial derivatives through third order;
- signed curvature and curvature derivative;
- transverse offset and the offset-to-nominal metric scale.

`smoothOffsets` constructs a septic seed that fixes position and heading at
the endpoints and segment junctions while making curvature and curvature rate
continuous. `derivativeCost` and `Polynomials.derivativeCostMatrix` provide the
quadratic objectives needed to penalize offset, curvature-related derivatives,
and higher derivatives. The coordinated-flight helpers map curvature and its
metric derivative to fixed-speed roll angle and roll rate.

The supplied smoothing seed constrains transverse offset and its first
derivative to zero at nominal segment junctions. This makes its C3 continuity
exact. An optimizer that permits nonzero junction offsets must additionally
apply a nonlinear true-derivative repair iteration; use
`junctionContinuityResidual` as the acceptance certificate. Every candidate
must also retain a positive `metricScale`, since a zero value indicates a
singular or locally reversed offset curve.

## License

This repository is licensed under the Apache License, Version 2.0. See
[LICENSE](LICENSE). Every Modelica source here is original CogniPilot content
under that license; the repository carries no third-party-derived source.

`Ekf2/`, a statement-for-statement transcription of PX4-Autopilot's ekf2
module and therefore BSD-3-Clause, was relocated out of this repository for
that reason. It is maintained separately, outside CogniPilot, as a PX4-parity
oracle for cross-validating the estimators here. Oracles are consumed only as
external comparators, checked out at benchmark time with only their outputs
compared, never vendored back in. The estimators in this repository derive
from papers and specifications, with citations, and not from that
transcription.

[NOTICE](NOTICE) indexes the third-party content redistributed here: the
NOAA/NCEI and BGS World Magnetic Model in `Geodesy/WMM2025/`, and the
CogniPilot Python implementations that three packages follow.
