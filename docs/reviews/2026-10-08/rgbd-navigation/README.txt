Sequential Modelica loose and tight fixed-map navigation
8 October 2026

Both visual paths now run through complete sequential ESKF prediction and
GPS/camera correction from Modelica source, exported with Rumoca 0.10.2.
Tests.VisualNavigationReplay owns all estimator mathematics. The C harness
only copies packets and the previous generated state/covariance. Python
generates synthetic measurements and independently computes physical errors
and right-tangent NEES. This is not a Python reimplementation of either filter.

qualification.json freezes source hashes and the declaration before capture
generation, identifies generated artifacts and records the actual commands.
result.json retains all 24 paired cases, including physical RMS, outage/return
windows, NEES15, dimension-specific NIS, input/output hashes and image bounds.
Raw captures and every state/covariance row remain under the owned scratch
directory rgbd-navigation/qualification-2. Reproduction is documented in
docs/slam-models.txt. Build outputs are intentionally outside the repository.

Experiment
----------
Eight independent sensor/prior noise draws, seeds 20271041 through 20271048.
Each draw is reused unchanged for GPS available, GPS denied and GPS loss/return,
and both visual paths. These are eight independent draws, not 24 or 144000.
Each flight lasts 30 seconds with a 100 Hz IMU, 5 Hz GPS and 10 Hz RGB-D camera.
The transition loses GPS over [10,20) seconds. Every case loses the camera over
[14,16) seconds, so the denied and transition cases temporarily lose both aids.
All available measurements are accepted, none are fused while unavailable,
and the filter continues prediction through the outages without reseeding.
Camera updates per path/flight: 280. GPS updates: 150, 0 and 100 respectively.

Four exact world-referenced ground landmarks are observed using a calibrated
synthetic pinhole RGB-D camera with a nonzero body/camera lever arm. All noisy
observations remain inside a 640x480 image and have positive optical depth.
This is not a calibrated D435 sensor noise model or recorded flight dataset.
The camera has full12 covariance with pixel/depth and cross-feature terms.
Loose retains a complete sufficient six-dimensional pose covariance; tight
uses the same twelve raw pixel/depth residuals. Both use the same initial
state/covariance, constant random biases, physical IMU/GPS noise and timing.
Transport delay is zero, random-walk noise is zero and gates are disabled.

Results
-------
All 24 paired cases pass the frozen numerical equivalence and covariance
checks. All 144000 raw full15 covariance rows are exactly symmetric and admit
Cholesky factorization without symmetrization, jitter or other repair.
Generated runtime error status is zero throughout. Maximum loose/tight nominal
entry difference is 3.923e-6; covariance entry difference is 1.062e-7. The first
number spans mixed state units; physical RMS values below are the useful
performance measures. These small differences do not establish a winner.

GCC reports possible uninitialized generated temporaries; build.log retains
those warnings. initialization-result.json checks both zero and pattern
automatic-variable initialization builds against every frozen replay. Both
builds reproduce all 24 paired output traces byte for byte. This sensitivity
check finds no initialization-dependent published output on these cases; it
does not prove absence of undefined behavior on all generated execution paths.

Aggregate whole-flight position RMS (metres), equally weighting all eight draws:
                     loose        tight
  GPS                0.022293     0.022293
  GPS denied         0.030759     0.030759
  GPS loss/return    0.029454     0.029454

Aggregate whole-flight velocity RMS (metres/second):
                     loose        tight
  GPS                0.039789     0.039789
  GPS denied         0.047762     0.047762
  GPS loss/return    0.044459     0.044459

Mean NEES15 is approximately 15.72, 15.84 and 15.79 for either path. Mean camera
NIS is 5.93--5.97 for the six-dimensional loose statistic and 11.94--11.98 for
the twelve-dimensional raw statistic. Mean GPS NIS6 is 6.25 in GPS flight and
6.21 in transition. These descriptive means are compatible with the intended
linearized model but do not certify consistency from eight noise draws.
Time samples within each flight are correlated; no independent-tick confidence
interval or universal error-bound assertion is made.

Scope and retained failures
---------------------------
The initial development export hit Rumoca ED010 when a discrete result was
assigned before a conditional correction overwrote it. Explicit predicted,
GPS-corrected and camera-corrected stages compile successfully without changing
any production filter equation. No compiler fix is claimed.

The first numerical replay passed its algebra gates but a subsequent image
audit found observations outside the image. development-visibility-audit.json
retains that disqualification. The admitted run uses a revised visible scene,
an explicit visibility gate and fresh seeds. Earlier outcomes are not presented
as camera flight evidence or independent validation of the revised scene.

This is fixed-known-map visual aiding. Its landmarks provide absolute world
position information during GPS denial. Those errors cannot be compared to
the existing nadir-flow native EKF2/EKF3 campaign as if the sensors or gauges
were equivalent. The loose baseline preserves sufficient information at the
common linearization, so near equality is expected; existing GNC visual
compression theorems address that conditional algebraic statement.

Live SLAM still requires map/reference uncertainty, shared-IMU/GPS correlation,
association and image processing, delayed-frame fusion, loop closure and a
single authoritative navigation/map session. This replay does not qualify
those components or the imported full SLAM graph, measure fair runtime cost,
prove nonlinear equivalence, or demonstrate superiority over native firmware.
