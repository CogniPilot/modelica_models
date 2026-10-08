Common-warmup native and ESKF pilot
=================================

This pilot corrects the startup scope failure in the preceding native innovation
audit. Every filter receives the same stationary capture until takeoff at 120 s.
Flight ends at 167 s; the loss/return scenario withholds GPS samples from 132 s
through 147 s. Both native filters actually fuse GPS before flight and before
loss, stop GPS corrections during the outage, and resume after return. No native
readiness threshold or estimator equation was changed to obtain this result.

Native EKF3 begins GPS position/velocity fusion at 36.7 s and makes 833 of each
before takeoff. Native PX4 begins at physical time 1.12625 s and makes 1189 of
each. In the loss/return case, GPS position fusion resumes 1.02625 s after return
for PX4 and 0.1 s after return for EKF3. These are observed correction epochs,
not merely packet delivery or an active-mode flag. Neither filter makes any GPS
position/velocity correction in the denied scenario. validation.json also
checks no GPS-position correction anywhere in the requested transition outage.

The old 13 s capture was inadequate to establish EKF3 GPS aiding before loss.
Its startup/acquisition scores must not be treated as established GPS transitions.
The longer warmup considerably improves EKF3's observed performance. ESKF still
has lower horizontal and vertical flight RMS than both native filters in every
one of the three pilot scenarios, for each of the four ESKF configurations.
This is one seed and motion, not proof of a general or theoretical ranking.

Capture and configuration
-------------------------

Seed 911, horizontal trajectory parameter 0.6, climb 2 m; independent analytic
truth, 800 Hz IMU, explicit 100 ms optical-flow exposure and independently noisy
camera gyro. GPS, flow, magnetic and pressure transport delays are 110/30/60/20 ms
with no jitter. The actual generated C transport trace supplies all filters.
The native release pins remain PX4 v1.17.0 (d6f12ad1c4f70ad3230afd7d86e971421e02fef4)
and ArduPilot Copter-4.7.1 (dbe792162d06cab66c3475fd5556bf7a120f119e).
Observers are read-only instrumentation in external validation copies.

ESKF horizon and retrodiction entries both use FOH, vector magnetic fusion,
geometric alignment and declared startup-rest pressure calibration. Joint entries
add the optional shared pressure-bias estimator; their horizontal error increases
slightly in several cases while vertical error improves. All are explicit
benchmark configurations, not a claim that RDD2's default policy was changed.
No ESKF algorithm, vehicle default, or native estimator equation changed here.

Accuracy uses the same 100 Hz publication epochs, 120 <= t < 166.7 s. Native
states are interpolated without extrapolation. Outage and return windows are
132-147 s and 147-166.7 s. Covariance uses each filter's actual physical fusion
epoch; native 24D covariance is transformed into the shared 15D navigation/bias
marginal. Each of the six covariance-observer state CSVs matches its corresponding
innovation-observer state CSV byte for byte. No covariance repair is applied.

Flight position RMS (m), followed by mean common-state 15D NEES:

scenario     configuration          horizontal   vertical     NEES
GPS          horizon                  0.0203      0.0322      7.89
GPS          retrodiction              0.0194      0.0372      9.11
GPS          horizon joint             0.0204      0.0294      7.65
GPS          retrodiction joint        0.0195      0.0211      8.84
GPS          native PX4                0.1538      0.1339     39.27
GPS          native EKF3               0.0523      0.0384      2.95
denied       horizon                  0.0689      0.0354      5.01
denied       retrodiction              0.0520      0.0432      6.34
denied       horizon joint             0.0701      0.0306      4.79
denied       retrodiction joint        0.0529      0.0430      6.00
denied       native PX4                0.0946      0.0581      4.85
denied       native EKF3               0.0971      0.0484      2.00
loss/return  horizon                  0.0166      0.0335      6.89
loss/return  retrodiction              0.0179      0.0392      8.37
loss/return  horizon joint             0.0166      0.0298      6.55
loss/return  retrodiction joint        0.0178      0.0242      7.99
loss/return  native PX4                0.1673      0.1207     27.82
loss/return  native EKF3               0.0760      0.0469      2.79

NEES is descriptive here. A smaller value is not automatically more consistent;
native priors, effective R/Q, magnetic policies and height policies still differ.
Temporal samples are correlated. An ensemble consistency or uniform superiority
claim requires more independent conditions and better configuration matching.

Native scalar NIS is retained by sensor, axis, window and observation stage.
Accepted-update NIS is selection-conditioned and is not joint vector NIS.
Candidate/rejection coverage is incomplete; EKF3 three-axis magnetic and auxiliary
terrain/range corrections are not fully observed. This pilot does not export
ESKF NIS or test full augmented 16D pressure-state covariance. All scored ESKF
15D marginals are positive definite. Earlier joint-pressure covariance tests
remain a separate campaign. Native observer logging precludes a CPU ranking from
these runs; ESKF timings are retained only as host measurements.

Clock correction
----------------

PX4 native observer time has a +1 s epoch relative to capture time. The covariance
scorer already removed it; innovation and readiness scorers now do too. The old
frozen innovation report's 2.12625 s first-GPS value was a native-clock value,
physically 1.12625 s, and its PX4 NIS windows were correspondingly shifted. Old
artifacts remain historical. EKF3 has no such offset; its old readiness failure
is unaffected. The new pilot consistently uses physical scoring windows.

Evidence and validation
-----------------------

pilot.json: 18 actual replays, state/input/binary hashes, RMS, ESKF NEES, native
scalar NIS, readiness counts and original campaign source hashes.
native-covariance.json: six native covariance replays, unchanged state hashes,
full observer manifests and native NEES by physical window.
flight-statistics.csv: all 18 flight results including velocity and attitude.
native-scalar-nis.csv: all native scalar NIS groups and coverage counters.
default-parity.json: default raw/exposure captures unchanged, three transport
traces unchanged, and eight frozen ESKF output CSVs byte-identical; shifted GPS
sample exclusion and invalid-offset negative controls also passed.
validation.json: independent postflight checks of provenance, common packet
delivery, PX4 flow API mapping, EKF3 flow serialization, finite full-coverage
states, actual GPS use and native observer state parity. It binds both campaign
artifacts and current tool sources. Driver provenance/receipt checks added after
the replay are verified postflight without repeating the unchanged algorithms.
The pilot manifest includes an unused preexisting input arrivals.csv hash;
validation independently regenerates each actual scenario trace. The reusable
driver excludes that unused file from its input manifest.

48 Python tests, Ruff and git diff --check pass. The current generator was rerun
after the schedule refactor and still produces all default files byte for byte.
CI run 37740558817 remained in progress when checked. Its predecessor 37738312745
passed regression and CUBS2 but failed RDD2 with its qualification step cancelled;
this checkpoint does not claim to resolve the outstanding Rumoca memory issue.

Reproduction entry points
-------------------------

Use $HOME/scratch/modelica_models/readiness-benchmark for captures and builds.
Generate with generate.py CAPTURE --seed 911 --speed 0.6 --warmup-s 120, then
flow_exposure.py --source CAPTURE --output EXPOSURE. Link replay.c against the
unchanged generated Modelica objects with the four recorded benchmark feature
sets, and compile transport_trace.c. Both replay and transport receive a 107 s
mission offset; the original 13 s default remains unchanged.

compare_readiness.py --help lists the capture, ESKF binaries, transport, audited
native innovation core/reference/observer and external adapter paths. Supply
the stable native innovation reference from the preceding dated review and the
native-exposure.json noise reference from 2026-10-07/native-releases.
compare_readiness_covariance.py --help lists the separate audited covariance
cores and manifests; it reuses the exact pilot captures and arrivals and rejects
any changed published state CSV. Large raw traces remain in scratch. Source and
binary hashes in the evidence identify the supplied validation builds.

Next work: independent seeds, motions, climb heights, sensor disturbances and
outage durations; matching effective aiding noise and magnetic policies; full
NIS coverage and flight-only runtime profiling. No universal win is claimed.
