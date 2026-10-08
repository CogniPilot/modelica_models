Native sensor-informed noise experiment
======================================

This is a configuration ablation on frozen captures and packet arrivals,
not a new recommended baseline. It tests how closer nominal sensor-noise
settings change the native comparison. No ESKF or upstream algorithm changes.

The native cores remain PX4 v1.17.0 and ArduPilot Copter 4.7.1 at the exact
revisions in source-audit.json. The 48 new replays cover twelve conditions
per native estimator, each repeated with a read-only full-covariance observer.
All 24 pairs produced byte-identical published-state CSVs. Physical capture,
arrival, native executable and observer source hashes are checked against the
published stable-release campaign. Actual PX4 parameters after initialization
and actual ArduPilot logged PARM values are verified against the fixed profile.
Raw CSVs, covariance and ArduPilot DataFlash logs remain in owned scratch.

The profile sets GPS horizontal/velocity standard deviations to 0.2 m and
0.05 m/s, pressure to 0.1 m, and terrain range variance to 0.0004 m2. Flow
and magnetic standard deviations use common native enforced floors of
0.05 rad/s and 1 microtesla; these exceed the physical capture noise used by
ESKF. Heading noise is 0.05 rad. Bias process settings map the configured
continuous PSD to each native prediction period instead of copying numeric
parameter values across different discretizations. Full parameters and
limitations are in frozen-replays.json. These settings were declared before
the runs and were not selected for favorable RMS results.

Results and failures
--------------------
The new settings do not improve every estimator or scenario. PX4's median
GPS-denied horizontal RMS falls from 0.12944 to 0.07246 m, while its vertical
RMS grows from 0.04692 to 0.07765 m. Its GPS median common 15D NEES rises from
47.70 to 1653.03. ArduPilot improves GPS horizontal RMS for seed 7, but has
much larger errors at seed 101. The latter seed's two GPS and two transition
conditions have a non-positive-definite mapped covariance. NEES is undefined
for those conditions; no diagonal jitter, covariance repair, invalid-row
removal or favorable-case subset is used. One 4 m GPS case also loses native
state validity briefly. The settings are not accepted for deployment or as
a better tuned baseline. The cause of these regressions still needs isolation.

comparison.csv retains every raw RMS and validity failure, with all four
declared time windows. comparison.txt includes raw medians and explicit
failures. Invalid state windows are excluded from paired RMS win counts with
an explicit eligible-condition denominator; covariance failures remain visible
even when published-state RMS is finite. A lower NEES is not a ranking of
quality. Repeated native epochs and samples are correlated, and two seeds
coupled to motion scales are not independent Monte Carlo validation.

Unchanged ESKF horizon/retrodiction controls and the disabled pressure-datum
candidate are included from their frozen 48-replay study. Against the original
native configuration, both ESKF controls still win horizontal/velocity/
attitude/yaw flight RMS in all twelve cases and vertical in zero. Against this
new configuration, attitude/yaw wins are fewer. The pressure candidate also
retains its horizontal tradeoff. None of these counts establish universal
superiority or a Lie-group advantage.

Range-noise source discrepancy
------------------------------
ArduPilot documents EK3_RNG_M_NSE as an RMS standard deviation in metres.
Its terrain fusion adds that parameter directly as observation variance,
whereas range height fusion squares it after clamping to 0.1--10.0. Exact
file hashes and line references are in source-audit.json; upstream code is
not copied here. This campaign uses barometric height and disables range
height, so its EK3_RNG_M_NSE=0.0004 mapping targets terrain variance only.
That value lies outside the advertised RMS parameter range. It is an isolated
validation mapping, not a deployment recommendation, and cannot be carried
over to range-height fusion. The experiment does not isolate this setting
from the other changed noise parameters.

Remaining comparison limits
----------------------------
Native source selection, priors, terrain and magnetic states, dynamic noise
additions, state inhibition and gates remain native policies. ESKF follows
physical packet R and has different vertical/magnetic aiding policies. This
is not a fully matched R/Q, native NIS, complete Modelica-port equivalence,
held-out validation or successful RDD2 qualification study.

Reproduction
------------
Use tools/estimator_comparison/compare_native_aiding_noise.py with the frozen
native-release and native-consistency references, frozen case and transport
paths, clean native cores, the audited read-only observer cores/manifests,
the external licensed harness, a C++ compiler and fresh owned --work/--output
paths under $HOME/scratch/modelica_models/native-aiding-noise. Its argument
help names every input. It rejects altered source/binary/input/delivery or
observer parity. It retains unsuccessful covariance diagnostics explicitly.

Run report_native_aiding_noise.py with --study frozen-replays.json,
--reference ../native-releases/native-exposure.json from the 2026-10-07 review,
--observers that review's native-consistency/native-covariance.json,
--eskf ../barometer-datum/frozen-replays.json and an owned --output directory.
The committed report includes complete input/reference hashes and is generated
from those immutable artifacts. The report rejects incomplete or duplicated
conditions; failed state windows remain visible with explicit denominators.

Python needs NumPy and pymavlink for replay, and Matplotlib for the shared
report imports. All 34 estimator-tool unit tests and Ruff checks passed.
