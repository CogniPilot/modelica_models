Native noise-group isolation
===========================

This diagnostic revisits the frozen 0.85 motion-scale, seed 101, 2 m GPS
capture where the full sensor-informed native configuration failed. Before
running, it declares fourteen variants: original aiding settings, full new
settings, each of six noise groups alone, and the full configuration with
each group removed. It changes no physical samples, packet arrivals, native
source revision or executable core. It is not a tuning search or a validated
comparison baseline. One capture cannot establish performance across GPS,
GPS-denied, transitions or new missions.

frozen-diagnostics.json contains all 56 new native replays: fourteen variants
for two estimators, each repeated with the read-only covariance observer.
All 28 observer pairs have byte-identical published states. Fresh original
and full controls for both estimators reproduce the four published state CSVs
byte for byte. Each subcampaign checks frozen input/arrival/core hashes and
actual native parameter values. Raw captures, state/covariance CSVs, logs
and individual subcampaign manifests remain in owned scratch storage.

The six parameter groups are GPS, barometer, range, magnetometer/heading,
flow, and gyro/accelerometer bias process noise. Exact membership and values
are in tools/estimator_comparison/native_aiding_noise.py. None means all groups
in the diagnostic declaration; an empty list means original aiding settings.
The latter retains the previously audited white-IMU-density mapping.

Results
-------
All six one-group variants keep the common covariance positive definite on
this capture. The full ArduPilot configuration fails, as do full settings
with range, flow or bias tuning removed. Removing GPS, barometer or magnetic
tuning from the full configuration instead keeps the mapped covariance
positive definite. This indicates an interaction; it does not identify one
faulty parameter or prove a native implementation defect.

GPS tuning alone raises ArduPilot horizontal RMS from 0.06710 to 0.28761 m;
flow tuning alone raises it to 0.54281 m. The magnetometer-only variant improves
attitude RMS from 0.38273 to 0.11959 degrees and yaw from 0.36115 to 0.08642
degrees. Its vertical RMS also improves on this capture. This is evidence
against treating earlier ESKF win counts as proof of algorithmic superiority;
it does not justify selecting this post hoc variant as a new deployment or
held-out baseline. Every variant, including the four failed covariance cases,
appears in comparison.txt/csv. Failed NEES is explicitly undefined.

Covariance witness
------------------
covariance-first-failure.json separately scans all 4970 observer rows in the
declared 10--59.7 s fusion-time window of the original full-profile failed
ArduPilot run. It retains repeated epochs and diagnoses 4272 Cholesky failures.
The first is at fusion time 16.9 s, published at 17.1 s. The native symmetric
24D matrix already has a negative eigenvalue, -2.304e-8; the common 15D marginal
has -2.877e-6. Its diagonal remains positive. Standardizing that marginal by
its own diagonal gives a dimensionless minimum eigenvalue of -0.001926, with
the negative mode concentrated in horizontal position and accelerometer bias.
This is not explained by the coordinate transform creating indefiniteness
from a positive-semidefinite native matrix. No covariance jitter, repair,
row removal or replacement NEES is applied. Mixed-unit raw eigenvalues are
diagnostic, not cross-estimator quality scores. The causal source operation
still needs isolation before claiming an upstream defect.

Reproduction
------------
diagnose_native_noise_groups.py accepts the same native source/core/observer,
external harness, frozen transport and reference arguments as
compare_native_aiding_noise.py. Add --full-reference pointing to the completed
native-aiding-noise/frozen-replays.json and --condition
0.85_101_2m_gps_explicit. The tool fixes the fourteen variants rather than
accepting a favorable subset; --estimator can isolate one native core.
Choose fresh --work and --output paths under
$HOME/scratch/modelica_models/native-aiding-noise. The existing full-campaign
runner retains its original default parameter profile when no group selection
is requested.

Use report_native_noise_groups.py --study frozen-diagnostics.json --output
an owned report directory. It rejects missing/duplicated variants and failed
observer or frozen-control parity. Use diagnose_native_covariance.py ekf3
with --covariance the retained full-profile raw observer CSV and a fresh
--output to reproduce the first-failure witness. No upstream source is copied
into this repository. These native experiments remain distinct from complete
Modelica ports and closed-loop RDD2 mission qualification.

validation.json records byte-identical report/witness reproduction and rejection
of missing variants, duplicate variants and broken frozen/observer parity. The
original control covariance has zero Cholesky failures in the same window.
The scanner validates complete finite raw columns, validates timestamps within
the declared scoring window, and separately counts unscored startup rows whose
native fusion epoch is ahead of publication. All 34 estimator-tool unit tests
and Ruff checks passed.
