Independent stationary-IMU validation on matched native estimator replays
8 October 2026

All 480 state replays and 96 separate native covariance replays completed.
All sixteen declared capture conditions pass state coverage, native readiness
and common-15 covariance qualification. The 192 stationary ESKF replays use
the frozen option selected from the earlier development campaign. No defaults
are changed. The result supports useful gains, with remaining losses described
below; it does not establish superiority across all scenarios and metrics.

The selected stationary joint-pressure fusion-horizon ESKF has lower velocity
RMS than both native filters in all 48 flight capture/scenario pairs. Horizontal
RMS is lower than both in all 16 GPS and all 16 loss/return conditions; in
GPS-denied flight it is lower in 12/16 conditions against each native filter.
It loses denied horizontal RMS in four conditions to each native stack, with
worst increases of 0.15330 m versus PX4 and 0.30555 m versus ArduPilot. Denied
vertical RMS splits 8/8 against PX4. ArduPilot has lower denied yaw RMS in 6/16.
Every loss remains in comparison.csv and pairs.csv.

Experiment and independence

Four new independent sensor seeds (20271011-20271014), motion frequencies
0.35/1.05 rad/s and heights 1.5/5 m yield sixteen conditions. Conditions sharing
a seed reuse its noise draw: these are four independent noise draws, not sixteen
independent trials. The declaration precedes generation, tool freezing and
replay. Its frozen development-selection, build and parent hashes are checked
by the validation reporter. The fresh captures were not used to select or tune
this stationary option. They become development evidence for any later change.
New joint-terrain and magnetic candidates are not evaluated or selected here.

All estimators receive the same physical IMU, GPS, magnetic, pressure, optical
flow and range noise realizations and arrival schedule. The independent camera
noise and GPS velocity audits pass for all sixteen captures. Effective internal
Q/R, priors, height/terrain models and magnetic policies remain stack-specific.
Native PX4 v1.17.0 and ArduPilot Copter-4.7.1 are actual compiled release code,
with the adapters, immutable source/build pins and observed arrival hashes
bound by each reference pilot and covariance report. This is not a ranking of
partial Modelica ports or an assertion of Modelica/native byte equivalence.

Preflight lasts 120 s with a common declared-rest schedule. First GPS fix is
at 21 s, loss at 132 s, return at 147 s, and flight scores use 120-166.7 s.
The denied case has no GPS aiding. The loss/return case is scored both over
flight and separately over outage and after-return windows. The auxiliary
pressure datum has a joint version in both horizon and retrodiction controls.
Stationary changes only -DSTATIONARY_IMU_MODEL; the public reporter checks
unchanged generated objects and other compiler definitions against the frozen
original builds. The model is a declared-rest process/observation model and
does not replace FOH integration or qualify a flight rest detector.

Median capture-level flight RMS and common-15 mean NEES

scenario    estimator                        horiz m    vert m   vel m/s   att deg   yaw deg   NEES15
gps         horizon_joint                      0.04552   0.03223   0.01565   0.16592   0.14613   9.52093
gps         stationary_horizon                 0.04423   0.03800   0.01657   0.14383   0.12906   9.20180
gps         stationary_retrodiction            0.04686   0.04480   0.01881   0.17073   0.15440   9.99728
gps         stationary_horizon_joint           0.04417   0.03001   0.01443   0.14486   0.12908  10.22802
gps         stationary_retrodiction_joint      0.04678   0.02997   0.01746   0.17331   0.15669  10.61758
gps         px4                                0.19007   0.04775   0.17012   2.97727   2.96146 2666.66131
gps         ekf3                               0.05364   0.05747   0.05049   0.32513   0.30547  44.07538
denied      horizon_joint                      0.36560   0.03944   0.04277   0.24839   0.22666   5.72992
denied      stationary_horizon                 0.32180   0.04169   0.04112   0.27461   0.25222   7.40959
denied      stationary_retrodiction            0.33069   0.04856   0.04126   0.30340   0.27872   7.27649
denied      stationary_horizon_joint           0.36325   0.04206   0.04107   0.27949   0.25015   7.81336
denied      stationary_retrodiction_joint      0.37136   0.04976   0.04111   0.30348   0.27674   7.75948
denied      px4                                0.48420   0.04617   0.05780   0.59214   0.54752  22.85414
denied      ekf3                               0.46444   0.07729   0.12469   0.40374   0.38113  40.57394
transition  horizon_joint                      0.08347   0.03050   0.02027   0.18131   0.16023   7.68114
transition  stationary_horizon                 0.08278   0.03984   0.02103   0.15883   0.13977   8.91720
transition  stationary_retrodiction            0.09703   0.04614   0.02354   0.17208   0.15531   9.94477
transition  stationary_horizon_joint           0.08516   0.02883   0.01981   0.15947   0.14012   8.36098
transition  stationary_retrodiction_joint      0.09890   0.03129   0.02227   0.17307   0.15623   9.40144
transition  px4                                0.28271   0.04746   0.17657   3.09473   3.07966 1458.44025
transition  ekf3                               0.13356   0.06560   0.07388   0.33827   0.31351  43.52756

Flight paired counts with lower RMS for stationary joint horizon. Each count
uses all sixteen valid declared pairs; H/V/velocity/attitude/yaw are distinct
metrics. Percentage changes are paired, not ratios of unpaired medians.

scenario    reference        H    V  vel  att  yaw
gps         horizon_joint     16   16   16   10   10
gps         px4               16   13   16   16   16
gps         ekf3              16   16   16   15   13
denied      horizon_joint      9    6   14    8    8
denied      px4               12    8   16   16   16
denied      ekf3              12   16   16   11   10
transition  horizon_joint      9   13   13   11   10
transition  px4               16   15   16   16   16
transition  ekf3              16   16   16   15   12

Remaining gaps and stationary regressions

Against its original joint-horizon control, stationary joint horizon improves
GPS horizontal, vertical and velocity RMS in all 16 conditions, with paired
median changes -2.68%, -6.15% and -7.57%. GPS yaw improves in 10/16 and worsens
in six. Denied horizontal improves in 9/16 (paired median -5.64%), velocity
in 14/16 (-3.85%), but vertical improves in only 6/16 (median +4.95% worse).
Denied attitude and yaw each split 8/8; yaw median change is +1.72% worse.
Transition horizontal improves in 9/16 (median -0.83%, worst loss 0.01060 m);
vertical and velocity each improve in 13/16. Thus the large denied yaw gain
in the earlier eight development captures did not generalize consistently.

Against PX4, GPS vertical loses in 3/16 and transition vertical loses in 1/16.
Against ArduPilot, GPS attitude loses in 1/16 and yaw in 3/16; denied attitude
loses in 5/16 and yaw in 6/16; transition attitude loses in 1/16 and yaw in 4/16.
The all-metric and all-scenario objective remains unmet.

Covariance, innovations, delays and cost

All 96 joint-pressure candidate replays retain complete raw float32 16-state
covariance checks over 477,120 matrices from 117-166.7 s at the appropriate
state/fusion epoch. Every matrix is symmetric and positive definite with a
positive pressure Schur complement; no jitter or repair is used. Common-15
NEES is reported at each filter's own epoch. NEES below 15 is not automatically
better: this is not a Monte Carlo distribution of initial errors, time samples
are correlated, biases and initial truth are fixed, and only four noise draws
are independent. Native consistency gaps warrant investigation, not a blanket
inference that Lie group filtering is theoretically superior.

reference/native-nis-summary.csv retains native per-sensor/axis scalar NIS,
observed, fused, rejected and invalid counts in every available window. Its
coverage is conditional and incomplete. No new stationary ESKF per-sensor NIS
is measured here, so native-versus-ESKF innovation consistency remains open.
The previous nonstationary sensor audit does not establish candidate NIS parity.
ancillary.json retains all candidate common-15 windows, transport/CPU and GPS
recovery observations. reference/transitions.csv retains original controls.
CPU includes preflight and host effects; native observers and I/O prevent a
fair native CPU ranking, and this is not target WCET.

Instrumented whole-capture ESKF CPU: median / maximum microseconds per IMU tick.
Values sum filter, preintegration, predictor and queue thread time, including
preflight. Runs include host contention, so this is not an isolated cost ablation.

estimator                         median us    maximum us
horizon                               11.671        20.058
horizon_joint                         15.249        25.816
retrodiction                           6.088         9.809
retrodiction_joint                     9.020        14.143
stationary_horizon                    15.393        16.468
stationary_horizon_joint              28.729        33.218
stationary_retrodiction                9.652        11.524
stationary_retrodiction_joint         21.874        22.989

Generated state storage is 173,772 bytes for horizon and 129,992 bytes for
retrodiction; the horizon aiding buffer adds 10,488 bytes. Transport adds
107,552 bytes to each. Storage is unchanged by the stationary build option.

Stationary loss/return GPS timing: every horizon replay first uses returning
GPS at +0.210 s; retrodiction at +0.110 s. For joint variants, median recovery
below 0.25 m for one second is +0.170 s for horizon and +0.110 s for retrodiction;
maxima are +0.320/+0.310 s. This threshold can already be satisfied at return
and is not a latency measurement. Maximum single return-step position-error
jumps reach 0.43679 m for joint horizon and 0.48735 m for joint retrodiction.
Every per-capture value remains in ancillary.json.

Fusion-horizon and retrodiction results are retained as separate variants on
identical captures and packet delivery. Neither is declared universally best:
all three windows and all five RMS components are available in the paired
report. The loss/return figures below are flight-wide; outage and post-return
CSV rows must be used to assess transient behavior separately.

Figures and provenance

figures/{horizontal,vertical,velocity,attitude,yaw} contains PNG/PDF/SVG paired
plots for ordinary and joint-pressure stationary horizons against each native
stack. Every capture is plotted, including points above the equal-error line.
comparison.csv has 1,440 rows covering ten estimators, three scenarios and
all three score windows. summary.json binds every candidate/native manifest
and the post-replay reporter sources; reference/summary.json binds original
control covariance, readiness and noise/configuration evidence. cases/ retains
all sixteen candidate score files. Raw source, captures, binaries, native
observer logs and full covariance traces remain under the owned directory
$HOME/scratch/modelica_models/stationary-independent. Files in this review
are durable evidence; any absolute paths in build records describe the measured
machine, not portable project settings. evidence-sha256.json binds this bundle.

Reproduce reference reporting using tools/estimator_comparison/report_readiness_campaign.py
with --declaration, --captures, --campaign, --snapshot and --root from that
scratch directory, and a fresh --output. Reproduce candidate reporting using
report_eskf_ablation.py --candidate "$HOME/scratch/modelica_models/stationary-independent/stationary"
--references "$HOME/scratch/modelica_models/stationary-independent"
--reference-summary FRESH_REFERENCE_REPORT/summary.json
--reference-declaration "$HOME/scratch/modelica_models/stationary-independent/declaration.json"
--baseline-build docs/reviews/2026-10-08/common-sensor-noise/replay-build.json
--stage validation --development-summary docs/reviews/2026-10-08/stationary-readiness/summary.json
--output FRESH_CANDIDATE_REPORT. Plot that comparison with
plot_readiness_campaign.py --prefix stationary_ --metric METRIC --output FRESH_FIGURE_DIRECTORY.
