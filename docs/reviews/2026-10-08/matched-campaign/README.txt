Matched native estimator comparison: eight independently declared captures
=========================================================================

The complete campaign has 144 state replays and 48 native covariance replays.
All eight captures finish, all native readiness checks pass, and every native
covariance observer reproduces its separate innovation-observer state bytes.
All 432 flight/outage/after-return scoring rows have valid state coverage and
positive-definite common-state covariance. This establishes a usable comparison
in the declared domain; it does not establish universal ESKF superiority.

The declaration, independently audited capture hashes and frozen tool snapshot
are in ../ekf3-imu-integrity/. campaign.json binds the final eight results;
cases/ retains every full pilot and covariance manifest. summary.json binds the
source manifests and all result files. comparison.txt is the complete readable
table; comparison.csv retains every condition and scoring window, including
position, velocity, attitude, yaw, NEES, coverage and output/arrival hashes.
horizontal-pairs.png/pdf/svg show every horizontal RMS pair for the two horizon
variants. Points above the diagonal are ESKF losses and remain visible.

Scope and common data
---------------------
The factorial was declared before these captures were generated: independent
noise seeds 20271009 and 20271010; motion frequencies 0.45 and 0.85 rad/s; climbs
2 and 4 m. Noise seed and motion frequency are crossed, not coupled. Captures
with the same seed reuse their noise draws across motion/height conditions;
there are two independent noise realizations, not eight independent trials.

Every core receives the same physical IMU/GPS/flow/range/mag/barometer samples
and the same delivered packet trace in each condition. All use a 120 s noisy
stationary preflight, GPS unavailable until 21 s, takeoff at 120 s, GPS loss at
132 s and return at 147 s. Flight scoring is 120-166.7 s at 100 Hz. The common
physical sensor-noise profile and 100 ms camera exposure remain unchanged.
Independent world-velocity/quaternion camera and GPS-noise oracles check every
capture. The reporter also verifies the physical climb from frozen truth.

The initial GPS delay follows the previously diagnosed EKF3 startup trigger;
it applies to every estimator. It avoids that known synthetic-startup failure
without repairing or tuning native source. The failed immediate-GPS case and
its full causal trace remain in the preceding review. This campaign alone does
not qualify immediate GPS startup, vibration, magnetic faults or arbitrary
terrain. Native cores remain PX4 v1.17.0 and Copter-4.7.1 at their recorded pins.

ESKF uses Rumoca 0.10.2 generated production C, FOH with bias Jacobians,
geometric alignment and calibrated vector magnetic fusion. Four variants cross
fusion horizon/retrodiction with joint/nonjoint pressure bias. The 200 ms horizon
predicts forward to publication time. Joint pressure fusion remains optional.
Vehicle defaults and Modelica algorithms are unchanged by this review.

Measured strengths and remaining losses
--------------------------------------
ESKF horizon has lower horizontal RMS than both native filters in every GPS
and loss/return capture. Every ESKF variant has lower 3D velocity RMS than both
native cores in all 24 scenario/capture conditions. Neither statement implies
better RMS on every component or on arbitrary flights.

Median flight RMS across capture-level RMS values:
scenario      estimator              horizontal m   vertical m   yaw deg
GPS           ESKF horizon               0.04445       0.03259      0.09320
GPS           ESKF joint horizon         0.04454       0.02747      0.09354
GPS           PX4 EKF2                   0.17219       0.04755      2.69741
GPS           ArduPilot EKF3             0.05460       0.05644      0.18578
GPS denied    ESKF horizon               0.37485       0.03831      0.33338
GPS denied    ESKF joint horizon         0.35891       0.03264      0.33640
GPS denied    PX4 EKF2                   0.40394       0.03157      0.99127
GPS denied    ArduPilot EKF3             0.41904       0.07278      0.15613
Loss/return   ESKF horizon               0.11887       0.03287      0.10760
Loss/return   ESKF joint horizon         0.11826       0.02991      0.10777
Loss/return   PX4 EKF2                   0.27465       0.04956      2.77369
Loss/return   ArduPilot EKF3             0.15687       0.06141      0.21138

In GPS-denied flight, ESKF horizon beats PX4 horizontal RMS in 5/8 and EKF3
in 4/8 captures. Joint horizon improves those counts to 7/8 and 5/8. EKF3 has
lower denied-flight attitude and yaw RMS in 6/8 captures. PX4 has lower vertical
RMS than either horizon variant in 5/8 denied captures and 3/8 GPS captures.
These are remaining gaps against the requested objective.

Joint pressure fusion improves vertical RMS versus its corresponding nonjoint
variant in all 24 scenario/capture conditions for both horizon and retrodiction.
It improves denied horizontal RMS in all eight captures for both methods, but
does not consistently improve attitude/yaw. A pressure improvement is not a
proof of a universally better navigation estimate or a reason to hide tradeoffs.

A separate 48-replay audit also checks the complete augmented 16D pressure/
navigation covariance at every fusion epoch from 117 to 166.7 s. All 238,560
matrices are exactly symmetric and positive definite after recovering the
actual float32 values from the round-trip-safe CSV. Every pressure Schur
complement is positive, with no symmetrization, jitter or repair. All 48 state
outputs are byte-identical to the frozen pilot. joint-covariance.json retains
every capture/scenario/variant, raw covariance hash and minimum Schur variance.
check_joint_replay_covariance.py reproduces the numerical check on an original
raw diagnostic CSV; pass --horizon for fusion-horizon state epochs.

Timing and consistency
-----------------------
The horizon/retrodiction comparison has no universal accuracy winner. Horizon
has lower nonjoint denied horizontal and vertical RMS in all eight pairs;
retrodiction has lower denied velocity, attitude and yaw RMS in all eight.
Retrodiction has lower transition horizontal RMS in 6/8 pairs. Full paired
metric counts and median/worst differences are recorded in summary.json.

Median whole-capture instrumented thread CPU time per 800 Hz IMU tick:
  horizon                    11.812 microseconds
  retrodiction                6.191 microseconds
  joint horizon              15.495 microseconds
  joint retrodiction          9.162 microseconds
eskf-timing.csv retains every case, state/buffer/transport size and CPU result.
These are replay measurements with instrumentation, including preflight and
output prediction. They are not worst-case flight execution bounds. Native CPU
is not compared because its innovation logging and CSV I/O change the work.

Median capture-level flight mean common 15D NEES, GPS / denied / loss-return:
  ESKF horizon                6.96 /  6.02 /   6.57
  ESKF joint horizon          7.08 /  6.17 /   6.67
  PX4                      1421.65 / 26.40 / 886.21
  EKF3                       42.94 / 41.39 /  42.57
NEES is evaluated at each filter's own fusion/state epoch. These correlated
samples and two noise realizations do not support independent Monte Carlo
confidence claims. NEES below 15 is not automatically better or proof that the
uncertainty model is calibrated. Bias truths and initial errors are not drawn
from each core's own prior. Stack-specific effective Q/R and priors remain.

native-nis.csv preserves every observed scalar record grouped by sensor, axis,
stage, navigation mode and window. native-nis-summary.csv gives capture-level
mean NIS medians/ranges without treating time samples as independent trials.
Many native scalar NIS means are near one despite much higher full-state NEES.
EKF3 accepted flight flow means have capture medians about 1.00/0.96 in denied
flight; its accepted GPS velocity means are about 0.95/0.93/0.96. PX4 accepted
GPS horizontal velocity means have medians about 1.28/1.32. Native observations
are selection-conditioned, coverage is incomplete, and scalar sequential NIS
is not vector NIS. ESKF per-sensor NIS is missing from this campaign; that is an
explicit follow-up requirement, not replaced by native statistics.

Native magnetic-policy investigation
-------------------------------------
Three separate known-seed-911 native PX4 replays observe only public magnetic
state getters after update. Every published state AND innovation CSV is byte
identical to the frozen original. No native source or parameters are changed.
px4-magnetic-observer.json records adapter/core/header hashes and all three
parity checks; px4-*-magnetic.csv retain every raw observation. The diagnostic
summary is px4-magnetic-analysis.json. This is a separate diagnostic, not a new
independent comparison or isolated explanation of all PX4 error.

The configured declination remains -4.53642 deg; WMM declination is -4.41266 deg.
Those differ by only 0.124 deg, not the approximately 2.64 deg GPS yaw RMS in
that capture. PX4 uses 3D magnetic fusion for 97.0% of the flight in all three
scenarios. Its estimated field direction reaches -6.77 deg in GPS flight and
-6.91 deg in loss/return, versus -4.45 deg denied. The native declination
constraint is active in 0.17%, 19.87% and 100% of those respective flight rows.
Earth-field direction and body magnetic biases are estimated states; ESKF's
fixed calibrated vector is a stronger model assumption. This policy difference
must remain visible in any algorithm ranking. Observed drift is not causal
proof that one parameter or source line explains the heading error.

The pinned native getMagDeclination() uses the saved manual value before
alignment and the learned earth-field direction after in-flight alignment.
resetMagStates() can initialize from WMM independently of that manual-value
selection. These are separate branches, not evidence that the manual parameter
was silently overwritten. Relevant pinned primary source:
https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/ekf2/EKF/aid_sources/magnetometer/mag_control.cpp

Reproduction and next work
---------------------------
report_readiness_campaign.py takes --declaration, --captures, --campaign,
--snapshot, --root and --output. --root is the owned scratch campaign containing
the eight captures and replay manifests. It rejects changed hashes, missing or
duplicated declared conditions, different deliveries, altered observer outputs,
different tools/configuration between captures and wrong physical climb height.
Failed captures remain in the paired denominator and prevent completion claims.
replay_px4_magnetics.py builds an adapter against the frozen native library and
requires the original capture, arrival, state and innovation hashes. It uses
public const getters only. plot_readiness_campaign.py makes the three exported
scientific figure formats from comparison.csv. Keep builds/data under $HOME/scratch.

The completed vector versus yaw-only development ablation is in
../magnetic-policy/. Its 96 new replays retain all four ESKF variants and all
three scenarios, with four exact rebuilt vector binary controls. Heading-only
worsens every GPS and denied velocity pair and is rejected as a general
replacement. These inspected captures are not untouched validation. Further improvements
need a fresh validation set, complete ESKF innovation diagnostics, broader
noise/outage/terrain/fault coverage, and explicit magnetic-policy controls.
The existing GNC conditional Lie/covariance proofs do not prove superiority
over these complete nonlinear firmware estimators.

Validation: 65 comparison tests pass in the hosted Python environment, with
no skips. Negative evidence tests reject changed native state hashes and
arrival traces; invalid covariances cannot become wins, and failed captures
remain in denominators. Actual magnetic-observer parity checks cover all three
scenarios. RDD2 mission qualification remains unresolved separately.
