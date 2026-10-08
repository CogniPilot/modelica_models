Declared startup rest pressure calibration
=========================================

This is an optional experiment, disabled by default. The prior calibration
stops after 25 distinct pressure samples. With
useDeclaredRestBarometerCalibration=true, calibration continues throughout
declared startup rest at the configured initial altitude. Learning packets
are withheld from navigation; the first release of vehicleAtRest closes
calibration until reset. Later landing cannot redefine the origin. Early
takeoff uses the learned datum or configured prior and enables pressure
fusion instead of waiting indefinitely for stationary samples.

The experiment changes neither sensor noise settings nor minimum sample
count. It adds finite altitude/variance guards and retains datum process
noise when there was no initial rest. The generated-C boundary test covers
continued learning beyond the minimum, duplicate timestamps, withholding,
release, later rest, reset, early takeoff, absent rest and nonfinite pressure.
tools/ci.py runs this test against the actual RDD2 estimator export.

Evidence and interpretation
---------------------------
frozen-replays.json contains 48 fresh replays: two timing methods, two policies
and twelve frozen physical/delivery conditions. Every one of the 24 default
controls reproduced the previously published output byte for byte. The same
captures, arrivals, IMU noise density, initial conditions and aiding settings
were used for each candidate/control pair. Native scores and covariance come
from the stable-release campaign and observer parity study; they were not
rerun for this experiment. Input, arrival, executable and reference hashes
are checked by compare_barometer_datum.py.

Vertical flight RMS fell 36.3--50.8% versus the corresponding ESKF control.
Horizon candidate vertical RMS beats both native cores in 10/12 conditions;
retrodiction does so in 11/12. However, horizontal RMS worsens in some denied
cases. For the horizon, seed 7 at 2 m rises from 0.04550 to 0.07744 m, and at
4 m from 0.04215 to 0.08941 m. The horizon control beats both native cores
for flight horizontal RMS in 12/12 cases, whereas the candidate does in
10/12. The analogous retrodiction count falls from 12/12 to 11/12. This
tradeoff is why the feature stays disabled. paired-flight.csv exposes every
candidate/control metric, including regressions.

comparison.csv includes flight, outage-time and after-return-time RMS and
common-coordinate 15D NEES, plus preflight NEES. The named outage/return
windows also apply to continuously aided/denied controls; their names do not
imply those controls change GPS mode. comparison.txt and summary.json record
all paired win counts. comparison.svg/png show individual condition values
and medians, with the nominal 15D NEES expectation as a descriptive reference.
The figure was visually inspected.

This is a matched-data policy ablation, not a matched-effective-noise study.
Native effective R/Q, priors and height/magnetic policies remain unequal.
There are only two seeds, coupled to two motion scales, and two heights.
Samples and repeated native fusion epochs are correlated; these are not
independent Monte Carlo NEES tests. Lower NEES alone is not a quality ranking.
Joint pressure/navigation cross covariance and per-sensor native NIS remain
unresolved. The known initial-altitude/rest assumption is essential. There
is no claim of universal superiority, a Lie-group advantage from this policy,
full native-port equivalence, held-out validation or deployment qualification.

Reproduction
------------
Use Rumoca 0.10.2, a C compiler and Python with NumPy/Matplotlib. Large builds,
case copies and retained covariance traces belong under
$HOME/scratch/modelica_models/barometer-datum. Start with the physical captures,
frozen transport executable and completed stable-release cases documented in
docs/reviews/2026-10-07/native-releases.

Build the actual Modelica blocks with tools/estimator_comparison/build.py:
--eskf-only --horizon --geometric-alignment --equivariant-magnetometer.
Repeat with --declared-rest-barometer in a separate output directory for the
candidate, or relink replay.c with -DDECLARED_REST_BAROMETER against the same
generated objects. Both methods use the same generated estimator mathematics.
The current evidence uses shared objects and separate bridge compilations.

Run compare_barometer_datum.py with --reference pointing to
docs/reviews/2026-10-07/native-releases/native-exposure.json, --cases to the
completed stable-release case directory, --transport-trace to the frozen
transport executable, four --{horizon,retrodiction}-{control,candidate}
binary arguments, a fresh --work directory and fresh --output JSON path.
It refuses changed captures/delivery or default-control outputs, and keeps
all raw covariance in its owned work directory.

Run report_barometer_datum.py with --study that JSON, --native the stable
reference, --native-covariance
docs/reviews/2026-10-07/native-consistency/native-covariance.json and --output
an owned report directory. The report refuses incomplete campaigns, invalid
NEES, missing conditions or failed native observer output parity.

Rejected build witness
----------------------
An initial calibration guard used an if-expression on the new Boolean
parameter. Rumoca 0.10.2 folded that expression to the default policy while
leaving the parameter public in generated C. Setting the parameter from the
bridge did not change behavior, and the boundary test failed on the third
learning packet. No replay score from that build is included. The final
explicit Boolean guard preserves the parameter read and passes the actual
generated-C test. The rejected and accepted builds remain in owned scratch
build-v2 and build-v3 respectively.
