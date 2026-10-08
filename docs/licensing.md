# Estimator licensing

CogniPilot ESKF, its Lie-group and preintegration implementation, and the
independent comparison orchestration use this repository's
[Apache-2.0 license](../LICENSE). ESKF changes must come from independent
derivations and published mathematics, rather than translating or adapting
ArduPilot implementation code.

## Native validation boundary

PX4 and ArduPilot are pinned validation submodules. Their original licenses
apply to their sources and native executables. Separate Modelica ports retain
their own licenses and are not production ESKF dependencies.

External native replay adapters live in the separately licensed comparison
repository. Build native sources, instrumentation and oracle executables in
owned external directories under `$HOME/scratch`. Keep their license notices
with generated native artifacts.

The ESKF and native comparison programs execute separately and exchange sensor
captures, state/covariance traces and scores. Production ESKF artifacts must
not include ArduPilot implementation headers or link ArduPilot objects.
Do not label a combined native distribution Apache-only.

## Visual algorithms

The `Vision` and `SLAM` packages use CogniPilot's Apache-licensed sources or
independently implemented interfaces. Check provenance and license terms
before adding a visual front end or distributing a combined artifact.

The comparison tools and their generated outputs do not change upstream
licensing. Reading upstream implementations for comparison also does not
establish a clean-room implementation claim.
