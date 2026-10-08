ESKF covariance-root CI follow-up
================================

GitHub run 37725402683 at 0b7d311 reproduced the Rumoca 0.10.2 EL005 failure
in Tests.StrapdownEstimatorInterfaceTests. The ea5c475 baseline passes that
same simulation to 0.02 s. The failure was narrowed to concatenating a
zero-row padding array inside a pure function. MaxWorkspace.mo passes with
the same max-dependent workspace declaration; EmptyPadding.mo fails. The
small reproducer/control are in tools/rumoca-repros/empty-function-padding.

LinearAlgebra.covarianceRoot now works directly on transpose(columns), and
returns zero columns where a narrow input lacks corresponding work rows.
It removes padding and replaces the two trailing-element loops with Givens
row expressions. The explicit upper-row snapshot remains necessary to avoid
the previously identified OpenModelica slice-alias issue. No diagonal noise
or covariance repair is introduced. The existing wide/narrow/zero Modelica
assertions pass under Rumoca and OpenModelica. The CI list now simulates that
assertion model directly before the estimator fixtures.

The previous combined interface fixture also produces excessive transient
Rumoca solver-construction memory after the padding error is removed. Its
three independent scenarios now run separately: the common ESKF/UKF interface,
the held GPS seed/covariance assertion, and invalid-then-valid seed timestamps.
Every original seed assertion and its 0.02 s observation window is retained.
Both fresh-cache GPS simulations pass, with 22,636,288 KiB cumulative maximum
child RSS (21.59 GiB). The interface-only simulation also passes. Failed or
interrupted combined attempts are not counted as passing tests.

The regression job allocates 8 GiB swap before testing. GitHub documents 16 GB
RAM for standard public ubuntu-latest runners:
https://docs.github.com/en/actions/reference/runners/github-hosted-runners
This bounds each independent fixture's transient demand without removing a
gate or changing filter settings. Hosted-run behavior still needs confirmation.
No swap configuration on the developer's computer was changed.

Generated-C checks pass for both correction geometries, constrained gains,
correlated observations and invalid joint covariances, square-root corrections,
raw dense/root prediction (640 cases), QR roots (412 cases in the CI seed pair),
120-second horizon epoch handling with reset, simultaneous aiding/GPS loss and
return, and UKF raw/preintegrated hover. Extra QR runs use seeds 20271007 and
911 (412 cases). Python's 32 estimator tests, Ruff and package structure checks
also pass. The all-other-checks log explicitly delegates the interface run to
a separate process; it is not a claim that one complete CI invocation passed.

The final all-other-checks gate initially rejected the already documented
HorizonEstimator.mo:259:5 cross-clock error because its diagnostic variable
identity changed from 578 to 583. The compiler pin, ED020 class, clock ownership
cause, assignment and source location are unchanged. The expected identity is
updated after direct reproduction. Other diagnostics still fail this gate.
This does not fix the composed fusion-horizon compiler limitation.

An attempted OpenModelica simulation of the interface fixture returned an empty
resultFile without a diagnostic. It is retained as unsuccessful diagnostic
evidence, not substituted for the passing Rumoca fixture or counted as a pass.
The regular OpenModelica Tests.All suite is unchanged by the fixture split.

validation.json records source/evidence hashes and statuses. Raw large builds
and HTML outputs remain in $HOME/scratch/modelica_models/rumoca-ci/contract-probes.
GitHub CI and both complete mission qualifications must still pass before
calling main qualified. This follow-up does not change ESKF sensor tuning or
claim improved RMS, matched native R/Q, full NIS, or theoretical superiority.
