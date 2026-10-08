Joint-terrain prototype initialization failure
8 October 2026

This concerns the optional, unmerged joint-terrain candidate in an owned
scratch worktree. It is not an enabled-main estimator or a native-filter
superiority result. All diagnosis uses the already-known seed-911 development
capture, not independent validation data.

The v12 denied/horizon candidate accepted flow at 6.10 Hz from a 10 Hz stream.
A read-only actual gain observer retained exactly the original state and full
covariance CSV bytes. In flight, 182/468 two-component gain calls rejected
strict symmetry with raw R asymmetry up to 2.84217094e-14, while PSD checks and
factorization succeeded. Three-component calls stayed symmetric. Gain calls
include other sensor updates; their dimension alone does not identify a
measurement or imply a count of independent sensor observations.

A second read-only replay observer retained exact state/full-covariance byte
parity and showed useJointTerrain true on all 16701 output rows, but the
terrain initialization flag false on every row. The candidate thus repeatedly
takes its bootstrap branch instead of maintaining the initialized terrain
state. Correct range exposure midpoint timestamps were checked separately;
there is no evidence for the suspected range timestamp mismatch.

Generated Rumoca 0.10.2 C for Estimator.mo assigns the held initialization
flag in a branch that shadows the subsequent step-result assignment. The
minimal portable Modelica programs and C probes reproduce the behavior for
four startup useJoint/reset combinations and four steps each. The sequential
source algorithm requires true for every sample. The original generated code
fails 8/16 samples; its single-assignment equivalent passes 16/16. Sources are
stored with .txt suffixes to keep review fixtures outside library compilation.
No Rumoca checkout was changed.

The v13 source candidate gives terrainInitialized one assignment after the
step. All 24 disabled-terrain state and covariance cases pass byte parity.
However, an enabled denied/horizon observation still shows initialization
false on all rows and exactly the v12 state/covariance hashes. All 24 enabled
state and full-covariance CSV files exactly match v12, and the read-only
observer has exact parity with its corresponding v13 replay.
The single-assignment workaround alone is insufficient; no success is claimed.

A second minimal reproduction identifies another issue: an if-expression
using useJoint=false, declared in the eFMI manifest as tunableParameter, is
folded to its default. Tuning the field true before the first step fails 8/16
samples. The same expression with an input Boolean passes 16/16, as does an
equivalent if-statement with the tunable parameter. The V14 candidate therefore
uses explicit mutually exclusive if-statements for initialized-state output
and the prior terrain mean/variance. It also gives the correction-status output
mutually exclusive assignments. Gains, noise, gates and defaults remain the
same. Both declarations precede their full Modelica exports.

V14 full export and all 24 disabled-candidate state/full-covariance byte parity
cases passed. The enabled denied/horizon observer now shows initialization
true on 16645/16701 rows, first at 0.560000002 s. Both its state and full
covariance CSV files exactly match the ordinary v14 replay. The gain observer
also retains exact state/full-covariance byte parity.

The enabled campaign stopped on its first required covariance failure: all
eight GPS cases passed the raw full17 check, but denied/horizon failed. Its
full joint covariance first has a nonpositive eigenvalue at fusion time
123.208748 s; 783 matrices fail in the scored window, with minimum eigenvalue
-0.0005302003054106185. The navigation submatrix also becomes indefinite.
No jitter, symmetrization or covariance repair was applied in the audit.
The remaining 15 enabled cases were not run. Recorded accuracy for the failing
case is diagnostic only; initialization success does not qualify this filter.

In that same v14 denied/horizon flight, 347/936 three-component gain calls have
asymmetric R, up to 2.27373675e-13. Other calls reject invalid prior covariance.
Sensor types cannot be inferred from measurement dimension alone. An actual
Rumoca-generated rank-one primitive separately reproduces asymmetric entries
in 2817/4096 seeded float32 cases for q * transpose({h}) * {h}, versus zero for
q * (transpose({h}) * {h}). This supports investigating expression grouping;
it is not an applied full-estimator fix or a covariance-validity result.

R symmetry and covariance regressions must be diagnosed independently. No
adoption claim follows from these reproductions or the conditional Lean bounds.

Subsequent v15 development qualification
--------------------------------------
The read-only v14 stage observer retains exact state/full-covariance byte
parity. Its first chronological indefinite output is the scalar terrain
correction at 123.208748 s, after usable prediction/limiter outputs; the later
three-component rejection carries that invalid prior forward. Stage listings
are grouped by specialization, so listing order is not execution order.

V15 was declared before export. It changes only the optional derived terrain
noise expression from q * transpose({h}) * {h} to q * (transpose({h}) * {h}).
Noise settings, gates, defaults and external input covariance are unchanged.
Full Rumoca 0.10.2 export passed, as did all 24 disabled-terrain state and
full-covariance byte parity controls. All 24 enabled development cases now
pass the raw full17 covariance and pressure/terrain Schur checks in the scored
117.0--166.7 s window, without audit-side jitter or covariance repair.

The v15 denied/horizon gain observer again preserves state/full-covariance
bytes. All 936 flight three-component calls have symmetric derived R and valid
prior/noise/factor checks; all are accepted. Flow acceptance returns to 10 Hz.
Horizontal RMS is 0.145979 m in that development case, versus 0.181459 m for
the original horizon baseline. Stationary retrodiction still has regressions;
the known seed-911 pilot is not independent validation. No candidate defaults
or full joint-terrain implementation have been adopted on main. Startup before
the scored window is not certified by the full17 campaign checks.

The declaration, export/source identities, parity receipt, compact per-case
campaign results and exact-parity observers are retained alongside the earlier
failure evidence. The complete candidate and raw replays remain in owned
scratch storage; these receipts alone do not reconstruct that unmerged source.

The shared v12 Jacobian precision change produces exact state/full-covariance
CSV equality with v11 on all 24 enabled development cases. This establishes
that the shared fix did not resolve the terrain candidate's existing losses.
The side-by-side parity receipt retains all cases.

Portable reproduction:
  python docs/reviews/2026-10-08/terrain-persistence/reproduce.py \
    --rumoca /path/to/rumoca-0.10.2 --cc /path/to/gcc \
    --work "$HOME/scratch/modelica_models/terrain-persistence-reproduction"

Use a fresh work directory. All five source algorithms require true for every
sample. The runner reports failures as compiler semantic failures; its separate
known_failure_reproduction flag identifies the observed 0.10.2 defects.
