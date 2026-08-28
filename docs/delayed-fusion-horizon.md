| `foldBuffer.mo` | function | the re-base kernel: a fixed-length branch-free walk of the ring plus one trailing row |# Delayed fusion horizon with an estimator-agnostic SE_2(3) output predictor

Status: design of record for `Estimation.FusionHorizon`.

Companion theory: J. Goppert et al., *Closed-Form First-Order-Hold
Preintegration on SE_2(3)* (the FOH paper). Numbered results cited below are
from that paper and are quoted by label in the source comments of every
function this document specifies.

## 1. The problem this replaces

The estimator fuses aiding measurements that are old when they arrive. GPS on
the deployed stack is roughly 100 ms behind the inertial stream by the time the
driver hands it over. Two families of answer exist.

1. **Correct at now, transport the Jacobian back.** Build `H = H_d Phi(-age)`
   and correct the current state with a measurement Jacobian walked backwards
   through the transition. This is what the corpus does today:
   `ESKF/retrodict.mo` walks the nominal state back by the exact inverse ZOH
   mixed flow over a single held IMU sample, and `ESKF/step.mo` applies the aged
   Jacobian.

2. **Correct at the horizon, reapply the buffered increments.** Run the filter
   at `t - D`, where every measurement has already arrived, and carry the state
   forward to now by composing the preintegrated increments buffered while
   waiting.

The FOH paper calls (1) the *transported-Jacobian form* and (2) the
*buffered-window form*, and states in Sec. XI-B that its delayed-fusion and
reapplication theorems are written for (2) while the implementation realizes
(1). This document specifies (2).

### The abstract is currently ahead of the implementation

Worth stating plainly, because it is the strongest single reason to do this
work: **there is no FIFO and no buffered horizon in the corpus today.** The
paper's abstract asserts a buffered 200 ms fusion horizon. What exists is a
destructive accumulator that is reset at every 100 Hz packet boundary, one
100 Hz loop predicting and correcting at the live edge, and measurement-age
alignment by `retrodict`. The paper's own Sec. XI-B retracts the claim; the
abstract does not.

`Estimation.FusionHorizon` is what makes the abstract true. Once it lands, the
paper needs either its abstract revised or its implementation reference
updated to point at this package. **That is flagged, not done here**: the paper
lives in its own repository with its own remote and is not this change's to
edit.

### What is replaced, and what remains

| concern | today | under this design |
| --- | --- | --- |
| measurement-age alignment of the nominal state | `ESKF/retrodict.mo`, one held IMU sample over the age | not needed for anything inside the horizon: the measurement is fused at its own timestamp, which IS the fusion instant |
| aged measurement Jacobian `H_d Phi(-age)` | built in `ESKF/step.mo` | `Phi(-age)` becomes identity for aiding inside the horizon; only `H_d` remains, and `H_d` is the half already mechanically verified (`lie-jacobian-eskf-proof`, commit 17767d3) |
| `maximumAidingDelay_s = 0.25` | admits packets whose cubic-Taylor transport is 12 to 27 percent wrong, against a deployed GPS age nearer 0.1 s | the transport is not used for aiding that reaches the horizon. What remains is the residual window for a packet that arrives AFTER its epoch has passed the horizon; that window is `age - fusionHorizon_s`, and the horizon length is chosen so it is normally empty. A packet later than the horizon is a supervision event, not a transport problem, and should be rejected on its timestamp |
| process noise over the aging window | the gain and the Joseph posterior neglect the process noise accumulated between the measurement timestamp and the fusion time | the class is removed by construction: at the horizon there is no gap between measurement epoch and fusion epoch, so there is no window to accumulate over |
| `correctMocap` has no retrodiction stanza at all, though the paper claims every `correct*` has one | a real asymmetry: mocap is aged like everything else and is aligned like nothing else | every source routes through one horizon path. The asymmetry closes by construction rather than by adding a fifth copy of the same stanza |
| carrying the state to now for control | nothing. Control reads the filter output, one estimator period stale, and there is no output predictor | the output predictor of Sec. 4 |
| the 29 `delay()` operators in `Vehicles/Rdd2/WaypointVehicleSystem.mo` | model the physical transport latency of the simulated sensors | **unchanged.** They are plant-side truth, not estimator machinery. A horizon does not remove sensor latency; it is where the horizon length comes from |
| bias relinearization of buffered increments | `correctPreintegratedImu.mo`, first order in `db` | unchanged in form, now bounded by Prop. 8 over a window whose length is a parameter rather than over whatever age a packet happened to have |

Retrodiction is superseded for nominal-state alignment and for the delay
transport factor of `H`. It is not superseded for anything else. The function
and its callers stay in the tree until the ESKF rewiring of Sec. 8 lands.

Two things about the current path are **verified correct and are not being
replaced because they are wrong**: `retrodict` is numerically exact as an
inverse to 1e-16, and its transport error is purely the cubic Taylor
truncation. The argument for the horizon is that it does not need either.

## 2. What makes the buffer exact

Everything rests on one structural fact. Under the mixed-invariant flow
`Xdot = M X + X N(t)` with `M` constant, the flow over `[0,T]` factors as

    X(T) = L X(0) R,     L = exp(M T),   R = exp(Xi(T))

with `L` and `R` **independent of `X(0)`** (FOH paper Lemma 3 and Theorem 6,
Sec. VI). Three consequences, and they are the whole design.

1. **A horizon correction reapplies the same factors.** If the filter shifts the
   horizon state, `X'(T) = L X0' R` with the *same* precomputed `R`. No
   re-integration. This is why the predictor is recomposed rather than tracked
   by a gain.
2. **Increments compose.** `L` and `R` over adjacent intervals multiply, so a
   buffer of per-tick right factors accumulates to the right factor of the whole
   window, exactly (Lemma 5, Sec. IV-C).
3. **The FOH corrections live inside `R`.** The coning, sculling, and scrolling
   terms of Theorem 1 (Sec. III-D) are components of the truncated Magnus
   exponent, which is a right factor like any other and inherits both properties.

In the corpus's composition order (paper eq. 18) the reapplication is exactly
what `ESKF/predictPreintegrated.mo` already computes for the nominal state:

    q+ = q0 (x) dq
    v+ = v0 + g dt + R0 dv
    p+ = p0 + v0 dt + (1/2) g dt^2 + R0 dp

The gravity terms are the blocks of `L`; the `(dp, dv, dq)` triple is `R`.

## 3. The estimator interface

The horizon is **estimator-agnostic by construction**, so that an ESKF, the
existing manifold UKF, and a later equivariant filter can be compared with the
buffer, the predictor, and the re-base held bit-identical and only the filter
swapped. Nothing in `Estimation.FusionHorizon` mentions covariance, sigma
points, error states, tangent ordering, or injection.

The interface is not new. It is the algorithm-neutral boundary the corpus
already has, used in one direction each way:

| direction | carrier | content |
| --- | --- | --- |
| horizon to filter | `Avionics.ImuSample` | the accumulated SE_2(3) delta since the last fusion instant, its span, the bias anchor it was integrated at, and the five bias Jacobians |
| horizon to filter | `Avionics.{Mocap,Gps,Magnetometer,Barometer,OpticalFlow}Sample` | the aiding streams, passed through untouched |
| filter to horizon | `Avionics.NavigationEstimate` | the corrected pose at the fusion instant: position, velocity, quaternion |
| filter to horizon | `gyroscopeBiasBodyFlu_rad_s`, `accelerometerBiasBodyFlu_m_s2` | a bias VALUE, not an increment and not an injection |
| filter to horizon | `Avionics.EstimatorStatus.acceptedCorrectionCount` | the state-shifted signal that triggers a re-base, read as an EDGE: a change in the count is one accepted correction. `correctionOutcome` beside it is a LEVEL held for a whole filter tick and is NOT usable here |

Every one of those already exists on
`Estimation.StrapdownINS.PartialEstimator`, which both shipped filters extend,
so the filter enters `HorizonEstimator` through a `replaceable ...
constrainedby` slot and nothing else changes. `PartialEstimator` does also
publish `navigationCovarianceLocal`; the horizon never reads it, and the
horizon-facing subset is the table above.

**One field was added to that boundary, deliberately.**
`Avionics.EstimatorStatus.acceptedCorrectionCount` is new. The horizon has to
act exactly once per accepted correction, and there was nothing on the boundary
that could tell it so. `correctionOutcome` is a level that stands for the whole
filter tick, which at a 100 Hz filter behind an 800 Hz predictor is eight
predictor ticks; reading it directly fired eight full folds per correction and
made the WCET record's correction-rate ceiling eight times optimistic. A rising
edge on that level is no better, because back-to-back accepted corrections hold
it true across the filter-tick boundary and the second correction disappears. A
monotonic count is the only signal that gives a well-defined edge across a rate
change, so it lives on the boundary rather than being guessed at downstream.
Both shipped filters maintain it in three lines each.

**Bias coupling, stated as an interface rule.** Buffered deltas are integrated
at one anchor bias fixed at initialization. The estimator supplies a bias
value; **the horizon computes the difference from its own anchor and applies
the Jacobian correction itself**. The filter is never asked for an error-state
injection, an increment, or a covariance, which is what keeps the same path
usable by an additive-bias ESKF, a manifold UKF, and a filter whose bias lives
somewhere else entirely. The remainder of that first-order move is second
order and bounded by Prop. 8 (Sec. VI-A) at `(T_D ||db_g||)^2` with `T_D` the
window span, **not** the mission length: about 1e-4 rad at a 200 ms horizon and
a 0.05 rad/s bias offset, and it does not grow with flight time.

The anchor is deliberately never moved. Re-anchoring mid-buffer would mix
linearization points inside a single composed Jacobian, which is an error
nothing in a closed-loop test would show.

**The bias move is bounded, flagged, and never clamped.** The first-order move
is good over a stated ball around the anchor and no further.
`OutputPredictor.maximumGyroscopeBiasMove_rad_s` and
`maximumAccelerometerBiasMove_m_s2` name that ball; outside it the block raises
`biasMoveExceeded` and publishes the state as computed. Clamping the move
instead would put the predictor on a bias nobody estimated and report nothing,
which is the worse of the two failures: a flagged wrong answer can be demoted
by a supervisor, a silently corrected one cannot.

**The incremental path does not carry the bias move, and that is bounded by a
re-anchor rather than left open.** Only a re-base applies `db` to the buffered
window; the incremental path composes tick factors integrated at the anchor and
composes them onto its own previous answer. Between re-bases the predictor
therefore leaves the filter's own bias at `||db_g||`, without bound in the time
since the last re-base, and under sustained correction rejection that time is
the whole rejection episode. Two answers were available.

1. Move each tick factor as it is composed. Correct to first order, but it puts
   a `rebiasDelta` on the 800 Hz path, which is the one path the WCET record
   measures and the one the design's cost story rests on.
2. Bound the drift and force a fold when it would be exceeded.

This design takes (2). `maximumPredictorDivergence_rad` is the bound, the
accumulated divergence is `||db_g||` times the time since the last re-base, and
exceeding it makes the tick take the re-base path. The common tick pays one
comparison; the fold, when it happens, costs what a fold costs and the WCET
record charges the re-anchor rate against the same budget as the correction
rate. The consequence to state plainly: under sustained rejection the horizon
is MORE expensive, not less, and that is the honest direction for the trade to
run.

## 4. The rate structure

Aligned with the lattice commit 8e19eba established, and not fighting it:

- **800 Hz**, IMU and rate loop off the same data-ready interrupt. One
  FOH-corrected SE_2(3) delta per tick from two consecutive samples.
- **100 Hz**, one filter release per composed packet.
- **200 ms** fusion horizon: 160 buffered deltas at 800 Hz, 20 releases.

The paper records the gap this closes. Remark `rev:asbuilt` in Sec. VIII states
that every hold-order result presupposes samples between releases are
accumulated into a delta packet, that the flight firmware does not do this, and
that accumulation is therefore the precondition for any hold-order result to
apply on hardware. The corpus does accumulate, destructively, over 8 ticks. The
ring specified here is the same accumulation held for 160 ticks and non-
destructively.

**A delayed horizon costs one horizon of start-up.** Until the ring spans the
horizon plus one release window there is no fusion instant, no packet is
released, and the filter has not started. The predictor runs from the seed pose
meanwhile. Releasing early would stamp a packet with an epoch the filter is not
standing on, which is the failure the whole design exists to avoid.

### Anti-aliasing is an assumption, not code

The buffer integrates one sample per 800 Hz tick and claims first-order-hold
accuracy for it. That is only true for a stream band-limited below 400 Hz. It
is, in silicon, before sampling: the deployed ICM-45686 runs its gyroscope at
1600 Hz ODR with the on-die low-pass at ODR/8 = 200 Hz and the accelerometer at
ODR/16 = 100 Hz (`cerebri_rdd2`, `boards/mr_vmu_tropic.overlay`). The raw
32 kHz mechanical bandwidth never reaches this code. The model carries no
filter because its input is defined to be the already-filtered stream. This is
written into the `Documentation` annotation of the package and of the block,
and a change to the hardware filter configuration would invalidate the
hold-order error budget with nothing in the model detecting it.

## 5. The output predictor

State carried at 800 Hz:

- `ring`: a fixed-size discrete ring of deltas, each carrying `(dp, dv, dq, dt)`
  and the five bias Jacobians, 56 reals per entry. **One entry per release
  window, not per tick.** Composition is exact at any granularity by Lemma 5 and
  the fusion instant only ever lands on a release, so one entry per window is
  the same group element as eight per window at an eighth of the storage
  traffic. Length is `fusionHorizon_s / fusionPeriod_s + 2` (22 at the flight
  rates): the horizon, the window being accumulated, and one slot of slack so a
  release never races the store. No dynamic allocation, and the index arithmetic
  wraps at most once.
- `ringTail`, `ringCount`: the complete windows standing behind the live one.
- `liveRow`: the window being accumulated, carried as its own state rather than
  read back out of the ring, because reading one entry through a fold costs a
  whole window of group products for one row.
- the predicted pose at now.

Per tick, in order: **integrate** the newest interval by Theorem 1 and
**accumulate** it into the live window; **adopt** the completed window into the
ring and **release** the oldest if the horizon is full; **compose**.

- *incremental*, the common case: one group composition onto the previous
  answer. Correct because a window slide with no correction does not move now:
  the factor the filter consumed in its own prediction is exactly the factor the
  predictor already composed, and associativity does the rest.
- *re-base*, only when the filter shifted the horizon state: fold the whole
  buffer, move it to the filter's current bias in one Jacobian step, and compose
  onto the corrected pose. Theorem 6 applied literally.

**Re-base is triggered by an accepted correction, not by a window slide, and by
the EDGE of one.** This is the single most important cost fact in the design.
The fusion instant advances every 10 ms unconditionally; only a nonzero state
shift requires the recomposition, and one correction is one recomposition. The
signal is a change in `acceptedCorrectionCount`, not the level beside it, for
the reason given in Sec. 3. A re-base also fires when the accumulated bias-move
divergence would exceed `maximumPredictorDivergence_rad`, which is the other
half of the same cost story and is charged against the same budget.


### The epoch invariant

The single property everything in Sec. 5 rests on, stated where it is enforced
(`step.mo`) and asserted where it is observable (`Tests.HorizonInterfaceTests`):

> The window a re-base folds and the pose it composes that window onto name the
> SAME fusion instant.

The horizon pose arrives one inertial tick after the filter published it, so it
belongs to the fusion instant reached by the PREVIOUS release. The fold
therefore runs over the ring as it stood BEFORE this tick's adopt and BEFORE
this tick's release, and the window accumulated up to the previous tick rides
that fold as a trailing row, because on a release boundary it is no longer part
of the live delta and the ring the fold is given predates this tick's store.

Folding the advanced indices instead is not a rounding error and it is not
visible off a boundary. On a release tick it drops the entry the pose has not
absorbed yet and picks up the head slot, which on that tick still holds the row
from a whole ring ago, or zeros before the ring has wrapped. A zero row is a
zero quaternion, and `normalize(product(q, 0))` is the identity, so the composed
rotation collapses without a diagnostic. Written the way it is now the identity
holds on every tick with no case split at all.

### Preconditions, asserted rather than assumed

Three things the block used to accept quietly and now refuses:

| precondition | why it is not a preference |
| --- | --- |
| `fusionPeriod_s` an exact multiple of `samplePeriod` | the cadence is counted in inertial ticks, so a fractional ratio is rounded and every published epoch is wrong by the remainder |
| `fusionHorizon_s` an exact multiple of `fusionPeriod_s` | the buffer is counted in whole windows, same argument |
| `fusionHorizon_s` at least one `fusionPeriod_s` | at zero windows the first release hands over the ring slot the tick has not written yet: a zero span and a zero quaternion, and the consumer divides the rotation increment by that span |

`Tests.HorizonRefusals` is one model per precondition and the requirement on
each is that it does not run.

**And one gate that is not a precondition but belongs with them.** Deleting a
single status assignment from either shipped filter -- the accepted-correction
count the re-base trigger reads -- leaves every gate in the repository green
while corrections stop reaching the predictor entirely. `checkModel` reports
3182 equations against 3183 variables and still prints "completed
successfully", and the composed model is translation-only, so no simulation
runs to notice. `Tests/run-horizon.mos` therefore reads the equation and
variable counts out of every `checkModel` report and fails when they differ.

Say what that gate catches, because it is narrower than it looks: not that the
counter is wired to the re-base, which nothing here can assert without
simulating the composed block, but that every discrete in these models has an
equation. That is the shape this failure takes, and any other unassigned
discrete fails the suite the same way.

### Readiness means a packet exists

`horizonReady` is latched by the FIRST RELEASE. For one release window the ring
already spans `fusionHorizon_s` and no packet has been handed over, so the
epoch is still its seed value; a consumer that stood on "the ring is long
enough" would be standing on that seed. There is no separate `bufferOverflowed`
signal any longer: the release is unconditional, so the ring can never exceed
the horizon it is sized for, and the condition the signal named was
unobservable from inside the block. A supervision signal that cannot fire is
worse than none.

### Reset re-anchors the epoch

The packet epoch is carried, not read off a clock, so the only place it can be
tied to anything outside the block is the tick that drops the buffer. A reset
re-anchors it to a monotonic tick counter that survives the reset. Seeding it
at minus one sample period, which is right at power-on and was what the block
did everywhere, leaves a mid-flight reset publishing an epoch that is behind
wall time by the whole flight so far, permanently: downstream, aiding aligned
by timestamp would be rejected from the reset onwards. The counter rather than
`time` because the code generator refuses a runtime coordinate in production
code, and because two instances handed the same boundary values must still
agree exactly.

### Packet delivery is pulsed

`valid` and `fresh` on the released packet are true on the release tick and no
other. They are the same boolean on purpose: a window either was handed over on
this tick or was not, and there is no held packet to distinguish. The consumer
is the filter, whose clock IS the release clock, so it samples the packet on
the tick it is published. Anything wired to a different clock must latch it.

### Why the full fold and not the peel

A cheaper re-base exists. Because composition is a group operation,

    D(h' -> now) = D(h -> h')^-1 (x) D(h -> now)

peels the consumed span off a single running accumulator: one inverse and one
composition instead of 160. `dev/2026-08-25-hot-loop-output-predictor.md` in
the rumoca repository argues for exactly this, and it is algebraically exact
for the nominal state.

This design takes the fold anyway, for two reasons. The peel never re-anchors,
so single-precision error in the accumulator compounds without bound over a
flight, in the one quantity control reads. And the peel's bias Jacobians are
only first order through the inverse, which is a silent error: nothing in a
closed-loop test distinguishes a slightly wrong Jacobian from slightly wrong
tuning. The fold re-derives the window from stored deltas every time, so the
predictor's error is bounded by the horizon length rather than by mission
length. The cost of that choice is measured, stated separately in the WCET
table, and is the design's worst case. The peel stays available if the numbers
ever demand it.

The measured outcome is recorded in `delayed-fusion-horizon-wcet.md` and it does
not flatter this choice. The fold is inside budget as an algorithm and two
orders of magnitude outside it as generated, for a code-generation reason that
is identified there. The peel is the fallback those numbers would force if the
code generator is not fixed first, which is why the argument above is recorded
rather than assumed settled.

### What the predictor is not

It is not a second integrator. PX4's `output_predictor.cpp` runs
`calculateOutputStates()` as an independent earth-frame integrator and
reconciles it to the EKF with a tuned complementary filter
(`att_gain = 0.5 dt / time_delay`, chosen for a 0.7 damping ratio), because its
increments are formed in the earth frame and therefore depend on the attitude
anchor it is trying to correct. Its own comment concedes the buffer-wide
correction is too expensive for the attitude states. Ours integrates in the
anchor's body frame, so the increment is independent of the state by
construction and the reconciliation is an algebraic identity rather than a
tuned loop. One integrator with two consumers, not two integrators that must be
made to agree.

## 6. Files

New package `Estimation/FusionHorizon/`:

| file | kind | role |
| --- | --- | --- |
| `package.mo` | package | package head, `DeltaLength`, the assumptions |
| `Delta.mo` | record | one SE_2(3) right factor, its span, its five Jacobians |
| `Pose.mo` | record | position, velocity, attitude. No filter content |
| `identityDelta.mo` | function | the identity over a zero span |
| `packDelta.mo` / `unpackDelta.mo` | function | the one place the ring layout is known |
| `integrateSample.mo` | function | Theorem 1: two raw samples to one delta |
| `composeDelta.mo` | function | Lemma 5: delta (x) delta, with the time block and the Jacobian chain rule |
| `rebiasDelta.mo` | function | Prop. 8: the first-order bias move |
| `composePose.mo` | function | Theorem 6: `L X R` |
| `foldBuffer.mo` | function | the re-base kernel |
| `readRow.mo` | function | select one ring row without a dynamic index |
| `storeRow.mo` | function | store one row, branch-free and fixed-length |
| `jacobianBlock.mo` | function | read one 3x3 Jacobian out of a row |
| `step.mo` | function | one whole inertial tick, pure |
| `navigationEstimate.mo` | function | pose to `Avionics.NavigationEstimate` |
| `OutputPredictor.mo` | block | the clocked shell over the ring |
| `HorizonEstimator.mo` | block | horizon plus a `replaceable` filter |

Existing files edited: `Estimation/package.order`, `Tests/package.order`,
`tools/ci.py` (one Rumoca target, one OpenModelica script), and
`Avionics/package.mo` plus the two shipped filters, for the one boundary field
Sec. 3 records.

Every buffer operation is a **function**, not a when-body loop. That is a
deliberate accommodation of three known compiler limits: conditional
accumulation inside a `for` in a `when` body, constant folding on long foldable
loops, and the fact that the generated C does not survive a record inside a
multiple-output tuple. `step` therefore returns plain arrays, integers, and
booleans, and the caller assembles the packet record from a returned row.

## 7. Tests

| model | what it gates |
| --- | --- |
| `Tests/HorizonPredictorTests.mo` | the algebraic identities, constant-folded: composition, re-base by fold, incremental-versus-re-base THROUGH `step.mo`, and the bias move against Prop. 8 |
| `Tests/HorizonInterfaceTests.mo` | the time domain: the epoch-equivalence probe on every tick, a correction taken ON a release boundary, the epoch-against-buffer invariant, and packet validity |
| `Tests/HorizonResetTests.mo` | the epoch through a mid-run reset |
| `Tests/HorizonRefusals.mo` | the three preconditions, one NEGATIVE model each |
| `Tests/HorizonEstimatorWiring.mo` | filter interchange, at translation |

with their own `Tests/run-horizon.mos` entry. `when`-clause assertions are
vacuous under OMC and a fully constant-foldable model is evaluated away inside
`Tests.All`, so the entry point that gates the properties is a top-level
simulation, following `Tests/run-position-loop.mos`.

Measured residuals are recorded in the assertion comments so the next reader
sees the margin rather than a bare limit.

**Two things the first version of this suite got wrong, recorded so they are
not reintroduced.** The interface model drove a body-constant stream, which
makes every buffered window the SAME group element: folding the wrong
contiguous run of ring slots then produces the right answer and a slot-identity
error is algebraically invisible. The stream is now coning-rich and
sculling-rich, the same one the algebraic drivers and the WCET rig use. And the
correction was taken deliberately OFF a release boundary, with a comment saying
so; a boundary tick is exactly where the epoch and the window can disagree, so
the correction is now taken at 0.25 s, which is one.

**The sharpest test is the equivalence probe.** A fourth output predictor is
handed the exact pose the dead-reckoning reference stood on at the fusion
instant and told a correction was accepted on every ready tick. With nothing to
shift, re-basing must reproduce the incremental answer to floating point, on
release boundaries and off them alike. It exercises the fold, the ring indices,
the carried live window and the state machine on every single tick, and it does
not depend on the correction being nonzero. Measured worst disagreement over
the run: 1.5e-17 m, 3.3e-16 m/s, 2.2e-16 on the quaternion.

**A tool limitation, recorded rather than worked around.** OpenModelica cannot
build a simulation containing a bare
`Estimation.StrapdownINS.PartialEstimator`: it reports an independent subset of
the flattened estimator over-determined by nineteen equations. The existing
`Tests.StrapdownEstimatorInterfaceTests` fails identically on an untouched tree
at 8e19eba, which is why the corpus compiles it and never simulates it. The
ESKF-and-UKF-through-one-horizon model is therefore gated at translation by
OpenModelica, and the time-domain behaviour is tested against a filter stand-in
on the same declared boundary.

Rumoca does NOT lower that composed model, and an earlier version of this
section implied otherwise. What `tools/ci.py` lowers is
`Estimation.FusionHorizon.OutputPredictor`, all the way to galec-production;
that is the largest subset that lowers. The composed model is over by fourteen
equations per harness, and the cause is upstream and unrelated to this package:
Rumoca does not count the Boolean components of a sub-block's input connector
among the unknowns, `Avionics.PartialNavigationEstimator` carries fourteen of
them, and a seventeen-line reproducer plus the bisection is in
`tools/rumoca-repros/connector-boolean-balance/`.

## 8. Landing split

The horizon package, the predictor, and their tests land as a **parallel path**:
new files plus three one-line edits. The ESKF rewiring that moves
`ESKF/Estimator.mo` and `ESKF/step.mo` to fuse at `t - D` and retires
`retrodict`'s callers is a **follow-up**. Three reasons, all about risk:

1. The rewiring touches the correction supervision path -- anchor selection,
   staleness clocks, the recovery ladder -- which is where the measured failure
   narratives live. It wants its own qualification run, not a shared one.
2. The `ESKF/` subtree has concurrent in-flight work. New files collide with
   nothing.
3. The parallel path is independently useful and independently testable, and
   the rewiring only changes what the horizon's filter slot is wired to.
