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
| measurement-age alignment of the nominal state | `ESKF/retrodict.mo`, one held IMU sample over the age | still used, over a much shorter interval. `AidingBuffer` delivers a measurement at the first fusion instant at or after its own timestamp, so what remains is a residual inside one release window: 10 ms rather than the sensor's whole transport latency |
| aged measurement Jacobian `H_d Phi(-age)` | built in `ESKF/step.mo` | `Phi(-age)` is transported over the residual only. It is NOT the identity, and an earlier version of this table said it would be: a measurement timestamp falls between two fusion instants, so the transport runs over `[0, fusionPeriod_s)`. `H_d` is the half already mechanically verified (`lie-jacobian-eskf-proof`, commit 17767d3) |
| `maximumAidingDelay_s = 0.25` | admits packets whose cubic-Taylor transport is 12 to 27 percent wrong, against a deployed GPS age nearer 0.1 s | the transport is not used for aiding that reaches the horizon. What remains is the residual window for a packet that arrives AFTER its epoch has passed the horizon; that window is `age - fusionHorizon_s`, and the horizon length is chosen so it is normally empty. A packet later than the horizon is a supervision event, not a transport problem, and should be rejected on its timestamp |
| process noise over the aging window | the gain and the Joseph posterior neglect the process noise accumulated between the measurement timestamp and the fusion time | REDUCED, not removed, and an earlier version of this table claimed removal. The neglected window is the residual above, so the term shrinks with it by the same factor of twenty-five; what changes in kind is that the window is now a parameter of the release lattice rather than whatever age a packet arrived with |
| `correctMocap` has no retrodiction stanza at all, though the paper claims every `correct*` has one | a real asymmetry: mocap is aged like everything else and is aligned like nothing else | closed two ways. Every source now routes through the same queue, and `correctMocap` carries the same age stanza as the other four, because the residual alignment is needed there too. An asymmetry that only the routing closed would have left mocap the one source with no sub-window transport |
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

**And the tolerance is not a free parameter.** The re-anchor rate at the
largest bias move the block declares it will tolerate is
`maximumGyroscopeBiasMove_rad_s / maximumPredictorDivergence_rad`, and that has
to fit in what the fold budget has left after the corrections the design exists
to serve:

    maximumGyroscopeBiasMove_rad_s / maximumPredictorDivergence_rad
      <= foldBudget_hz - correctionRateBudget_hz

The block asserts it. The first version of these parameters did not, and did
not satisfy it either: 0.05 rad/s against a 1e-3 rad tolerance is fifty folds a
second against a measured budget of 7.3, seven times the ceiling, in a
configuration nothing refused. The tolerance the budget actually leaves is
0.025 rad, so that is the default, and the WCET record states the price in
attitude that buys.

## 3b. The delayed measurement queues

`Estimation.FusionHorizon.AidingBuffer`. This is the half the first landing
did not have, and the gap it leaves is worth stating exactly, because it is
sharper than "incomplete".

**A horizon without measurement queues fuses nothing.** The predictor moves the
filter's epoch back to `t - D`. The aiding streams stay at the live edge. So
every sensor packet is stamped AHEAD of the instant the filter is standing on,
the age `imuTimestampHeld - source.timestamp_s` comes out NEGATIVE, and
`ESKF/step.mo` refuses it as `CorrectionRejectedTimestamp`. The only aiding
that ever reached a filter behind the horizon was in
`Tests.HorizonEstimatorWiring`, which hand-stamps its mocap sample at
`time - fusionHorizon_s` so that its age is zero by construction. The merged
composition was therefore correct in every part and inert as a whole.

### What a queue does

One bounded FIFO per source, holding measurements packed into flat `Real` rows
whose column 1 is the timestamp. `stepQueue` knows nothing else about any row,
which is what lets one kernel serve five sources of five different widths.

Per inertial tick, in order: **deliver**, then **admit**. Deliver first so a
full queue can drain on the tick it is written to. Delivery happens only on a
release, pulsed onto the same clock the inertial packet is pulsed onto, because
the two packets the filter consumes on one tick have to name one fusion
instant. The epoch is read off the released inertial packet rather than
recomputed, so that identity is structural.

An entry is RIPE when the fusion instant has reached its timestamp. That is the
whole delayed-fusion contract: at the instant the filter stands on, the
measurement is no longer in the future, so fusing it is fusing a measurement at
its own epoch rather than transporting one backwards to meet a state that has
moved on.

### The residual, which is the quantitative claim

A measurement ripens on the first release at or after its own timestamp, so the
offset between the two is in `[0, fusionPeriod_s)` by construction. That offset
is `maximumResidualAge_s`, it is the interval `retrodict` and `Phi(-age)` now
run over, and it is published as `worstDeliveredAge_s` rather than argued.

Two ratios follow, and they are different sizes. Keeping them apart matters,
because quoting the larger one for the smaller one's claim is the easiest way
to overstate this whole change.

**Bound against bound:** 0.01 s against the 0.25 s `maximumAidingDelay_s`
admits, a factor of twenty-five. That is the worst case the live-edge path
would accept measured against the worst case this one can produce, and it is
the figure the source comments carry.

**Actual against actual:** 0.008750 s against the 0.110 s a GPS fix really
arrives with, a factor of 12.6. That is what the deployed configuration
actually gains, and it is smaller than the bound ratio because the deployed
GPS is nowhere near the bound the live-edge path tolerates.

The transport error is cubic in the interval either way. Measured in
`Tests.AidingHorizonTests` at the flight lattice and the flight latencies:
worst delivered residual 0.008750 s against a derived bound of 0.01 s, on a
stream whose fixes arrived 0.110 s old.

One consequence of the new latencies belongs here rather than buried. With GPS
at PX4's 110 ms, a live-edge fix is up to 110 ms plus one 100 ms sample period
old, so its worst age is about 0.21 s against a `maximumAidingDelay_s` of
0.25 s. The live-edge path therefore runs with about 16 percent margin on its
own admission bound, not the comfortable fraction the old 0.1 s placeholder
implied.

### Four outcomes, all named

| outcome | when |
| --- | --- |
| `AidingQueued` | stored, to be fused when the fusion instant reaches it |
| `AidingRefusedLate` | the fusion instant had already passed it by more than the residual bound. There is no instant left to fuse it at. This is the outcome that replaces transporting a Jacobian a quarter second backwards |
| `AidingRefusedOverflow` | the queue was full. The queue keeps what it holds and the NEW measurement is lost |
| `AidingDroppedStale` | admitted, then discarded at delivery. Reachable by a packet whose latency lands between `fusionHorizon_s` and `fusionHorizon_s + maximumResidualAge_s`: delivery reads the queue as it stood before this tick's store, so a measurement arriving already ripe waits one release and the instant has moved a window past it by then. An earlier note called this unreachable, which was wrong |
| `AidingBeforeHorizon` | presented before the first release. NOT a refusal and not counted as one: the horizon costs one horizon of start-up, during which the filter had no inertial packet either. The sample is left unconsumed, so a held packet is admitted on the first tick after the horizon becomes real |

plus `AidingDroppedStale` on the delivery side, for an entry that was ripe but
older than the residual bound.

### The overflow policy is the opposite of a live-edge buffer's

This took an adversarial test to see, and the intuition runs the wrong way.

A live-edge buffer that overflows should drop its OLDEST entry: the newest
measurement is the most useful. A delayed queue must drop the NEWEST, because
the oldest entry is the one the fusion instant is about to reach. Displace it
and the queue becomes a sliding window of the newest arrivals, every one of
which is still in the filter's future when the next arrival displaces it.
Nothing ever ripens. The source is silent for the whole flight while every
arrival is dutifully stored, and every assertion about ordering and residuals
still passes, because a queue that delivers nothing violates none of them.

Measured on a source delivering forty times its declared rate: 0 deliveries
with the oldest displaced, 47 deliveries with the arrival refused.

Refusing the arrival cannot deadlock. On every release the oldest entry either
delivers or, if the fusion instant has passed it, is discarded as stale, so one
slot frees per release whatever the source does. The queue degrades to
delivering at the release rate rather than to delivering nothing.

### Capacity, and why it is the whole horizon

In steady flight a measurement stamped `t_m` reaches the driver at `t_m + L`
and ripens at `t_m + D`, so it waits `D - L` and the queue stands at
`(D - L) / P` entries. STARTUP is the worse case and is the one the capacity
covers: the fusion instant does not advance until the delta ring has filled, so
everything a source delivers during that first horizon is queued at once, which
is `D / P`. Depths are `ceil(D / P) + slack`, fixed at translation, and final
rather than overridable, because a capacity and the rate it is derived from
must not be settable independently.

At the flight lattice: mocap 22, GPS 4, magnetometer 6, barometer 12, optical
flow 22 slots, 1312 reals in all, 5,248 B at single precision.

### The horizon must cover the slowest source, and now says so

`fusionHorizon_s >= maximumSourceDelay_s + horizonJitterMargin_s`, asserted in
`AidingBuffer`, with `Tests.AidingHorizonRefusals.ShortHorizon` as the negative
test.

PX4 ships the same relation without the headroom: `EKF2_DELAY_MAX`, documented
as "the delay between the current time and the delayed-time horizon", "should
be at least as large as the largest `EKF2_XXX_DELAY` parameter". Its default is
**200 ms**, which is the same quantity and the same value as
`fusionHorizon_s`. Worth recording plainly: the 200 ms horizon is not this
corpus's invention, it is what the reference implementation runs.

The margin is what makes the older-than-the-horizon refusal an ANOMALY path
rather than a routine one. A declared delay is a nominal, not a bound; a
horizon sized exactly to the nominal turns every late delivery into a refused
measurement. At the flight configuration the sum is 110 + 50 = 160 ms against a
200 ms horizon, so it clears by a further 40 ms.

### The plant-side sensor delays are PX4's, not placeholders

`Vehicles/Rdd2/WaypointVehicleSystem.mo` models how old each measurement
already is when a driver hands it over. Those latencies were placeholders; they
are now taken from PX4-Autopilot `src/modules/ekf2/params_*.yaml`, read
2026-08-28, because a delayed-aiding result computed against a placeholder says
more about the placeholder than about the filter.

| source | model parameter | value | PX4 parameter |
| --- | --- | --- | --- |
| GPS | `gpsLatency_s` | 0.110 s | `EKF2_GPS_DELAY`, v1.15.0 default 110 ms |
| optical flow | `opticalFlowLatency_s` | 0.020 s | `EKF2_OF_DELAY`, main default 20 ms |
| magnetometer | `magnetometerTransportDelay_s` | 0.0 s | `EKF2_MAG_DELAY`, main default 0 ms |
| barometer | `barometerTransportDelay_s` | 0.0 s | `EKF2_BARO_DELAY`, main default 0 ms |
| mocap | none | not modelled | `EKF2_EV_DELAY`, main default 0 ms |

Three things about that table are worth stating rather than leaving to be
noticed.

**GPS is cited from v1.15.0 because the parameter no longer exists.** The whole
EKF2 delay family on main is `EKF2_ASP_DELAY` 100, `EKF2_AVEL_DELAY` 5,
`EKF2_BARO_DELAY` 0, `EKF2_EV_DELAY` 0, `EKF2_MAG_DELAY` 0, `EKF2_OF_DELAY` 20,
`EKF2_RNGBC_DELAY` 0, `EKF2_RNG_DELAY` 5, all in ms, and GNSS carries no delay
parameter at all: the driver's own sample timestamp replaced it. 110 ms is
PX4's last shipped default, used because this model needs an explicit
plant-side latency where PX4 now needs none.

**Zero for the barometer and the magnetometer is a topology claim, not a
rounding.** PX4 ships zero because those are onboard sensors read by the
autopilot that reads the IMU, timestamped against one clock with no transport
to model. This vehicle matches that topology, so zero is the honest value; an
off-board magnetometer arriving over a link would not be entitled to it. The
previous 0.02 and 0.04 described a bus this vehicle does not have, and made the
barometer the second most delayed source in a model whose reference treats it
as undelayed.

**A zero delay is expressed by the ABSENCE of a `delay()` operator.** Rumoca
requires `delayTime` to be a finite positive scalar and refuses the operator
otherwise, which is the right refusal. The two capture equations therefore read
plant truth directly and the two parameters are `final`, so a modification
cannot move the packet timestamp without moving the measurement and leave the
plant reporting an age it never applied.

`EKF2_ASP_DELAY` and `EKF2_RNG_DELAY` are not adopted: this vehicle carries no
airspeed sensor, and its downward range arrives inside the optical-flow packet
rather than as an independent aided source.

### Capacity is unchanged by the new delays, and here is why

A depth is `ceil(D / P) + slack`, which depends on the horizon and the source
PERIOD and not on the source delay, so no depth moved. What moved is the
steady-state occupancy, `(D - L) / P`, and every case still fits:

| source | period | depth | steady before | steady after | startup |
| --- | --- | --- | --- | --- | --- |
| GPS | 0.1 | 4 | 1.0 | 0.9 | 2.0 |
| magnetometer | 0.05 | 6 | 3.6 | 4.0 | 4.0 |
| barometer | 0.02 | 12 | 8.0 | 10.0 | 10.0 |
| optical flow | 0.01 | 22 | 19.0 | 18.0 | 20.0 |

Startup is the binding case and it is delay-independent, which is the reason
the capacities did not have to move: a shorter latency means a measurement
waits longer in the queue, and the worst wait is the whole horizon whatever the
latency is.

### Bias coupling on the filter side

The released window carries the bias anchor it was integrated at and its five
Jacobians, and `StrapdownINS/correctPreintegratedImu.mo` applies the first-order
move to the filter's current bias when the filter CONSUMES it. So the filter
side needs nothing new, and the bound is sharper than the predictor's: a
released window spans ONE `fusionPeriod_s`, not the horizon, so the Prop. 8
remainder is `(fusionPeriod_s ||db_g||)^2` rather than `(fusionHorizon_s
||db_g||)^2` -- a factor of 400 at the flight lattice, about 2.5e-7 rad at a
0.05 rad/s bias offset. Buffered-but-unconsumed windows are not touched when
the bias moves, and must not be: each carries its own anchor and is moved when
it is consumed.

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

Four things the block used to accept quietly and now refuses:

| precondition | why it is not a preference |
| --- | --- |
| `fusionPeriod_s` an exact multiple of `samplePeriod` | the cadence is counted in inertial ticks, so a fractional ratio is rounded and every published epoch is wrong by the remainder |
| `fusionHorizon_s` an exact multiple of `fusionPeriod_s` | the buffer is counted in whole windows, same argument |
| `fusionHorizon_s` at least one `fusionPeriod_s` | at zero windows the first release hands over the ring slot the tick has not written yet: a zero span and a zero quaternion, and the consumer divides the rotation increment by that span |
| the supervision parameters inside the fold budget | a divergence bound the target has no cycles to honour is not a bound; see Sec. 3 for the inequality |

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
| `Tests/HorizonRefusals.mo` | the four preconditions, one NEGATIVE model each |
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

## 8. Landing split (superseded; see Sec. 9)

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

## 9. What has landed, and what the vehicle integration waits on

The follow-up Sec. 8 deferred has now landed as far as it can be validated, and
the part that cannot be is named here rather than left implied.

**Landed.** `AidingBuffer`, the `stepQueue` kernel and the five pack/unpack
pairs; the age stanza on `correctMocap`; `HorizonEstimator` routing every
aiding stream through a queue, reading the queue epoch off the released
inertial packet, and binding the filter's own aiding-delay bound to the queue's
residual bound; `Tests.AidingHorizonTests` and
`Tests.AidingHorizonRefusals` under `Tests/run-horizon.mos`, with the balance
gate extended to six reports; and `AidingBuffer` as a Rumoca
galec-production target in `tools/ci.py`, which lowers in about one second.

**Not landed: the RDD2 vehicle configuration, and it is blocked rather than
deferred.** Neither tool can run a mission built on the horizon composition
today.

- OpenModelica cannot BUILD a model containing `HorizonEstimator`. Measured
  twice for this change, because the obvious suspicion was worth eliminating:
  a single-instance harness using the block's `replaceable` filter slot was
  still building after TEN minutes, and a second harness that redeclares the
  slot to the concrete ESKF -- removing the replaceable indirection entirely --
  was still building after FORTY, at which point it was killed rather than
  waited out. So the cause is not the replaceable slot, and there is no
  cheap way around this by making the composition concrete. It is the
  limitation already recorded for a bare `PartialEstimator` in
  `Tests.HorizonEstimatorWiring`, and it is why every property in
  `Tests.AidingHorizonTests` is tested up to the filter's connector rather
  than through it.
- Rumoca cannot LOWER it: unbalanced by 26 equations, which is exactly the
  connector Boolean balance gap, 14 on the filter's six input connectors plus
  12 on the aiding buffer's five. The RDD2 qualification missions are simulated
  by Rumoca, so this is the binding constraint on the deployment gate.

The mission qualification is therefore not "passing" or "degraded"; it cannot
be executed. The live-edge ESKF stays the RDD2 default, which is the
conservative reading of that, and `tools/ci.py` carries a
`PIN_DEPENDENT_MODELS` entry that pins the exact 26 and fails if the model ever
stops lowering for a DIFFERENT reason. It reports plainly when the model starts
lowering, which is the moment the entry should be promoted and the
qualification run.

**The vehicle change the configuration will need, recorded so it is not
rediscovered.** `Vehicles/Rdd2/WaypointVehicleSystem.mo` selects its estimator
through `replaceable block EstimatorModel ... constrainedby
Estimation.StrapdownINS.PartialEstimator`, and fills `estimator.imu` with the
100 Hz preintegrated packet. `HorizonEstimator` cannot drop into that slot as
it stands, for one concrete reason: it needs the RAW 800 Hz inertial stream and
that boundary does not carry one. `estimator.imu.angularVelocityBodyFlu_rad_s`
is the packet AVERAGE, `deltaAngle / integrationTime`, held for eight ticks, so
feeding it to the predictor's integrator would hand a 100 Hz staircase to a
block whose whole claim is first-order-hold accuracy at 800 Hz.

Two ways out, and neither should be taken before the qualification can be run:

1. Carry the raw inertial stream on `Estimation.StrapdownINS.PartialEstimator`
   as two inputs. Balance-neutral for both shipped filters, which ignore them.
   The cost is real and is not model-side: `Vehicles.Rdd2.NavigationEstimator`
   is exported as the `rdd2-estimator` eFMU, so its interface gains two ports
   that the deployed wrapper must fill.
2. Buffer the vehicle's existing 100 Hz packets instead of integrating raw
   samples. The ring is already release-granular and a vehicle packet IS one
   window's SE_2(3) delta with all five Jacobians, so this fits the deployed
   lattice exactly and needs no boundary change. It costs a second predictor
   block, and it gives up the 800 Hz republication that the output predictor
   exists for.

(2) is the better fit for the deployed rate lattice and (1) is the smaller
diff. The choice wants the qualification numbers that neither can produce yet,
which is why it is recorded rather than made.

## 10. Compiler and tool defects this work hit

Recorded with what each one does, because every one of them is SILENT and each
makes a check pass rather than fail.

| tool | defect | effect |
| --- | --- | --- |
| Rumoca 0.10.0 | a user function call in an array-dimension expression is not folded once the block is a SUB-component; the same arithmetic written inline is | `AidingBuffer` reported unevaluable dimensions inside `HorizonEstimator` while lowering standalone. Worked around inline, with the site marked |
| Rumoca 0.10.0 | Boolean components of a sub-block's input connector are not counted among the unknowns (AS-051) | `HorizonEstimator` unbalanced by 26. Upstream fix in flight; `tools/ci.py` pins the number |
| OpenModelica 1.27 | a sub-component's output-connector Real members read as zero from the parent, while the Boolean members beside them read correctly | a delivered packet whose row held timestamp 0.03 read as `timestamp_s = 0` in a parent when-clause and in the result file. Every assertion about payload contents passed |
| OpenModelica 1.27 | a whole-record equality between two connectors publishes zeros on the target | a test arm wired that way was never valid, so its queue admitted nothing and every assertion about it passed vacuously |
| OpenModelica 1.27 | a driven input connector's Boolean did not report at all in the result file | made the two above much harder to localize |
| OpenModelica 1.27 | `buildModel` on a model holding a navigation estimator sub-component does not complete | no time-domain test of the composed horizon is possible; every property is tested against the filter's connector or inside the block |

The three OpenModelica ones are why `AidingBuffer` publishes
`deliveryOutOfOrder` and `deliveryAfterHorizon` itself rather than leaving
those to a consumer. That is weaker evidence than an external check and the
test model says so.

## 11. Two states on the boundary, and which consumer gets which

Once the filter fuses at `t - D` the block holds two different answers to
"where is the vehicle", 200 ms and one horizon of motion apart.
`HorizonEstimator` therefore publishes `predictedEstimate` and
`horizonEstimate` and **no field called `estimate`**. Removing the generic name
is the point: a consumer cannot pick one by accident because there is nothing
generic to pick. The two carry different timestamps -- now, and the fusion
instant -- so a mis-wiring is visible in any log rather than only in flight.

`horizonEstimate` is built from the same latched filter pose the re-base
composes onto, not from a second read, so the state the predictor was built on
and the state published for analysis cannot disagree.

### Which one, per consumer

Audited across `Vehicles/Rdd2` and the whole repository.

| consumer | wire to | why |
| --- | --- | --- |
| `AvionicsSystem` `manualTask.navigation` | **predicted** | anchors the carrot latch and the leash |
| `AvionicsSystem` `guidanceTask.navigation` | **predicted** | position/velocity/attitude cascade |
| `AvionicsSystem` `rateTask` body rates | **predicted** | 1600 Hz rate loop, 50 ms time constant |
| `Controller.mo` `navigation` port | **predicted** | a SECOND, independent `NavigationEstimateInput`, not reached through `PartialController` |
| `FlightModeSelector` | nothing | reads no estimate field at all; decodes RC only |
| `LogLinear`/`Proportional`/`ReducedRate` controllers | nothing directly | inherited or unconnected |
| `navigationError_m` | **decide at integration** | today one signal serves as both a guidance metric and an estimator metric; at a horizon those want different states |
| `controllerEstimatorFeedbackError_m` | **analysis** | best repurposed as the now-minus-horizon prediction magnitude |
| Python qualification harness | **analysis** | reads `estimator.estimate.*` by STRING NAME and will pick up whichever state keeps the old name |

Three findings from that audit are worth stating rather than leaving in a
table.

**The rate loop is not a degradation case, it is an instability case.**
`RateControlAllocator` runs at 1600 Hz with a rate gain of 20 s^-1, a 50 ms
loop time constant. A 200 ms delay is four time constants of pure dead time.

**Guidance and the carrot must be on the SAME epoch as each other, not merely
both on a sensible one.** The LogLinear position integral is driven by
`positionReference - positionWorld`, where the reference comes from the carrot
and the feedback from guidance. Split those across the two states and the
integral sees a constant fictitious error of `D` times ground speed -- 1.0 m at
5 m/s -- and winds to its limit of 0.25 g, about 2.45 m/s^2 of spurious tilt
command. This is the failure that a per-block test cannot see, because each
block is individually correct.

**The carrot's bumpless entry is the sharpest single case.**
`ManualTrajectorySource` latches `basePose := vehiclePose` on the entry sample,
seeding position, velocity and heading together from the estimate. On the
horizon state the entry starts from where the vehicle was `D` ago: 1.0 m of
commanded position step at 5 m/s, and 0.3 rad -- 17 degrees -- of heading step
at the 1.5 rad/s pilot heading rate, which rotates the whole stick frame. The
leash has the same anchor twice, and it cancels exactly while no bound binds
and stops cancelling the moment one does, turning a symmetric 2.0 m leash into
roughly 1.0 m ahead of the vehicle and 3.0 m behind it.

### The closed-loop assert, specified and not yet runnable

The signature of consuming the wrong state is a persistent along-track offset
of `D` times ground speed between commanded and achieved position, and the
required gate is that no such offset appears. It is NOT added here, because
nothing can run it: no tool in this tree builds a mission containing
`HorizonEstimator`. Adding a check that cannot execute would be the dead
assertion this package refuses elsewhere. It is the gate the vehicle
integration must bring with it, and the existing witness needs care --
`Test/ManualFlightMission.mo` measures the carrot residual against the same
`avionics.navigation` signal the carrot latched from, so on a single shared
epoch it is self-consistent and passes while the vehicle is really 0.2 to 1.0 m
off.

## 12. Counting shifted fusion instants, not corrections

`Avionics.EstimatorStatus.acceptedCorrectionCount` now states the contract it
always needed: **at most one increment per estimator tick**, counting ticks on
which at least one correction was accepted rather than counting measurements.
The predictor recomposes once per tick on which the pose it composes onto
moved, and two measurements fused into one tick move it once.

Both shipped filters already satisfied this, by construction rather than by
arithmetic: each fuses at most one source per tick from a priority chain, so
the per-tick outcome is a single value. The contract is now written on the
field and on both filters, so a filter that fused several sources in one tick
would know it has to coalesce.

**What the audit of that arithmetic actually found is more serious than the
counting.** The predictor's fold budget was charged a nominal 5 Hz of accepted
corrections, documented as covering a 5 Hz GPS fix rate. Behind the delayed
measurement queues that number is wrong in kind. Every measurement the horizon
reaches is ripe; the filter can accept one on every tick; and this vehicle's
aiding set offers a candidate on essentially every tick. **The shifted-instant
rate is the FUSION rate, 100 Hz at the flight lattice, against a measured fold
budget of 7.3 Hz.** A fourteenfold overrun.

`HorizonEstimator` therefore forwards `correctionRateBudget_hz = 1 /
fusionPeriod_s` to the predictor, so the block's existing budget assertion sees
the true number -- and refuses. `Tests.HorizonRefusals.FusionRateOverBudget`
demonstrates the refusal on the predictor alone, where it can actually be built
and run.

That refusal is the honest state of the architecture on today's code generator,
and it is a code-generation limit rather than an architectural one: the fold
costs about 2500 times its algorithmic content because a record-valued call is
materialized once per component. Section 13 is the change that removes the
dependence on that fix for the re-base path.

## 13. Retiring the earliest factor is exact, and what follows from it

### The result

A composed window divided by its own earliest factor returns the rest of the
window EXACTLY. `Estimation.FusionHorizon.retireDelta` is the closed form and
`Tests.HorizonPredictorTests` measures the residual below 1e-15 in position,
velocity and attitude and below 1e-14 in every bias Jacobian block, over two
160-tick factors of the coning-rich and sculling-rich stream.

**This refutes a claim in this document.** Section 5 rejects the peel partly
because "the peel's bias Jacobians are only first order through the inverse,
which is a silent error". For the composition this package actually uses they
are not first order, they are exact, and the reason is structural rather than
lucky: read `composeDelta` as a map from the second factor to the composed one
with the first held fixed, and every line is a known quantity plus a ROTATION
times a block of the second factor. An affine map with orthogonal leading
coefficients is solved by a subtraction and a transpose. Nothing is
linearized and nothing is inverted. The other reason Section 5 gives, drift in
an accumulator that never re-anchors, is untouched by this and is dealt with
below.

### What that enables, and what is NOT built

A correction never changes the buffer. It moves only the left anchor, so the
composed window product is invariant across corrections and can be MAINTAINED
rather than recomputed: on release compose the new window on the right, on
retirement divide the oldest out on the left, and a re-base becomes one
composition regardless of horizon length. That removes the fold-rate ceiling
from Section 12 entirely and decouples correction cost from horizon length.

**Only the enabling algebra is shipped here.** `retireDelta` and its exactness
test are in the tree; the maintained product is not wired into
`OutputPredictor`, and the re-base still folds. Three reasons, in order of
weight.

1. **The rotating shadow rebuild does not fit this ring.** The drift answer for
   a maintained product is to rebuild it from the stored windows and swap. At
   one composition per fusion tick a rebuild of `horizonWindows = 20` takes 20
   fusion ticks, during which the ring must still hold the rows being rebuilt
   from. The ring is `horizonWindows + 2` -- two slots of slack -- so those rows
   are overwritten after two releases. Making the rebuild fit means roughly
   doubling the ring, which doubles the buffer memory the WCET record is
   written against. That is a design fork with a real cost, and it wants
   deciding rather than guessing.
2. **It changes the re-base path of a block whose epoch-consistency bug is
   recorded as having cost a full review cycle**, and no tool in this tree can
   simulate the composed estimator to check the result end to end.
3. **It changes nothing about deployability today.** The composed block neither
   builds under OpenModelica nor lowers under Rumoca, so a faster re-base does
   not move the gate.

### The numerical guarantee the maintained product would have to carry

Stated now because it is the input to the tolerance the eventual swap-boundary
assertion must use, and because it is cheap to derive and expensive to
rediscover.

**Finite memory.** A swap REPLACES the maintained product with one rebuilt from
stored windows, and a stored window itself retires after `k` releases. No
floating-point error can survive more than one rebuild period plus one window
lifetime, so the predictor path holds no state in which unbounded accumulation
can live. The bound at any instant is one `k`-fold's error plus at most `k`
inverse-update pairs since the last swap. That is a hard bound, not a
statistical one.

**Why it is small.** Rotations are isometries and every step normalizes its
quaternion, so constraint violation sits at machine epsilon and does not
accumulate; velocity and position errors are rotated, which preserves their
norm, and added. Error grows like `N * eps * scale` with `N` around `3k`, about
60 operations at the flight geometry. At a 200 ms horizon and 5 m/s, scale is
about 1 m in position, 5 m/s in velocity and unity on the quaternion:

| precision | position | velocity | attitude |
| --- | --- | --- | --- |
| binary64, simulation | 1.3e-14 m | 6.7e-14 m/s | 1.3e-14 rad |
| binary32, the flight artifact | 7.1e-6 m | 3.6e-5 m/s | 7.1e-6 rad |

The flight-precision numbers are the ones that matter, and they are seven
microns and four ten-thousandths of a degree, against a predictor divergence
tolerance of 0.025 rad. The boundedness claim is comfortable in single
precision by four orders.

**Runtime enforcement.** The swap-boundary assertion must take its tolerance
from that binary32 row rather than from a hand-picked constant, the same
discipline the Proposition 8 arms already follow, so that a subtly wrong
left-division, a conditioning pathology or a compiler defect fires within one
swap period in flight instead of drifting quietly. The long-run test that goes
with it must record the maintained-versus-folded residual at every swap over
hundreds of swaps and assert it is STATIONARY as well as bounded: a trend is
the signature of an error surviving the swap through a path the analysis
missed, which is exactly what a bound alone would not catch.

**The corollary worth stating.** The only long-lived state in the pipeline is
then the filter's own estimate, whose error dynamics are governed by
measurement corrections and are the filter's own business, with its own
consistency machinery. The predictor's boundedness composes with that and does
not depend on it.

## 14. The maintained window product, as built

The re-base no longer folds the ring. `step.mo` carries the composition of the
ring entries and advances it by one composition when a window is adopted and
one exact division when the oldest is retired, so a correction costs ONE
composition onto the trailing row regardless of horizon length. The
fold-rate ceiling of Section 12 stops being a function of the horizon.

The enabling fact is Section 13: the left division is exact, measured below
1e-15. The decision record's stated fallback reason -- that a peel's bias
Jacobians are only first order through the inverse -- does not apply to this
composition and is withdrawn. Drift, the record's other reason, is what the
rebuild below answers.

### The rebuild is prospective, and that is why the ring did not have to grow

A carried product is exact in real arithmetic and drifts in floating point, and
an accumulator never forgets a rounding error. So a second product is built
from stored rows and REPLACES the carried one every `horizonWindows` adopts:
no error can outlive one rebuild period. That is the finite-memory guarantee,
and it is a hard bound rather than a statistical one.

The obvious way to rebuild is retrospective: snapshot the current window and
walk it. That needs those rows to survive the whole rebuild, which is a second
horizon of retention and a ring of `2k`, and the rebuilt product then lands
`k` ticks stale and needs a catch-up that is itself `O(k)`.

**The rebuild here targets the window that will exist when it FINISHES.** Each
adopted row is composed on the tick it is adopted, in the order the window
needs. After `horizonWindows` adopts the accumulated rows are exactly the last
`horizonWindows` adopted, which is exactly the window the next tick presents,
so the replacement lands with no catch-up and no stale rows. The rebuild never
reads a row older than the one being adopted, so **the retention requirement is
the live window and nothing more.**

That was checked against `step.mo`'s own index bookkeeping before it was
written, by replaying the adopt-then-release-when-count-exceeds-`k` sequence and
comparing the rebuild's row set against the ring's window at every replacement.
They agree at every one, at `bufferLength = horizonWindows + 2`. **The ring is
NOT doubled**, and the earlier conclusion that `2k` retention is fundamental
holds only for the retrospective ordering.

### Cost

Per FUSION tick: one composition to append the adopted window, one division to
retire the oldest, one composition into the rebuild. Three group operations,
flat, on the tick a window closes and nothing on the seven ticks between. A
re-base is one composition plus the trailing row, against the twenty-three the
fold cost. Ring memory is unchanged.

### What is asserted, and where

| check | where | limit |
| --- | --- | --- |
| carried product equals the rebuilt one | `OutputPredictor`, equation section, every rebuild period | `maximumProductResidual`, derived from the binary32 rounding floor of one rebuild period with a factor of a hundred in hand |
| the same, through the real state machine | `Tests.HorizonPredictorTests`, `incremental[4]` | 1e-12, a binary64 simulation limit four orders under the block's own |
| left division is exact | `Tests.HorizonPredictorTests`, `retirement[1..4]` | 1e-15, 1e-14 on the Jacobians |

The block's assertion lives in the equation section rather than in the clause
that produces the number, for two reasons that agree: an assertion inside a
`when` is vacuous under OpenModelica, and Rumoca refuses to read a variable in
an assertion after an earlier write in the same algorithm.

The retired rows are immutable, so how long a row is retained does not enter
the tolerance derivation. Only the rebuild period does.

## 15. The alternative that was evaluated and not taken: predicted-state snapshots

Instead of storing increments, store the predicted state at each fusion instant
plus a cumulative bias Jacobian, and compute the window transform fresh from
two rows: `W = X_h^-1 (x) X_now`. This is PX4 EKF2's output-predictor shape. It
deletes the carried product, the division chain, the rebuild and the
maintained-equals-rebuilt invariant in one move, because every `W` is a fresh
two-operand computation over immutable rows and nothing accumulates.

### The duality, which is exact

The two designs are the same algebra with the storage transposed. If
`S_i = D_1 (x) ... (x) D_i` are prefix products from a common base, then
`S_{i-1}^-1 (x) S_i = D_i` -- and that left division is precisely
`retireDelta`. **Snapshots relative to a common base ARE the prefix products,
and differencing two of them IS the operation Section 13 measured exact to
1e-15.** So the snapshot design's per-window filter input and its relative
Jacobian recovery inherit that exactness result directly; item 2 of the
evaluation needed no new measurement.

It is the classic prefix-sums-versus-increments trade. Prefix sums answer a
range query in `O(1)` but lose relative precision when the prefix magnitude
greatly exceeds the range queried; increments keep full relative precision but
need `O(k)` for a range query, unless the product is maintained, which is
Section 14.

### What decided it: binary32 conditioning of the position block

Snapshots are global-frame, so the window displacement is `R^T (p_now - p_h)`:
a metre-scale quantity differenced out of two position vectors whose magnitude
is the distance from the local origin. Velocity is metres per second and does
not grow, and quaternion components are bounded by one, so **the cancellation
concern is the position block and nothing else.**

Measured, binary32, against a 1 m window displacement:

| distance from origin | snapshot error | delta-design error |
| --- | --- | --- |
| 100 m | 9.3e-6 m | 4.4e-8 m |
| 1 km | 7.7e-5 m | 4.4e-8 m |
| 10 km | 1.0e-3 m | 4.4e-8 m |
| 100 km | 8.8e-3 m | 4.4e-8 m |

The delta design's error is **magnitude independent**; the snapshot design's
grows linearly with distance from the origin.

Against the swap-tolerance-class bound of 7.1e-6 m the snapshot design is 1.3x
over at 100 m and 140x over at 10 km. Clearing it comfortably needs the
position magnitude held under roughly 30 m, which at 5 m/s is a local-origin
re-anchor every six seconds, and a re-anchor rewrites every stored row: an
`O(k)` spike at 0.17 Hz, plus origin state, a per-tick distance test, and a
rule for a window that straddles a re-anchor. **That machinery does not
undercut the rebuild it deletes, and it reintroduces the periodic `O(k)` spike
that was already rejected.**

**Decision: keep the maintained product.** Say the honest counter-case with it,
because the snapshot design is not wrong: against the QUALIFICATION tolerances
that actually gate this vehicle -- 3.0e-2 m on a box corner, 1.3e-1 m on rms
navigation error -- even the unanchored 10 km case has thirty times margin. The
bar it fails is the maintained product's own consistency-check tolerance, not a
flight-accuracy requirement, and a vehicle that never leaves a 100 m box would
be well served by it.

What decides it for a library rather than for one vehicle is the structural
property: an estimator whose window transform degrades linearly with distance
from the origin is one that has to be told where it is allowed to fly. The
delta design has no such parameter, and that is worth the three group
operations a fusion tick that Section 14 costs.

`retireDelta` and its exactness test stay regardless. They are the theory both
designs rest on.

