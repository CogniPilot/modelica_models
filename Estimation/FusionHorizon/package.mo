within Estimation;

package FusionHorizon
  "Estimator-agnostic delayed fusion horizon and SE_2(3) output predictor"
  constant Integer DeltaLength = 56
    "Flat storage width of one Estimation.FusionHorizon.Delta:
     3 position, 3 velocity, 4 quaternion, 1 span, and five 3x3 bias
     Jacobians stored row-major.";

  constant Real TimestampMagnitudeLimit = 1.0e30
    "A timestamp whose magnitude reaches this is not a time. The delayed
     measurement queues order themselves by timestamp, so a not-a-number or an
     unbounded one does not merely produce a wrong answer, it destroys the
     ordering the whole horizon rests on: every comparison against a
     not-a-number is false, so such a packet is never ripe, never late, and
     never leaves the queue. It is refused at arrival instead.

     The same magnitude the filters use, restated here rather than imported,
     because nothing in this package may depend on a filter.";

  constant Integer MocapMeasurementLength = 26
    "Flat storage width of one Avionics.MocapSample as a queue holds it:
     1 timestamp, 3 position, 4 quaternion, 9 position covariance, 9 attitude
     covariance. valid and fresh are NOT stored: a queued packet is by
     construction one that was valid when it arrived, and both flags are
     asserted on the tick it is delivered.";
  constant Integer GpsMeasurementLength = 30
    "1 timestamp, positionValid, velocityValid, 3 geodetic, 3 position,
     3 velocity, 9 position covariance, 9 velocity covariance. The two
     per-solution validity flags ARE stored, because unlike valid and fresh
     they say which half of the fix is usable, and that is a property of the
     measurement rather than of its delivery.";
  constant Integer MagnetometerMeasurementLength = 13
    "1 timestamp, 3 field, 9 covariance";
  constant Integer BarometerMeasurementLength = 3
    "1 timestamp, 1 altitude, 1 variance";
  constant Integer OpticalFlowMeasurementLength = 23
    "1 timestamp, 2 line of sight, 4 line-of-sight covariance, 3 gyroscope
     integral, 9 gyroscope covariance, 1 integration time, 1 ground distance,
     1 ground-distance variance, 1 quality";

  // ---- arrival outcomes, one per aiding source per inertial tick ----------
  // Every one of these is NAMED. Inferring what happened to a measurement from
  // the absence of a delivery is the predicate exhaustion the estimator status
  // boundary already refuses elsewhere: a packet that never arrived and a
  // packet refused for being older than the horizon are different failures,
  // and a supervisor has to be able to tell them apart.
  constant Integer AidingNoArrival = 0
    "No novel, valid, finitely stamped sample was presented on this tick";
  constant Integer AidingQueued = 1
    "Stored, to be fused when the fusion instant reaches its timestamp";
  constant Integer AidingRefusedLate = 2
    "Refused at arrival: the fusion instant had already passed this timestamp
     by more than the residual alignment covers, so there is no fusion instant
     left to fuse it at. This outcome is what replaces transporting a
     measurement Jacobian a quarter of a second backwards to meet it";
  constant Integer AidingRefusedOverflow = 3
    "Refused at arrival because the queue was full. The queue keeps what it
     already holds and the NEW measurement is the one that is lost.

     That is the opposite of the right answer for a live-edge buffer, and the
     reason it is right here is worth stating because the intuition runs the
     other way. In a delayed queue the OLDEST entry is the one closest to
     being fusable: it is the next one the fusion instant will reach. Dropping
     it to make room for a newer one throws away the entry that was about to
     be used and replaces it with one that cannot be used for another horizon.
     Do that on every arrival and a queue too short for its source never
     ripens ANY entry -- the contents are a sliding window of measurements
     that are all still in the filter's future -- so the source goes silent
     for the whole flight while every arrival is dutifully stored. Measured on
     an oversampled source before this was corrected: zero deliveries, and
     every assertion about ordering and residuals still passing, because a
     queue that delivers nothing violates none of them.

     Refusing the arrival cannot deadlock, because the delivery path always
     drains: on every release the oldest entry is either delivered or, if the
     fusion instant has passed it, discarded as stale. One slot therefore
     frees per release whatever the source does, and the queue degrades to
     delivering at the release rate rather than to delivering nothing";
  constant Integer AidingBeforeHorizon = 4
    "Presented before the first release, so there is no fusion instant for it
     to name yet. NOT a refusal and not counted as one: the horizon costs one
     horizon of start-up, during which the filter has had no inertial packet
     either and could not have fused anything. The sample is left unconsumed,
     so a source that holds its packet between pulses has it admitted on the
     first tick after the horizon becomes real";

  // ---- delivery outcomes -------------------------------------------------
  constant Integer AidingNoDelivery = 0
    "Nothing was ripe on this tick, or this tick released no window";
  constant Integer AidingDeliveredAtHorizon = 1
    "The oldest entry's timestamp has been reached by the fusion instant, and
     the entry was handed to the filter";
  constant Integer AidingDroppedStale = 2
    "The oldest entry was ripe but older than the residual alignment covers,
     so it was discarded rather than fused against a state that has already
     moved past it. Unreachable while every source samples no faster than the
     release rate; reported because the configuration that makes it reachable
     is not refused";

  annotation(Documentation(info = "<html>
    <p>The estimator fuses at a delayed horizon <code>t - D</code>, where every
    aiding measurement has already arrived, and the state control needs at
    <code>t</code> is recovered by composing the SE_2(3) preintegrals that were
    buffered while waiting. This package owns everything on the buffer side of
    that split: the fixed-size ring of per-tick deltas, the composition algebra,
    the output predictor, and the re-base that follows a horizon correction.</p>

    <p><b>Nothing here knows what a filter is.</b> There is no covariance, no
    sigma point, no error state, and no tangent ordering in this package. The
    estimator boundary is the one that already exists:
    <code>Avionics.ImuSample</code> carries the accumulated delta and its bias
    Jacobians into the filter, <code>Avionics.NavigationEstimate</code> plus the
    published gyroscope and accelerometer bias carry the corrected horizon state
    back out, and <code>Avionics.EstimatorStatus.acceptedCorrectionCount</code>
    is the signal that triggers a re-base. It is read as an EDGE: a change in
    the count is one accepted correction. The <code>correctionOutcome</code>
    level beside it is not usable for this, because it stands for a whole
    filter tick and would fire a re-base once per inertial tick of it. Any
    block extending <code>Estimation.StrapdownINS.PartialEstimator</code> plugs
    in unchanged; <code>HorizonEstimator</code> does exactly that through a
    <code>replaceable</code> slot, and the ESKF and the manifold UKF are both
    translated through it in <code>Tests.HorizonEstimatorWiring</code>. That
    gate is a translation gate, not a simulation: OpenModelica cannot build a
    simulation containing either composition, for a reason recorded in that
    model. The time-domain behaviour is exercised against a filter stand-in on
    the same declared boundary.</p>

    <p><b>Why the buffer is exact.</b> Under the mixed-invariant flow
    <code>Xdot = M X + X N(t)</code> with constant <code>M</code>, the flow over
    an interval factors as <code>X(T) = L X(0) R</code> with <code>L</code> and
    <code>R</code> independent of the initial state. A horizon correction
    therefore reapplies the same precomputed right factors exactly, and adjacent
    right factors multiply, so composing N buffered deltas and integrating the
    same N samples in one pass are the same group element. See the FOH paper,
    Lemma 3, Theorem 6 (Sec. VI, exact reapplication), and Lemma 5 (Sec. IV-C,
    error composition).</p>

    <p><b>Anti-aliasing is an assumption, not code.</b> The buffer integrates one
    sample per tick and claims first-order-hold accuracy for it. That claim holds
    only for a stream band-limited below half the sampling rate. It is, in
    silicon, before sampling: the deployed ICM-45686 runs its gyroscope at
    1600 Hz output data rate with the on-die low-pass at ODR/8 = 200 Hz and the
    accelerometer at ODR/16 = 100 Hz, so the raw 32 kHz mechanical bandwidth
    never reaches this code. No filter appears in this package because its input
    is defined to be the already-filtered stream. A change to the hardware filter
    configuration invalidates the hold-order error budget, and nothing in this
    model would detect it.</p>

    <p><b>Bias anchoring.</b> Every buffered delta is integrated at one anchor
    bias, fixed at initialization. The estimator supplies a bias estimate; the
    horizon moves the whole composed window to it through the accumulated
    Jacobians, which is the same first-order move
    <code>Estimation.StrapdownINS.correctPreintegratedImu</code> performs on a
    single packet. The remainder is second order and bounded by the horizon
    length rather than by mission length: at a 200 ms window and a 0.05 rad/s
    bias offset it is about 1e-4 rad. The horizon never asks the estimator for an
    error-state injection, only for a bias value, so the same path serves an
    additive-bias ESKF and a manifold UKF without either being privileged.</p>
  </html>"));
end FusionHorizon;
