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
  constant Integer AidingDeliveredOnArrival = 5
    "Delivered directly at an empty queue's current release";
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
    "The oldest queued entry exceeded the residual alignment bound";

  annotation(Documentation(info = "<html>
    <p>Delayed aiding queues, preintegrated IMU increments and current-time
    SE_2(3) prediction. Start with
    <a href=\"modelica://Estimation.FusionHorizon.HorizonEstimator\">HorizonEstimator</a>;
    its replaceable filter implements
    <a href=\"modelica://Estimation.StrapdownINS.PartialEstimator\">PartialEstimator</a>.</p>
    <p>Aiding packets wait until their capture timestamps are reached by the
    delayed filter. The output predictor reapplies buffered increments and
    first-order bias-Jacobian corrections to carry that state to the present.
    Buffered increments use a fixed bias anchor; corrections beyond the
    configured range raise <code>biasMoveExceeded</code>.</p>
    <p>Choose the fusion interval as an integral number of IMU intervals and
    the horizon as an integral number of fusion intervals. The horizon must
    cover the maximum source delay plus jitter margin. These values size fixed
    arrays and are set at translation. Supply an appropriately band-limited
    IMU stream; this package does not perform sensor anti-alias filtering.</p>
    <p>Monitor readiness, late/overflow refusals and the delivered residual age.
    Budget buffer folds using generated-code measurements on the intended
    target. Host replay timings do not establish target WCET.</p>
    </html>"));
end FusionHorizon;
