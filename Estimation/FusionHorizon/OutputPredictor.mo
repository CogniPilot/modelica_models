within Estimation.FusionHorizon;

block OutputPredictor
  "SE_2(3) delta ring, delayed fusion window, and output predictor"

  parameter Real samplePeriod(unit = "s", min = 1.0e-9) = 0.00125
    "800 Hz inertial tick, paced by the IMU data-ready interrupt";
  parameter Real fusionPeriod_s(unit = "s", min = 1.0e-9) = 0.01
    "100 Hz fusion release; one filter step per composed window";
  parameter Real fusionHorizon_s(unit = "s", min = 0.0) = 0.2
    "Lag of the fusion instant behind now. Chosen so every aiding source has
     already delivered by the time the filter reaches that instant: on the
     deployed stack GPS is the latest, at about 100 ms at the driver and about
     200 ms end to end. A measurement that arrives after the horizon has
     passed its epoch is late in exactly the way it is late today, and the
     filter rejects it on its own timestamp rule rather than transporting a
     Jacobian to meet it";
  parameter Real gravityWorldEnu_m_s2[3] = {0.0, 0.0, -9.81};
  parameter Boolean useFirstOrderHold = true
    "True composes each interval under a first-order hold with the coning,
     sculling, and scrolling cross terms; false holds the current sample";
  parameter Real initialGyroscopeBiasAnchorBodyFlu_rad_s[3] = zeros(3);
  parameter Real initialAccelerometerBiasAnchorBodyFlu_m_s2[3] = zeros(3);
  parameter Real initialPositionWorldEnu_m[3] = zeros(3);
  parameter Real initialVelocityWorldEnu_m_s[3] = zeros(3);
  parameter Real initialQuaternionWorldBody[4] = {1.0, 0.0, 0.0, 0.0};

  final parameter Integer deltasPerFusion(min = 1) =
    integer(fusionPeriod_s / samplePeriod + 0.5)
    "Inertial ticks per release; 8 at 800 Hz against a 100 Hz release";
  final parameter Integer horizonWindows(min = 0) =
    integer(fusionHorizon_s / fusionPeriod_s + 0.5)
    "Complete release windows that must stand between the fusion instant and
     now; 20 at a 200 ms horizon and a 100 Hz release";
  final parameter Integer bufferLength(min = 2) = horizonWindows + 2
    "Fixed ring capacity: the horizon, the window being accumulated, and one
     slot of slack so a release never has to race the store. Nothing here is
     allocated at run time -- the length is a parameter, the index arithmetic
     wraps at most once, and the buffer walks are fixed-length";

  input Boolean reset;
  input Real angularVelocityMeasuredBodyFlu_rad_s[3](each unit = "rad/s");
  input Real specificForceMeasuredBodyFlu_m_s2[3](each unit = "m/s2");
  // The estimator-facing inputs below are the ENTIRE filter side of this
  // component's interface: a pose, a bias, and one boolean. There is no
  // covariance, no tangent, no sigma point, and no injection.
  input Boolean horizonStateValid
    "The horizon pose and bias below are usable";
  input Boolean horizonStateShifted
    "The filter moved the horizon state on this release. A window slide alone
     must NOT set this: sliding the window with no correction does not move
     now, because the factor the filter consumed is exactly the factor the
     predictor already composed. Only a correction requires the re-base";
  input Real horizonPositionWorldEnu_m[3](each unit = "m");
  input Real horizonVelocityWorldEnu_m_s[3](each unit = "m/s");
  input Real horizonQuaternionWorldBody[4](each unit = "1");
  input Real horizonGyroscopeBiasBodyFlu_rad_s[3](each unit = "rad/s");
  input Real horizonAccelerometerBiasBodyFlu_m_s2[3](each unit = "m/s2");

  discrete output Real positionWorldEnu_m[3](
    each unit = "m", each start = 0.0, each fixed = true);
  discrete output Real velocityWorldEnu_m_s[3](
    each unit = "m/s", each start = 0.0, each fixed = true);
  discrete output Real quaternionWorldBody[4](
    each unit = "1", start = {1.0, 0.0, 0.0, 0.0}, each fixed = true);
  discrete output Real angularVelocityBodyFlu_rad_s[3](
    each unit = "rad/s", each start = 0.0, each fixed = true)
    "Bias-corrected gyroscope sample at the inertial rate. The rate loop's
     signal is strictly causal and carries no smoothing horizon; the filter
     contributes only the slowly varying bias, so the fusion lag delays the
     bias estimate and never the rate";
  discrete Avionics.ImuSampleOutput horizonPacket
    "The composed window handed to the filter at the fusion instant";
  discrete output Integer bufferedDeltaCount(start = 0, fixed = true)
    "Inertial ticks standing between the fusion instant and now";
  discrete output Boolean bufferOverflowed(start = false, fixed = true)
    "The ring exceeded the horizon it is sized for. Reported rather than
     silently absorbed: it means the fusion side stopped consuming";
  discrete output Boolean rebased(start = false, fixed = true)
    "This tick recomposed the predictor over the whole buffer";
  discrete output Boolean horizonReady(start = false, fixed = true)
    "The buffer spans the full horizon, so the fusion instant is real";

protected
  discrete Real ring[bufferLength, DeltaLength](
    each start = 0.0, each fixed = true);
  discrete Integer headSlot(
    start = 1, fixed = true, min = 1, max = bufferLength)
    "Slot carrying the window being accumulated";
  discrete Integer ringTail(start = 1, fixed = true);
  discrete Integer ringCount(start = 0, fixed = true);
  discrete Integer fusionCountdown(start = 0, fixed = true);
  discrete Real anchorGyroscopeBias_rad_s[3](
    each start = 0.0, each fixed = true);
  discrete Real anchorAccelerometerBias_m_s2[3](
    each start = 0.0, each fixed = true);
  discrete Real previousAngularVelocity_rad_s[3](
    each start = 0.0, each fixed = true);
  discrete Real previousSpecificForce_m_s2[3](
    each start = 0.0, each fixed = true);
  discrete Boolean seeded(start = false, fixed = true);
  discrete Real packetTimestamp_s(start = 0.0, fixed = true);
  discrete Real liveRow[DeltaLength](
    start = cat(1, zeros(6), {1.0}, zeros(49)), each fixed = true)
    "The window being accumulated, seeded at the group identity";
  discrete Real storedRow[DeltaLength](each start = 0.0, each fixed = true);
  discrete Integer storeSlot(start = 0, fixed = true);
  discrete Real releasedRow[DeltaLength](each start = 0.0, each fixed = true);
  discrete Real predictedVector[10](
    start = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
    each fixed = true);
  discrete Integer headSlotNext(
    start = 1, fixed = true, min = 1, max = bufferLength);
  discrete Boolean released(start = false, fixed = true);
  discrete Real releasedSpan_s(start = 0.0, fixed = true);
  discrete Real releasedDeltaAngle_rad[3](each start = 0.0, each fixed = true);

algorithm
  when sample(0.0, samplePeriod) then
    // The bias anchor is fixed at seeding and never moved. Every buffered
    // delta therefore shares one linearization point, so their Jacobians
    // compose exactly and one first-order move carries the whole window to
    // whatever bias the filter currently believes. Re-anchoring mid-buffer
    // would mix linearization points inside a single composed Jacobian, which
    // is the kind of error that is invisible in a closed-loop test.
    anchorGyroscopeBias_rad_s := if reset or not pre(seeded)
      then initialGyroscopeBiasAnchorBodyFlu_rad_s
      else pre(anchorGyroscopeBias_rad_s);
    anchorAccelerometerBias_m_s2 := if reset or not pre(seeded)
      then initialAccelerometerBiasAnchorBodyFlu_m_s2
      else pre(anchorAccelerometerBias_m_s2);
    // One pure function per tick, every history read written as pre(), and the
    // only thing the block itself does with the buffer is store the row the
    // function returned. Returning the whole ring instead would copy it.
    (liveRow,
     storedRow,
     storeSlot,
     releasedRow,
     predictedVector,
     headSlotNext,
     ringTail,
     ringCount,
     fusionCountdown,
     seeded,
     bufferOverflowed,
     horizonReady,
     rebased,
     released,
     bufferedDeltaCount,
     packetTimestamp_s) := Estimation.FusionHorizon.step(
      reset,
      angularVelocityMeasuredBodyFlu_rad_s,
      specificForceMeasuredBodyFlu_m_s2,
      pre(previousAngularVelocity_rad_s),
      pre(previousSpecificForce_m_s2),
      pre(ring),
      pre(liveRow),
      pre(headSlot),
      pre(ringTail),
      pre(ringCount),
      pre(fusionCountdown),
      pre(seeded),
      pre(bufferOverflowed),
      Estimation.FusionHorizon.Pose(
        positionWorldEnu_m=pre(positionWorldEnu_m),
        velocityWorldEnu_m_s=pre(velocityWorldEnu_m_s),
        quaternionWorldBody=pre(quaternionWorldBody)),
      pre(packetTimestamp_s),
      anchorGyroscopeBias_rad_s,
      anchorAccelerometerBias_m_s2,
      horizonStateValid,
      horizonStateShifted,
      Estimation.FusionHorizon.Pose(
        positionWorldEnu_m=if horizonStateValid
          then horizonPositionWorldEnu_m else initialPositionWorldEnu_m,
        velocityWorldEnu_m_s=if horizonStateValid
          then horizonVelocityWorldEnu_m_s else initialVelocityWorldEnu_m_s,
        quaternionWorldBody=if horizonStateValid
          then horizonQuaternionWorldBody else initialQuaternionWorldBody),
      horizonGyroscopeBiasBodyFlu_rad_s,
      horizonAccelerometerBiasBodyFlu_m_s2,
      samplePeriod,
      deltasPerFusion,
      horizonWindows,
      gravityWorldEnu_m_s2,
      useFirstOrderHold);
    // The store goes through a function because the code generator refuses a
    // dynamic array index at model level: it cannot prove the slot in range
    // there, and a ring buffer whose index it cannot bound is exactly what it
    // is right to refuse.
    ring := Estimation.FusionHorizon.storeRow(pre(ring), storeSlot, storedRow);
    headSlot := headSlotNext;
    // The packet is assembled field by field. Assigning a whole record to a
    // connector is not something OpenModelica generates code for -- it reports
    // a template error on the left-hand side, and where the connector is
    // aliased it silently publishes zeros instead -- which is why every
    // estimator in this library publishes its connector the same way.
    //
    // Avionics.ImuSample is the whole estimator-facing interface of the
    // horizon: an accumulated SE_2(3) delta, its span, the bias anchor it was
    // integrated at, and the five Jacobians needed to move it. It already
    // exists, it is already algorithm-neutral, and both shipped filters
    // already consume it. Presenting the horizon window in it is what lets a
    // filter be swapped without the buffer knowing.
    releasedSpan_s := max(releasedRow[11], 1.0e-9);
    releasedDeltaAngle_rad := LieGroups.SO3.Quat.log_map(releasedRow[7:10]);
    horizonPacket.valid := released;
    // Level-triggered delivery: the packet is held until replaced and the
    // filter makes consumption exactly-once by timestamp novelty, which is the
    // handshake the rest of the corpus already uses across clocks.
    horizonPacket.fresh := released;
    horizonPacket.timestamp_s := packetTimestamp_s;
    // The scalar rate fields are DERIVED from the composed increment rather
    // than sampled, so a consumer of the packet sees the motion the deltas
    // describe.
    horizonPacket.angularVelocityBodyFlu_rad_s :=
      releasedDeltaAngle_rad / releasedSpan_s;
    horizonPacket.specificForceBodyFlu_m_s2 :=
      releasedRow[4:6] / releasedSpan_s;
    horizonPacket.deltaAngleBodyFlu_rad := releasedDeltaAngle_rad;
    horizonPacket.deltaVelocityBodyFlu_m_s := releasedRow[4:6];
    horizonPacket.deltaPositionBodyFlu_m := releasedRow[1:3];
    horizonPacket.deltaQuaternionBodyFlu := releasedRow[7:10];
    horizonPacket.integrationTime_s := releasedRow[11];
    horizonPacket.gyroscopeBiasLinearizationBodyFlu_rad_s :=
      anchorGyroscopeBias_rad_s;
    horizonPacket.accelerometerBiasLinearizationBodyFlu_m_s2 :=
      anchorAccelerometerBias_m_s2;
    horizonPacket.deltaRotationGyroscopeBiasJacobian_s :=
      Estimation.FusionHorizon.jacobianBlock(releasedRow, 11);
    horizonPacket.deltaVelocityGyroscopeBiasJacobian_m :=
      Estimation.FusionHorizon.jacobianBlock(releasedRow, 20);
    horizonPacket.deltaVelocityAccelerometerBiasJacobian_s :=
      Estimation.FusionHorizon.jacobianBlock(releasedRow, 29);
    horizonPacket.deltaPositionGyroscopeBiasJacobian_m_s :=
      Estimation.FusionHorizon.jacobianBlock(releasedRow, 38);
    horizonPacket.deltaPositionAccelerometerBiasJacobian_s2 :=
      Estimation.FusionHorizon.jacobianBlock(releasedRow, 47);
    previousAngularVelocity_rad_s := angularVelocityMeasuredBodyFlu_rad_s;
    previousSpecificForce_m_s2 := specificForceMeasuredBodyFlu_m_s2;
    positionWorldEnu_m := predictedVector[1:3];
    velocityWorldEnu_m_s := predictedVector[4:6];
    quaternionWorldBody := predictedVector[7:10];
    angularVelocityBodyFlu_rad_s := angularVelocityMeasuredBodyFlu_rad_s
      - (if horizonStateValid then horizonGyroscopeBiasBodyFlu_rad_s
         else anchorGyroscopeBias_rad_s);
  end when;

  annotation(Documentation(info = "<html>
    <p>Owns the buffer side of a delayed-fusion architecture and nothing else.
    Per inertial tick it integrates one interval into an SE_2(3) right factor,
    accumulates it into the window being built, and composes it onto the
    predicted state. At each fusion release the completed window is adopted and
    the oldest one leaves the buffer as an <code>Avionics.ImuSample</code> for
    whatever filter is wired to it. When that filter reports it moved the
    horizon state, the predictor is recomposed over the whole buffer.</p>
    <p><b>The ring is release-granular, not tick-granular.</b> Composition is
    exact at any granularity (FOH paper Lemma 5), and the fusion instant only
    ever lands on a release, so one entry per window is the same group element
    as eight per window at an eighth of the storage traffic. That is not a
    micro-optimization: measured, a per-tick ring spent about nine tenths of
    every inertial tick moving the buffer around rather than integrating.</p>
    <p><b>Re-base is triggered by a correction, not by a window slide.</b> The
    fusion instant advances every release unconditionally, but an advance with
    no correction does not move now: the factor the filter consumed in its own
    prediction is exactly the factor the predictor already composed, so
    associativity leaves the answer unchanged. Only a nonzero state shift makes
    the buffered factors apply to a different pose.</p>
    <p><b>Startup.</b> A horizon that is real costs one horizon of startup.
    Until the ring spans <code>fusionHorizon_s</code> there is no fusion
    instant, <code>horizonReady</code> is false, and no packet is released. The
    predictor still runs, dead reckoning from the seed pose.</p>
    <p><b>Anti-aliasing</b> is assumed, not implemented: see the package
    documentation. The 800 Hz stream is taken to be the output of the sensor's
    on-die low-pass, not raw 32 kHz mechanical bandwidth.</p>
  </html>"));
end OutputPredictor;
