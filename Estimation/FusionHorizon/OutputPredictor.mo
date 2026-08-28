within Estimation.FusionHorizon;

block OutputPredictor
  "SE_2(3) delta ring, delayed fusion window, and output predictor"

  parameter Real samplePeriod(unit = "s", min = 1.0e-9) = 0.00125
    "800 Hz inertial tick, paced by the IMU data-ready interrupt.
    Structural: the release cadence and the buffer lengths derived from it are
    array dimensions and loop bounds, which carry their value when the code is
    generated rather than at run time. Evaluate = true says so; the
    alternative is a dynamically sized ring, which is what the code generator
    is right to refuse and what a flight timing record cannot be written
    against."
    annotation(Evaluate = true);
  parameter Real fusionPeriod_s(unit = "s", min = 1.0e-9) = 0.01
    "100 Hz fusion release; one filter step per composed window. Structural,
     for the reason recorded on samplePeriod"
    annotation(Evaluate = true);
  parameter Real fusionHorizon_s(unit = "s", min = 1.0e-9) = 0.2
    "Lag of the fusion instant behind now. Chosen so every aiding source has
     already delivered by the time the filter reaches that instant: on the
     deployed stack GPS is the latest, at about 100 ms at the driver and about
     200 ms end to end. A measurement that arrives after the horizon has
     passed its epoch is late in exactly the way it is late today, and the
     filter rejects it on its own timestamp rule rather than transporting a
     Jacobian to meet it.

     At least one fusionPeriod_s, and an exact multiple of it. Both are
     asserted below rather than rounded quietly. Structural, for the reason
     recorded on samplePeriod: the ring length is derived from it"
    annotation(Evaluate = true);
  parameter Real gravityWorldEnu_m_s2[3] = {0.0, 0.0, -9.81};
  parameter Boolean useFirstOrderHold = true
    "True composes each interval under a first-order hold with the coning,
     sculling, and scrolling cross terms; false holds the current sample";
  parameter Real maximumGyroscopeBiasMove_rad_s(unit = "rad/s", min = 0.0) =
    0.05
    "Largest gyroscope-bias move from the anchor the first-order Jacobian move
     is declared good for. The Prop. 8 remainder is (T_D ||db_g||)^2, about
     1e-4 rad at the flight horizon and this bound. Exceeding it raises
     biasMoveExceeded and changes nothing else: the state is published as
     computed rather than clamped to a bias nobody estimated";
  parameter Real maximumAccelerometerBiasMove_m_s2(unit = "m/s2", min = 0.0) =
    0.5 "The same bound for the accelerometer bias";
  parameter Real maximumPredictorDivergence_rad(unit = "rad", min = 0.0) =
    0.025
    "Attitude divergence between the predictor and the filter's own bias that
     is allowed to accumulate on the incremental path before a re-base is
     forced. The incremental path composes factors integrated at the ANCHOR
     bias, so the divergence grows at ||db_g|| in the time since the last
     re-base and is unbounded under sustained correction rejection. Forcing
     the fold bounds it; the cost is the re-base cost the WCET record already
     charges.

     It is NOT a free choice. Together with maximumGyroscopeBiasMove_rad_s it
     fixes the worst-case rate at which this block asks for a fold, and the
     target can only afford so many. The two are related by the assertion
     below, and 0.025 rad is what the declared bias-move bound and the
     measured fold budget leave. It used to be 1e-3, which is the right number
     for the bias moves a healthy filter produces and is fifty folds a second
     at the bias move this block declares it will tolerate: seven times the
     whole budget, in a configuration the parameters said was legal";
  parameter Real foldBudget_hz(unit = "1/s", min = 0.0) = 7.3
    "Buffer folds per second the flight target can afford, from
     docs/delayed-fusion-horizon-wcet.md: 600 MHz against the 82 million
     instructions one re-base costs as generated today. A property of the code
     generator, not of the architecture, and the first number to change when
     that is fixed";
  parameter Real correctionRateBudget_hz(unit = "1/s", min = 0.0) = 5.0
    "The share of foldBudget_hz reserved for accepted corrections, which are
     the folds the design exists to perform. Five per second covers a 5 Hz GPS
     fix rate. What is left is what the re-anchor may spend";
  parameter Real initialGyroscopeBiasAnchorBodyFlu_rad_s[3] = zeros(3);
  parameter Real initialAccelerometerBiasAnchorBodyFlu_m_s2[3] = zeros(3);
  parameter Real initialPositionWorldEnu_m[3] = zeros(3);
  parameter Real initialVelocityWorldEnu_m_s[3] = zeros(3);
  parameter Real initialQuaternionWorldBody[4] = {1.0, 0.0, 0.0, 0.0};

  final parameter Integer deltasPerFusion(min = 1) =
    integer(fusionPeriod_s / samplePeriod + 0.5)
    "Inertial ticks per release; 8 at 800 Hz against a 100 Hz release. The
     rounding is safe only because the ratio is asserted integral below";
  final parameter Integer horizonWindows(min = 1) =
    integer(fusionHorizon_s / fusionPeriod_s + 0.5)
    "Complete release windows that must stand between the fusion instant and
     now; 20 at a 200 ms horizon and a 100 Hz release";
  final parameter Real worstReanchorRate_hz(unit = "1/s") =
    maximumGyroscopeBiasMove_rad_s / maximumPredictorDivergence_rad
    "Folds per second the re-anchor asks for at the largest bias move this
     block declares it will tolerate. The divergence grows at the size of the
     bias move and the fold is forced when it reaches the tolerance, so the
     worst case is the ratio of the two";
  final parameter Integer bufferLength(min = 3) = horizonWindows + 2
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
    "The filter moved the horizon state on this release, EDGE triggered: true
     for exactly one inertial tick per newly accepted correction. A window
     slide alone must NOT set this, because sliding with no correction does
     not move now -- the factor the filter consumed is exactly the factor the
     predictor already composed. Neither may a held level: the filter's
     outcome field stands for every inertial tick of a filter step, so a level
     would re-base once per tick of a 100 Hz step and claim eight corrections
     where one happened";
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
  discrete output Boolean biasMoveExceeded(start = false, fixed = true)
    "The filter's bias has moved further from the horizon's anchor than the
     first-order move is declared good for";
  discrete output Boolean rebased(start = false, fixed = true)
    "This tick recomposed the predictor over the whole buffer";
  discrete output Boolean horizonReady(start = false, fixed = true)
    "A packet has been released, so the fusion instant is real and the epoch
     the packet carries can be stood on";

protected
  discrete Real ring[bufferLength, DeltaLength](
    each start = 0.0, each fixed = true);
  discrete Integer headSlot(
    start = 1, fixed = true, min = 1, max = bufferLength)
    "Slot carrying the window being accumulated";
  discrete Integer ringTail(start = 1, fixed = true);
  discrete Integer ringCount(start = 0, fixed = true);
  discrete Integer fusionCountdown(start = 0, fixed = true);
  discrete Integer ticksSinceRebase(start = 0, fixed = true);
  discrete Integer tickIndex(start = 0, fixed = true)
    "Inertial ticks completed since power-on, never cleared by a reset. It is
     the monotonic reference the packet epoch is re-anchored to when a reset
     drops the buffer: this block has no runtime coordinate to read, and the
     code generator refuses `time` in production code.";
  discrete Real anchorGyroscopeBias_rad_s[3](
    each start = 0.0, each fixed = true);
  discrete Real anchorAccelerometerBias_m_s2[3](
    each start = 0.0, each fixed = true);
  discrete Real previousAngularVelocity_rad_s[3](
    each start = 0.0, each fixed = true);
  discrete Real previousSpecificForce_m_s2[3](
    each start = 0.0, each fixed = true);
  discrete Boolean seeded(start = false, fixed = true);
  discrete Boolean horizonReleased(start = false, fixed = true);
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
    //
    // A carried tick counter is passed rather than a clock, and the function
    // reads it only where a reset has to re-anchor the packet epoch. Every
    // other tick is wall-clock free, which is what lets two instances handed
    // the same boundary values agree exactly, and it is also what the code
    // generator will lower: `time` in production code is a refused GALEC
    // projection feature.
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
     tickIndex,
     horizonReleased,
     ticksSinceRebase,
     horizonReady,
     rebased,
     released,
     biasMoveExceeded,
     bufferedDeltaCount,
     packetTimestamp_s) := Estimation.FusionHorizon.step(
      reset,
      pre(tickIndex),
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
      pre(horizonReleased),
      pre(ticksSinceRebase),
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
      useFirstOrderHold,
      maximumGyroscopeBiasMove_rad_s,
      maximumAccelerometerBiasMove_m_s2,
      maximumPredictorDivergence_rad);
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
    // PULSED delivery, on the release tick and no other. valid and fresh are
    // the same boolean here on purpose: a window either was handed over on
    // this tick or was not, and there is no held packet to distinguish the two
    // cases. The consumer is the filter, whose clock IS the release clock, so
    // it samples the packet on the tick it is published; anything wired to a
    // different clock must latch the packet itself rather than assume it is
    // still standing.
    //
    // The span guard is the second half of the zero-horizon defence. The
    // parameter assertions below make a zero-span release unreachable; if a
    // future change reaches it anyway, the packet goes out NOT valid instead
    // of carrying a zero span into a consumer that divides by it.
    horizonPacket.valid := released and releasedRow[11] > 0.0;
    horizonPacket.fresh := released and releasedRow[11] > 0.0;
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

equation
  // The rate lattice is a precondition, not a preference, and it used to be
  // rounded silently: deltasPerFusion and horizonWindows are formed with a
  // +0.5 rounding, so a fusionPeriod_s of 0.009 would have quietly become
  // eight ticks and every epoch the block published would have been wrong by
  // a tenth of a window with nothing reporting it. Assert the ratios instead.
  assert(abs(fusionPeriod_s - deltasPerFusion * samplePeriod)
      <= 1.0e-9 * fusionPeriod_s,
    "fusionPeriod_s must be an exact integer multiple of samplePeriod: the
     release cadence is counted in inertial ticks, so a fractional ratio makes
     every published packet epoch wrong by the remainder");
  assert(abs(fusionHorizon_s - horizonWindows * fusionPeriod_s)
      <= 1.0e-9 * fusionHorizon_s,
    "fusionHorizon_s must be an exact integer multiple of fusionPeriod_s: the
     buffer is counted in whole release windows, so a fractional ratio makes
     the fusion instant differ from the declared horizon by the remainder");
  // The supervision parameters and the timing record have to agree, and they
  // did not: at the declared bias-move bound the re-anchor asked for fifty
  // folds a second against a measured budget of 7.3, in a configuration
  // nothing refused. A bound that cannot be honoured is not a bound.
  //
  //     maximumGyroscopeBiasMove_rad_s / maximumPredictorDivergence_rad
  //       <= foldBudget_hz - correctionRateBudget_hz
  //
  // The left side is the worst-case re-anchor rate, the right side is what the
  // fold budget has left after the corrections the design exists to serve.
  assert(correctionRateBudget_hz < foldBudget_hz,
    "correctionRateBudget_hz must leave something under foldBudget_hz for the
     re-anchor, or the horizon has no way to bound the drift the incremental
     path does not carry");
  assert(worstReanchorRate_hz <= foldBudget_hz - correctionRateBudget_hz,
    "The supervision parameters ask for more buffer folds than the timing
     record says the target can run:
     maximumGyroscopeBiasMove_rad_s / maximumPredictorDivergence_rad must be at
     most foldBudget_hz - correctionRateBudget_hz. Widen the divergence
     tolerance, narrow the bias-move bound the block claims to tolerate, or
     raise the fold budget once the code generator stops materializing a
     record-valued call once per component");

  assert(horizonWindows >= 1,
    "fusionHorizon_s must be at least one fusionPeriod_s: at zero windows the
     first release would hand over the ring slot this tick has not written
     yet, which is a zero span and a zero quaternion, and the consumer divides
     the rotation increment by that span");

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
    <p><b>Re-base is triggered by an accepted correction, not by a window
    slide, and on the EDGE of one.</b> The fusion instant advances every
    release unconditionally, but an advance with no correction does not move
    now: the factor the filter consumed in its own prediction is exactly the
    factor the predictor already composed, so associativity leaves the answer
    unchanged. Only a nonzero state shift makes the buffered factors apply to a
    different pose, and one correction is one re-base:
    <code>horizonStateShifted</code> is a one-tick pulse, not the filter's
    outcome level held across every inertial tick of a filter step.</p>
    <p><b>The epoch invariant.</b> The window the re-base folds and the pose it
    composes onto name the same fusion instant. The pose arrives one inertial
    tick after the filter published it, so the fold runs over the ring as it
    stood before this tick's adopt and release, and the partly accumulated
    window rides that fold as its trailing row. See <code>step.mo</code>,
    which states the invariant where it is enforced.</p>
    <p><b>Startup.</b> A horizon that is real costs one horizon of startup.
    <code>horizonReady</code> is latched by the FIRST RELEASE, not by the ring
    merely being long enough: for one release window the ring already spans
    <code>fusionHorizon_s</code> and no packet has been handed over, so the
    epoch is still its seed value and nothing may stand on it. The predictor
    runs throughout, dead reckoning from the seed pose.</p>
    <p><b>Anti-aliasing</b> is assumed, not implemented: see the package
    documentation. The 800 Hz stream is taken to be the output of the sensor's
    on-die low-pass, not raw 32 kHz mechanical bandwidth.</p>
  </html>"));
end OutputPredictor;
