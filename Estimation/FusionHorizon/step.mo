within Estimation.FusionHorizon;

function step
  "One inertial tick of the buffered horizon: integrate, accumulate, release,
   predict"
  input Boolean reset;
  input Real angularVelocityMeasuredBodyFlu_rad_s[3];
  input Real specificForceMeasuredBodyFlu_m_s2[3];
  input Real previousAngularVelocityMeasuredBodyFlu_rad_s[3];
  input Real previousSpecificForceMeasuredBodyFlu_m_s2[3];
  input Real ring[:, DeltaLength]
    "The buffer as it stood at the previous tick. One entry per FUSION WINDOW,
     not per tick: composition is exact at any granularity (FOH paper Lemma 5),
     the fusion instant only ever lands on a release, and a per-tick ring costs
     eight times the storage traffic for the same group element.";
  input Real previousLiveRow[DeltaLength]
    "The window accumulated up to the previous tick. Carried as its own state
     rather than read back out of the ring: reading one entry through a fold
     costs a whole window of group products for one row.";
  input Integer headSlot(min = 1)
    "Slot the completed window is adopted into. Owned by the caller and derived
     only from state this tick has not written, because an index read back
     after a write in the same clause is not lowerable.";
  input Integer ringTail(min = 1);
  input Integer ringCount(min = 0) "Complete windows behind the live one";
  input Integer fusionCountdown(min = 0)
    "Ticks remaining before the next release. A countdown rather than a
     modulus: the code generator has no checked owner for a dynamic quotient,
     and a counter states the cadence in the words the scheduler uses.";
  input Boolean seeded;
  input Boolean bufferOverflowed;
  input Estimation.FusionHorizon.Pose predicted "The state at the previous now";
  input Real packetTimestamp_s(unit = "s");
  input Real gyroscopeBiasAnchorBodyFlu_rad_s[3];
  input Real accelerometerBiasAnchorBodyFlu_m_s2[3];
  input Boolean horizonStateValid;
  input Boolean horizonStateShifted;
  input Estimation.FusionHorizon.Pose horizonPose;
  input Real horizonGyroscopeBiasBodyFlu_rad_s[3];
  input Real horizonAccelerometerBiasBodyFlu_m_s2[3];
  input Real dt(unit = "s");
  input Integer deltasPerFusion(min = 1);
  input Integer horizonWindows(min = 0);
  input Real gravityWorldEnu_m_s2[3];
  input Boolean useFirstOrderHold;
  output Real liveRow[DeltaLength]
    "The window accumulated so far, carried by the caller as state";
  output Real storedRow[DeltaLength]
    "The completed window, to be stored at storeSlot";
  output Integer storeSlot
    "Slot the caller must store storedRow into. Zero on a tick that closes no
     window, which stores nothing without a branch";
  output Real releasedRow[DeltaLength]
    "The window handed to the filter, the identity when none is released";
  output Real predictedVector[10]
    "The state at now, as {position, velocity, quaternion}";
  output Integer nextHeadSlot(min = 1);
  output Integer nextRingTail(min = 1);
  output Integer nextRingCount(min = 0);
  output Integer nextFusionCountdown(min = 0);
  output Boolean nextSeeded;
  output Boolean nextBufferOverflowed;
  output Boolean horizonReady;
  output Boolean rebased;
  output Boolean released "A fusion window was handed over on this tick";
  output Integer bufferedDeltaCount "Inertial ticks spanning horizon to now";
  output Real nextPacketTimestamp_s(unit = "s");
protected
  Integer bufferLength;
  Boolean fusionBoundary;
  Integer adoptedCount;
  Estimation.FusionHorizon.Delta tickDelta;
  Estimation.FusionHorizon.Delta liveDelta;
  Estimation.FusionHorizon.Delta windowDelta;
  Estimation.FusionHorizon.Delta openingDelta;
  Estimation.FusionHorizon.Delta foldedWindow;
  Estimation.FusionHorizon.Delta movedWindow;
  Real identityRow[DeltaLength];
  Estimation.FusionHorizon.Pose predictedNext;
  Real gyroscopeBiasMove_rad_s[3];
  Real accelerometerBiasMove_m_s2[3];
algorithm
  // The whole tick is one pure function so the block stays a thin clocked
  // shell over discrete state, which is how every other estimator in this
  // library is built. It also keeps every buffer walk inside a function rather
  // than a when-body loop, which is where the code generator's limits on
  // conditional accumulation and data-dependent trip counts live.
  bufferLength := size(ring, 1);
  nextFusionCountdown := if reset or fusionCountdown <= 1 then deltasPerFusion
    else fusionCountdown - 1;
  fusionBoundary := nextFusionCountdown >= deltasPerFusion;

  // ---- 1. integrate this interval, into the live window -------------------
  // The sample closing this interval opened it on the previous tick; after a
  // reset the hold starts from a zero slope. Theorem 1 (Sec. III-D): the
  // trapezoid means plus one Lie bracket, whose three components are the
  // classical coning, sculling, and scrolling corrections.
  tickDelta := Estimation.FusionHorizon.integrateSample(
    angularVelocityMeasuredBodyFlu_rad_s,
    specificForceMeasuredBodyFlu_m_s2,
    if reset or not seeded then angularVelocityMeasuredBodyFlu_rad_s
      else previousAngularVelocityMeasuredBodyFlu_rad_s,
    if reset or not seeded then specificForceMeasuredBodyFlu_m_s2
      else previousSpecificForceMeasuredBodyFlu_m_s2,
    gyroscopeBiasAnchorBodyFlu_rad_s,
    accelerometerBiasAnchorBodyFlu_m_s2,
    dt,
    useFirstOrderHold);
  // The head slot always carries the window accumulated so far. At a release
  // it is adopted as a complete window and the accumulation restarts; between
  // releases it is rewritten in place. The buffer therefore always spans
  // exactly horizon to now with no separate live term to remember.
  // A release boundary closes the window that was being accumulated and opens
  // a new one on this tick. The closed window is what the ring adopts.
  // Every record-returning call is bound to a local before it is used, here
  // and in foldBuffer, for the reason recorded there: written as an argument
  // it is re-evaluated once per component of the record it returns.
  openingDelta := if fusionBoundary or reset
    then Estimation.FusionHorizon.identityDelta()
    else Estimation.FusionHorizon.unpackDelta(previousLiveRow);
  liveDelta := Estimation.FusionHorizon.composeDelta(openingDelta, tickDelta);
  liveRow := Estimation.FusionHorizon.packDelta(liveDelta);
  // The very first tick has no completed window behind it. Adopting one there
  // would put an empty entry at the head of the buffer and advance the fusion
  // instant by a window that covers no time, which is exactly the kind of
  // off-by-one that makes an epoch look right and be wrong.
  storedRow := previousLiveRow;
  storeSlot := if fusionBoundary and not reset and seeded then headSlot else 0;

  // ---- 2. adopt the completed window and advance the head -----------------
  if reset then
    nextRingTail := headSlot;
    nextRingCount := 0;
    nextHeadSlot := headSlot;
    nextBufferOverflowed := false;
  elseif fusionBoundary and seeded then
    // The window just closed becomes a complete entry and the head moves on.
    nextRingCount := ringCount + 1;
    nextHeadSlot := if headSlot >= bufferLength then 1 else headSlot + 1;
    nextRingTail := ringTail;
    nextBufferOverflowed := bufferOverflowed;
  else
    nextRingTail := ringTail;
    nextRingCount := ringCount;
    nextHeadSlot := headSlot;
    nextBufferOverflowed := bufferOverflowed;
  end if;

  // ---- 3. release the oldest window to the filter -------------------------
  // A delayed horizon costs one horizon of start-up. Until the buffer spans it
  // there is no fusion instant to fuse at, and releasing early would stamp a
  // packet with an epoch the filter is not standing on.
  adoptedCount := nextRingCount;
  released := adoptedCount > horizonWindows;
  horizonReady := adoptedCount >= horizonWindows;
  // Reading the oldest entry is row selection, not composition. A tick that
  // releases nothing still produces a row, and it is the group identity rather
  // than zeros: a zero quaternion has no logarithm, and handing one to a
  // consumer that takes its log is how a not-a-number leaves a block that
  // never computed one.
  identityRow := Estimation.FusionHorizon.packDelta(
    Estimation.FusionHorizon.identityDelta());
  releasedRow := Estimation.FusionHorizon.readRow(
    ring, if released then nextRingTail else 0)
    + (if released then 0.0 else 1.0) * identityRow;
  if released then
    nextRingTail := if nextRingTail >= bufferLength then 1
      else nextRingTail + 1;
    nextRingCount := adoptedCount - 1;
  end if;
  if nextRingCount > horizonWindows then
    // Capacity is a hard bound. Growing the buffer is not available on a
    // flight controller, and pretending the horizon is still full depth would
    // be worse than saying it is not.
    nextBufferOverflowed := true;
  end if;
  // The fusion instant advances by exactly one release window per release and
  // by nothing otherwise, so it is carried rather than read off a clock: the
  // code generator has no runtime coordinate, and a carried epoch says the
  // same thing with one addition.
  // The first tick closes the interval that ENDED at time zero, so the epoch
  // base is one tick before it. Getting this wrong shifts every fusion instant
  // by a sample and nothing downstream would notice.
  nextPacketTimestamp_s := if reset or not seeded then -dt
    elseif released then packetTimestamp_s + dt * deltasPerFusion
    else packetTimestamp_s;
  bufferedDeltaCount := nextRingCount * deltasPerFusion
    + deltasPerFusion - nextFusionCountdown + 1;

  // ---- 4. predict now -----------------------------------------------------
  rebased := reset or not seeded or (horizonStateValid and horizonStateShifted);
  // The estimator supplies a bias VALUE; the horizon computes the move from
  // its own anchor and applies it through the Jacobians it accumulated. No
  // filter-internal quantity crosses this line.
  gyroscopeBiasMove_rad_s := (if horizonStateValid
    then horizonGyroscopeBiasBodyFlu_rad_s
    else gyroscopeBiasAnchorBodyFlu_rad_s) - gyroscopeBiasAnchorBodyFlu_rad_s;
  accelerometerBiasMove_m_s2 := (if horizonStateValid
    then horizonAccelerometerBiasBodyFlu_m_s2
    else accelerometerBiasAnchorBodyFlu_m_s2)
    - accelerometerBiasAnchorBodyFlu_m_s2;
  if rebased then
    // Theorem 6: the filter moved the left factor, and the buffered right
    // factors do not depend on it, so the state at now is recovered by
    // reapplying them to the corrected pose. No gain, no damping ratio, no
    // tracking error. The buffer passed in predates this tick's store, so the
    // live window is composed on explicitly.
    foldedWindow := Estimation.FusionHorizon.foldBuffer(
      ring, nextRingTail, nextRingCount);
    windowDelta := Estimation.FusionHorizon.composeDelta(
      foldedWindow, liveDelta);
    movedWindow := Estimation.FusionHorizon.rebiasDelta(
      windowDelta, gyroscopeBiasMove_rad_s, accelerometerBiasMove_m_s2);
    predictedNext := Estimation.FusionHorizon.composePose(
      horizonPose, movedWindow, gravityWorldEnu_m_s2);
  else
    // Nothing moved the horizon, so composing the newest factor onto the
    // previous answer is the same element by associativity. This is the common
    // case and it is one group composition per tick.
    predictedNext := Estimation.FusionHorizon.composePose(
      predicted, tickDelta, gravityWorldEnu_m_s2);
  end if;
  predictedVector := cat(1, predictedNext.positionWorldEnu_m,
    predictedNext.velocityWorldEnu_m_s, predictedNext.quaternionWorldBody);
  nextSeeded := not reset;
end step;
