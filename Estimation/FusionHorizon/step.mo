within Estimation.FusionHorizon;

function step
  "One inertial tick of the buffered horizon: integrate, accumulate, release,
   predict"
  input Boolean reset;
  input Integer tickIndex(min = 0)
    "Inertial ticks completed since power-on. NOT cleared by a reset, which is
     the whole point of it: the packet epoch has to be re-anchored to a
     monotonic reference when the buffer is dropped, and this block has no
     runtime coordinate to read. A counter is that reference, it is carried
     like everything else here, and the code generator lowers it, which the
     clock it stands in for is not (`runtime-coordinate` is a refused GALEC
     projection feature).

     At 800 Hz a 32-bit counter wraps after about 31 days of continuous
     power-on, and single-precision tickIndex * dt stops being exact after
     about 5.8 hours. Both are properties the carried epoch already had.";
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
  input Boolean horizonReleased
    "A packet has already been handed over, so a fusion instant exists";
  input Integer ticksSinceRebase(min = 0)
    "Inertial ticks since the predictor was last recomposed from the horizon";
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
  input Integer horizonWindows(min = 1);
  input Real gravityWorldEnu_m_s2[3];
  input Boolean useFirstOrderHold;
  input Real maximumGyroscopeBiasMove_rad_s(min = 0.0);
  input Real maximumAccelerometerBiasMove_m_s2(min = 0.0);
  input Real maximumPredictorDivergence_rad(min = 0.0);
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
  output Integer nextTickIndex(min = 0);
  output Boolean nextHorizonReleased;
  output Integer nextTicksSinceRebase(min = 0);
  output Boolean horizonReady;
  output Boolean rebased;
  output Boolean released "A fusion window was handed over on this tick";
  output Boolean biasMoveExceeded
    "The filter's bias has moved further from the anchor than the first-order
     move is declared good for. Reported, never clamped: a silently clamped
     bias move is a wrong state published as a right one";
  output Integer bufferedDeltaCount "Inertial ticks spanning horizon to now";
  output Real nextPacketTimestamp_s(unit = "s");
protected
  Integer bufferLength;
  Boolean fusionBoundary;
  Integer adoptedCount;
  Integer epochRingCount;
  Boolean reanchor;
  Real gyroscopeBiasMoveMagnitude_rad_s;
  Real accelerometerBiasMoveMagnitude_m_s2;
  Real predictorDivergence_rad;
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
  elseif fusionBoundary and seeded then
    // The window just closed becomes a complete entry and the head moves on.
    nextRingCount := ringCount + 1;
    nextHeadSlot := if headSlot >= bufferLength then 1 else headSlot + 1;
    nextRingTail := ringTail;
  else
    nextRingTail := ringTail;
    nextRingCount := ringCount;
    nextHeadSlot := headSlot;
  end if;

  // ---- 3. release the oldest window to the filter -------------------------
  // A delayed horizon costs one horizon of start-up. Until the buffer spans it
  // there is no fusion instant to fuse at, and releasing early would stamp a
  // packet with an epoch the filter is not standing on.
  //
  // horizonWindows >= 1 is a precondition, asserted where the parameters are
  // declared. At zero the first boundary would release the slot this tick is
  // about to WRITE: the caller stores after the call, so that slot still holds
  // zeros, and a zero row is a zero span and a zero quaternion. The consumer
  // divides the rotation increment by the span, so the packet would carry a
  // not-a-number into the filter on the first release.
  adoptedCount := nextRingCount;
  released := adoptedCount > horizonWindows;
  // The first release is what creates a fusion instant. Before it the epoch is
  // still the seed value and nothing downstream may stand on it, so readiness
  // is latched on the release and not on the buffer merely being long enough.
  nextHorizonReleased := (not reset) and (horizonReleased or released);
  horizonReady := nextHorizonReleased;
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
  // The fusion instant advances by exactly one release window per release and
  // by nothing otherwise, so it is carried rather than read off a clock: the
  // code generator has no runtime coordinate, and a carried epoch says the
  // same thing with one addition.
  // The first tick closes the interval that ENDED at time zero, so the epoch
  // base is one tick before it. Getting this wrong shifts every fusion instant
  // by a sample and nothing downstream would notice.
  // A reset RE-ANCHORS the epoch to the tick it happened on. Seeding it at
  // -dt from a mid-flight reset would leave the packet epoch permanently
  // behind wall time by the whole flight so far, and a consumer that aligns
  // aiding by timestamp would then reject everything it was handed.
  nextPacketTimestamp_s := if reset or not seeded then tickIndex * dt - dt
    elseif released then packetTimestamp_s + dt * deltasPerFusion
    else packetTimestamp_s;
  bufferedDeltaCount := nextRingCount * deltasPerFusion
    + deltasPerFusion - nextFusionCountdown + 1;

  // ---- 4. predict now -----------------------------------------------------
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
  gyroscopeBiasMoveMagnitude_rad_s := sqrt(
    gyroscopeBiasMove_rad_s * gyroscopeBiasMove_rad_s);
  accelerometerBiasMoveMagnitude_m_s2 := sqrt(
    accelerometerBiasMove_m_s2 * accelerometerBiasMove_m_s2);
  // The first-order move is good over a stated ball around the anchor and no
  // further (FOH paper Prop. 8). Outside it the published state is still the
  // best available answer, so it is published and FLAGGED rather than clamped:
  // clamping would move the state to a bias nobody estimated and report
  // nothing.
  biasMoveExceeded :=
    gyroscopeBiasMoveMagnitude_rad_s > maximumGyroscopeBiasMove_rad_s
    or accelerometerBiasMoveMagnitude_m_s2 > maximumAccelerometerBiasMove_m_s2;
  // The incremental path composes tick factors integrated AT THE ANCHOR and
  // never moves them to the filter's bias; only a re-base does that, over the
  // whole buffered window at once. So between re-bases the predictor drifts
  // away from the filter's own bias at ||db_g||, without bound in the time
  // since the last re-base. Under sustained correction rejection that time is
  // the whole rejection episode. A re-anchor bounds it: when the accumulated
  // divergence would exceed the stated tolerance the fold runs anyway, at the
  // cost the WCET record already charges a re-base.
  predictorDivergence_rad :=
    gyroscopeBiasMoveMagnitude_rad_s * ticksSinceRebase * dt;
  reanchor := horizonStateValid
    and predictorDivergence_rad > maximumPredictorDivergence_rad;
  rebased := reset or not seeded
    or (horizonStateValid and horizonStateShifted) or reanchor;
  nextTicksSinceRebase := if rebased then 0 else ticksSinceRebase + 1;
  if rebased then
    // Theorem 6: the filter moved the left factor, and the buffered right
    // factors do not depend on it, so the state at now is recovered by
    // reapplying them to the corrected pose. No gain, no damping ratio, no
    // tracking error.
    //
    // THE EPOCH INVARIANT. The folded window and the pose it is composed onto
    // must name the SAME fusion instant. horizonPose is the filter state at
    // the instant reached by the PREVIOUS release: the caller reads it back
    // through pre(), one inertial tick after the filter published it. So the
    // window is folded over the ring as it stood BEFORE this tick's adopt and
    // BEFORE this tick's release -- ringTail and ringCount, not their advanced
    // successors -- and the window accumulated up to the previous tick rides
    // the fold as its trailing row, because on a boundary tick it is no longer
    // part of liveDelta and the ring passed in predates this tick's store.
    //
    // Folding the advanced tail instead is not a rounding error. On a release
    // boundary it drops the entry the pose has not yet absorbed and picks up
    // headSlot, which on this tick still holds the row from a whole ring ago,
    // or zeros before the ring has wrapped. A zero row is a zero quaternion,
    // and normalize(product(q, 0)) is the identity, so the composed rotation
    // collapses silently.
    //
    // Written this way the identity holds on EVERY tick without a case split:
    // off a boundary the trailing row plus tickDelta is exactly liveDelta; on
    // a boundary liveDelta is tickDelta alone and the trailing row is the
    // window the ring has not adopted yet from the pose's point of view.
    epochRingCount := if reset then 0 else ringCount;
    foldedWindow := Estimation.FusionHorizon.foldBuffer(
      ring, ringTail, epochRingCount,
      if reset then identityRow else previousLiveRow);
    windowDelta := Estimation.FusionHorizon.composeDelta(
      foldedWindow, tickDelta);
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
  // Seeded from the tick after power-on AND from the tick after a reset. It
  // used to be cleared by the reset itself, which left the block in its
  // first-tick state for one tick too many: the epoch was re-anchored twice,
  // once on the reset tick and once on the tick after it, so the epoch and
  // the buffer span disagreed by a sample from there on, and the interval
  // following a reset was integrated with a zero slope even though the reset
  // tick had already recorded a perfectly good previous sample.
  nextSeeded := true;
  nextTickIndex := tickIndex + 1;
end step;
