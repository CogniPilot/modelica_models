within Tests.HorizonChecks;

function runHorizonArm
  "Drive Estimation.FusionHorizon.step over a whole run and keep its poses"
  input Integer count(min = 1) "Inertial ticks to run";
  input Real dt(unit = "s");
  input Integer deltasPerFusion(min = 1);
  input Integer horizonWindows(min = 1);
  input Real gravityWorldEnu_m_s2[3];
  input Boolean rebaseEveryTick
    "False dead reckons, so every tick after the first takes the incremental
     path. True hands the EXACT pose the reference stood on at the fusion
     instant back in on every ready tick, so every tick takes the re-base
     path with nothing to shift.";
  input Real referencePose[count + 1, 10] = zeros(count + 1, 10)
    "Row j is the reference pose at the end of tick j-1; row 1 is the seed";
  input Integer referenceBufferedCount[count] = zeros(count);
  input Boolean referenceReady[count] = fill(false, count);
  output Real poseHistory[count + 1, 10]
    "Row j is this arm's pose at the end of tick j-1";
  output Integer bufferedCount[count];
  output Boolean ready[count];
protected
  Integer bufferLength;
  Real ring[horizonWindows + 2, Estimation.FusionHorizon.DeltaLength];
  Real liveRow[Estimation.FusionHorizon.DeltaLength];
  Real storedRow[Estimation.FusionHorizon.DeltaLength];
  Real releasedRow[Estimation.FusionHorizon.DeltaLength];
  Real predictedVector[10];
  Integer headSlot;
  Integer headSlotNext;
  Integer ringTail;
  Integer ringCount;
  Integer fusionCountdown;
  Integer storeSlot;
  Integer ticksSinceRebase;
  Integer tickIndex;
  Integer bufferedDeltaCount;
  Integer epochIndex;
  Boolean seeded;
  Boolean horizonReleased;
  Boolean horizonReadyOut;
  Boolean rebased;
  Boolean released;
  Boolean biasMoveExceeded;
  Boolean shiftNow;
  Real packetTimestamp_s;
  Real angularVelocity_rad_s[3];
  Real specificForce_m_s2[3];
  Real previousAngularVelocity_rad_s[3];
  Real previousSpecificForce_m_s2[3];
  Real epochPose[10];
  Estimation.FusionHorizon.Pose predicted;
  Estimation.FusionHorizon.Pose horizonPose;
algorithm
  // The clocked lattice of Estimation.FusionHorizon.OutputPredictor written
  // out as a loop: the same state, carried the same way, calling the same pure
  // function. Running it here rather than only inside a simulated block is
  // what lets an identity be asserted on the RING and the STATE MACHINE
  // without an event scheduler in the way, and it is a check on step.mo
  // itself rather than on an algebraic helper step.mo happens to call.
  bufferLength := horizonWindows + 2;
  ring := zeros(bufferLength, Estimation.FusionHorizon.DeltaLength);
  liveRow := Estimation.FusionHorizon.packDelta(
    Estimation.FusionHorizon.identityDelta());
  headSlot := 1;
  ringTail := 1;
  ringCount := 0;
  fusionCountdown := 0;
  ticksSinceRebase := 0;
  tickIndex := 0;
  seeded := false;
  horizonReleased := false;
  packetTimestamp_s := 0.0;
  predicted := Estimation.FusionHorizon.Pose(
    positionWorldEnu_m=zeros(3),
    velocityWorldEnu_m_s=zeros(3),
    quaternionWorldBody={1.0, 0.0, 0.0, 0.0});
  poseHistory[1, :] := {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
  previousAngularVelocity_rad_s := zeros(3);
  previousSpecificForce_m_s2 := zeros(3);
  for k in 1:count loop
    (angularVelocity_rad_s, specificForce_m_s2) :=
      Tests.HorizonChecks.syntheticImu(k, dt);
    shiftNow := rebaseEveryTick and k >= 2 and referenceReady[max(1, k - 1)];
    // THE EPOCH INDEX, written out because it is the invariant under test.
    // The pose a tick is handed belongs to the fusion instant reached by the
    // PREVIOUS release, and the count published on a tick already describes
    // the epoch after that tick's own release. So the age of the pose is the
    // previous tick's count plus one, and the row that holds it is
    // k - referenceBufferedCount[k-1].
    epochIndex := if shiftNow
      then max(1, min(count + 1, k - referenceBufferedCount[max(1, k - 1)]))
      else 1;
    // Off a shift the pose is unused, but it still has to be a POSE: the block
    // substitutes its initial pose when horizonStateValid is false, and a row
    // of zeros is a zero quaternion, which normalizes to the identity and
    // silently rewrites the first tick.
    epochPose := if shiftNow then referencePose[epochIndex, :]
      else {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    horizonPose := Estimation.FusionHorizon.Pose(
      positionWorldEnu_m=epochPose[1:3],
      velocityWorldEnu_m_s=epochPose[4:6],
      quaternionWorldBody=epochPose[7:10]);
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
     horizonReadyOut,
     rebased,
     released,
     biasMoveExceeded,
     bufferedDeltaCount,
     packetTimestamp_s) := Estimation.FusionHorizon.step(
      false,
      tickIndex,
      angularVelocity_rad_s,
      specificForce_m_s2,
      previousAngularVelocity_rad_s,
      previousSpecificForce_m_s2,
      ring,
      liveRow,
      headSlot,
      ringTail,
      ringCount,
      fusionCountdown,
      seeded,
      horizonReleased,
      ticksSinceRebase,
      predicted,
      packetTimestamp_s,
      zeros(3),
      zeros(3),
      shiftNow,
      shiftNow,
      horizonPose,
      zeros(3),
      zeros(3),
      dt,
      deltasPerFusion,
      horizonWindows,
      gravityWorldEnu_m_s2,
      true,
      1.0,
      1.0,
      1.0);
    ring := Estimation.FusionHorizon.storeRow(ring, storeSlot, storedRow);
    headSlot := headSlotNext;
    predicted := Estimation.FusionHorizon.Pose(
      positionWorldEnu_m=predictedVector[1:3],
      velocityWorldEnu_m_s=predictedVector[4:6],
      quaternionWorldBody=predictedVector[7:10]);
    previousAngularVelocity_rad_s := angularVelocity_rad_s;
    previousSpecificForce_m_s2 := specificForce_m_s2;
    poseHistory[k + 1, :] := predictedVector;
    bufferedCount[k] := bufferedDeltaCount;
    ready[k] := horizonReadyOut;
  end for;
end runHorizonArm;
