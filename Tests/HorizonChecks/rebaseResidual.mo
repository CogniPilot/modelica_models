within Tests.HorizonChecks;

function rebaseResidual
  "Worst |re-base by one fold - re-base by stepwise composition| after a shift"
  input Integer count(min = 1);
  input Real dt(unit = "s");
  input Real gravityWorldEnu_m_s2[3];
  input Real correction[9]
    "Right-injected tangent applied to the horizon pose, in the SE_2(3)
     ordering {position, velocity, rotation}";
  output Real worst[3] "position [m], velocity [m/s], attitude [rad]";
protected
  Real angularVelocity_rad_s[3];
  Real specificForce_m_s2[3];
  Real previousAngularVelocity_rad_s[3];
  Real previousSpecificForce_m_s2[3];
  Real ring[count, Estimation.FusionHorizon.DeltaLength];
  Estimation.FusionHorizon.Delta tick;
  Estimation.FusionHorizon.Pose horizonPose;
  Estimation.FusionHorizon.Pose correctedPose;
  Estimation.FusionHorizon.Pose foldedPose;
  Estimation.FusionHorizon.Pose steppedPose;
  Real extendedPose[10];
  Real correctedExtendedPose[10];
  Real attitudeError_rad[3];
algorithm
  // Theorem 6 (FOH paper Sec. VI): the left and right factors of the flow do
  // not depend on the state, so a corrected horizon pose reapplies the SAME
  // buffered factors. The test checks that folding the buffer once and then
  // composing is the same element as composing the pose through the buffer one
  // delta at a time, which is the associativity the re-base relies on, taken
  // from a pose the correction actually moved.
  previousAngularVelocity_rad_s := zeros(3);
  previousSpecificForce_m_s2 := zeros(3);
  for k in 1:count loop
    (angularVelocity_rad_s, specificForce_m_s2) :=
      Tests.HorizonChecks.syntheticImu(k, dt);
    if k == 1 then
      previousAngularVelocity_rad_s := angularVelocity_rad_s;
      previousSpecificForce_m_s2 := specificForce_m_s2;
    end if;
    tick := Estimation.FusionHorizon.integrateSample(
      angularVelocity_rad_s, specificForce_m_s2,
      previousAngularVelocity_rad_s, previousSpecificForce_m_s2,
      zeros(3), zeros(3), dt, true);
    ring[k, :] := Estimation.FusionHorizon.packDelta(tick);
    previousAngularVelocity_rad_s := angularVelocity_rad_s;
    previousSpecificForce_m_s2 := specificForce_m_s2;
  end for;

  horizonPose := Estimation.FusionHorizon.Pose(
    positionWorldEnu_m={3.0, -2.0, 11.0},
    velocityWorldEnu_m_s={0.7, 1.3, -0.4},
    quaternionWorldBody=LieGroups.SO3.Quat.exp_map({0.2, -0.35, 0.9}));
  // The shift is the same right injection the filters apply, written as the
  // group operation rather than through any filter's inject function: the
  // horizon must not care which filter produced it.
  extendedPose := cat(1, horizonPose.positionWorldEnu_m,
    horizonPose.velocityWorldEnu_m_s, horizonPose.quaternionWorldBody);
  correctedExtendedPose := LieGroups.SE23.Quat.product(
    extendedPose, LieGroups.SE23.Quat.exp_map(correction));
  correctedPose := Estimation.FusionHorizon.Pose(
    positionWorldEnu_m=correctedExtendedPose[1:3],
    velocityWorldEnu_m_s=correctedExtendedPose[4:6],
    quaternionWorldBody=LieGroups.SO3.Quat.normalize(
      correctedExtendedPose[7:10]));

  foldedPose := Estimation.FusionHorizon.composePose(
    correctedPose,
    Estimation.FusionHorizon.foldBuffer(ring, 1, count),
    gravityWorldEnu_m_s2);

  steppedPose := correctedPose;
  for k in 1:count loop
    steppedPose := Estimation.FusionHorizon.composePose(
      steppedPose,
      Estimation.FusionHorizon.unpackDelta(ring[k, :]),
      gravityWorldEnu_m_s2);
  end for;

  attitudeError_rad := LieGroups.SO3.Quat.log_map(
    LieGroups.SO3.Quat.product(
      LieGroups.SO3.Quat.inverse(steppedPose.quaternionWorldBody),
      foldedPose.quaternionWorldBody));
  worst := {
    max(abs(foldedPose.positionWorldEnu_m - steppedPose.positionWorldEnu_m)),
    max(abs(foldedPose.velocityWorldEnu_m_s
      - steppedPose.velocityWorldEnu_m_s)),
    sqrt(attitudeError_rad * attitudeError_rad)};
end rebaseResidual;
