within Tests.HorizonChecks;

function compositionResidual
  "Worst |fold of per-tick deltas - one accumulating integration pass|"
  input Integer count(min = 1);
  input Real dt(unit = "s");
  input Boolean useFirstOrderHold;
  output Real worst[4]
    "position [m], velocity [m/s], attitude [rad], bias Jacobians [mixed]";
protected
  Real angularVelocity_rad_s[3];
  Real specificForce_m_s2[3];
  Real previousAngularVelocity_rad_s[3];
  Real previousSpecificForce_m_s2[3];
  Estimation.FusionHorizon.Delta folded;
  Estimation.FusionHorizon.Delta tick;
  Real accumulatedPosition_m[3];
  Real accumulatedVelocity_m_s[3];
  Real accumulatedQuaternion[4];
  Real accumulatedRotationJacobian_s[3, 3];
  Real accumulatedVelocityGyroJacobian_m[3, 3];
  Real accumulatedVelocityAccelJacobian_s[3, 3];
  Real accumulatedPositionGyroJacobian_m_s[3, 3];
  Real accumulatedPositionAccelJacobian_s2[3, 3];
  Real attitudeError_rad[3];
  Real worstJacobian;
algorithm
  // The composition property: adjacent right factors multiply, so N buffered
  // deltas composed and the same N samples integrated in one pass are the same
  // group element. FOH paper Lemma 5, Sec. IV-C.
  folded := Estimation.FusionHorizon.identityDelta();
  accumulatedPosition_m := zeros(3);
  accumulatedVelocity_m_s := zeros(3);
  accumulatedQuaternion := {1.0, 0.0, 0.0, 0.0};
  accumulatedRotationJacobian_s := zeros(3, 3);
  accumulatedVelocityGyroJacobian_m := zeros(3, 3);
  accumulatedVelocityAccelJacobian_s := zeros(3, 3);
  accumulatedPositionGyroJacobian_m_s := zeros(3, 3);
  accumulatedPositionAccelJacobian_s2 := zeros(3, 3);
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
      zeros(3), zeros(3), dt, useFirstOrderHold);
    folded := Estimation.FusionHorizon.composeDelta(folded, tick);
    (accumulatedPosition_m,
     accumulatedVelocity_m_s,
     accumulatedQuaternion,
     accumulatedRotationJacobian_s,
     accumulatedVelocityGyroJacobian_m,
     accumulatedVelocityAccelJacobian_s,
     accumulatedPositionGyroJacobian_m_s,
     accumulatedPositionAccelJacobian_s2) :=
      Estimation.StrapdownINS.preintegrateImuStep(
        accumulatedPosition_m,
        accumulatedVelocity_m_s,
        accumulatedQuaternion,
        accumulatedRotationJacobian_s,
        accumulatedVelocityGyroJacobian_m,
        accumulatedVelocityAccelJacobian_s,
        accumulatedPositionGyroJacobian_m_s,
        accumulatedPositionAccelJacobian_s2,
        angularVelocity_rad_s, specificForce_m_s2,
        zeros(3), zeros(3), dt, useFirstOrderHold,
        previousAngularVelocity_rad_s, previousSpecificForce_m_s2);
    previousAngularVelocity_rad_s := angularVelocity_rad_s;
    previousSpecificForce_m_s2 := specificForce_m_s2;
  end for;
  attitudeError_rad := LieGroups.SO3.Quat.log_map(
    LieGroups.SO3.Quat.product(
      LieGroups.SO3.Quat.inverse(accumulatedQuaternion),
      folded.deltaQuaternionBodyFlu));
  // The Jacobians are NOT expected to agree exactly. The accumulating pass
  // evaluates each interval's sensitivity at the midpoint of the running
  // preintegral, and the per-tick pass evaluates it at the midpoint of its own
  // interval, so the two linearize at different points. The disagreement is
  // second order in dt per step and the test asserts that scaling rather than
  // an exactness the mathematics does not claim.
  worstJacobian := max(abs(folded.deltaRotationGyroscopeBiasJacobian_s
    - accumulatedRotationJacobian_s));
  worstJacobian := max(worstJacobian,
    max(abs(folded.deltaVelocityGyroscopeBiasJacobian_m
      - accumulatedVelocityGyroJacobian_m)));
  worstJacobian := max(worstJacobian,
    max(abs(folded.deltaVelocityAccelerometerBiasJacobian_s
      - accumulatedVelocityAccelJacobian_s)));
  worstJacobian := max(worstJacobian,
    max(abs(folded.deltaPositionGyroscopeBiasJacobian_m_s
      - accumulatedPositionGyroJacobian_m_s)));
  worstJacobian := max(worstJacobian,
    max(abs(folded.deltaPositionAccelerometerBiasJacobian_s2
      - accumulatedPositionAccelJacobian_s2)));
  worst := {
    max(abs(folded.deltaPositionBodyFlu_m - accumulatedPosition_m)),
    max(abs(folded.deltaVelocityBodyFlu_m_s - accumulatedVelocity_m_s)),
    sqrt(attitudeError_rad * attitudeError_rad),
    worstJacobian};
end compositionResidual;
