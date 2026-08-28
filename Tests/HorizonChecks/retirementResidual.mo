within Tests.HorizonChecks;

function retirementResidual
  "Worst |retireDelta(first, first (x) second) - second| over every block"
  input Integer firstCount(min = 1) "Ticks in the factor to be retired";
  input Integer secondCount(min = 1) "Ticks in the factor that must survive";
  input Real dt(unit = "s");
  output Real worst[4]
    "position [m], velocity [m/s], attitude [rad], worst Jacobian block";
protected
  Estimation.FusionHorizon.Delta first;
  Estimation.FusionHorizon.Delta second;
  Estimation.FusionHorizon.Delta composed;
  Estimation.FusionHorizon.Delta recovered;
  Estimation.FusionHorizon.Delta tick;
  Real angularVelocity_rad_s[3];
  Real specificForce_m_s2[3];
  Real previousAngularVelocity_rad_s[3];
  Real previousSpecificForce_m_s2[3];
  Real attitudeError_rad[3];
  Real jacobianWorst;
algorithm
  // A composed window divided by its own earliest factor must return the rest
  // of the window EXACTLY, not to first order. This measures that directly on
  // the same coning-rich and sculling-rich stream every other arm uses, so a
  // block whose retirement is only approximately right shows up here rather
  // than as slow drift in a maintained product nobody re-anchors.
  first := Estimation.FusionHorizon.identityDelta();
  second := Estimation.FusionHorizon.identityDelta();
  previousAngularVelocity_rad_s := zeros(3);
  previousSpecificForce_m_s2 := zeros(3);
  for k in 1:(firstCount + secondCount) loop
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
    if k <= firstCount then
      first := Estimation.FusionHorizon.composeDelta(first, tick);
    else
      second := Estimation.FusionHorizon.composeDelta(second, tick);
    end if;
    previousAngularVelocity_rad_s := angularVelocity_rad_s;
    previousSpecificForce_m_s2 := specificForce_m_s2;
  end for;
  composed := Estimation.FusionHorizon.composeDelta(first, second);
  recovered := Estimation.FusionHorizon.retireDelta(first, composed);
  attitudeError_rad := LieGroups.SO3.Quat.log_map(
    LieGroups.SO3.Quat.product(
      LieGroups.SO3.Quat.inverse(second.deltaQuaternionBodyFlu),
      recovered.deltaQuaternionBodyFlu));
  jacobianWorst := max({
    max(abs(recovered.deltaRotationGyroscopeBiasJacobian_s
      - second.deltaRotationGyroscopeBiasJacobian_s)),
    max(abs(recovered.deltaVelocityGyroscopeBiasJacobian_m
      - second.deltaVelocityGyroscopeBiasJacobian_m)),
    max(abs(recovered.deltaVelocityAccelerometerBiasJacobian_s
      - second.deltaVelocityAccelerometerBiasJacobian_s)),
    max(abs(recovered.deltaPositionGyroscopeBiasJacobian_m_s
      - second.deltaPositionGyroscopeBiasJacobian_m_s)),
    max(abs(recovered.deltaPositionAccelerometerBiasJacobian_s2
      - second.deltaPositionAccelerometerBiasJacobian_s2))});
  worst := {
    max(abs(recovered.deltaPositionBodyFlu_m - second.deltaPositionBodyFlu_m)),
    max(abs(recovered.deltaVelocityBodyFlu_m_s
      - second.deltaVelocityBodyFlu_m_s)),
    sqrt(attitudeError_rad * attitudeError_rad),
    jacobianWorst};
end retirementResidual;
