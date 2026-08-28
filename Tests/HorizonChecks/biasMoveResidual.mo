within Tests.HorizonChecks;

function biasMoveResidual
  "Worst |Jacobian bias move - re-integration at the new bias| over a window"
  input Integer count(min = 1);
  input Real dt(unit = "s");
  input Real gyroscopeBiasDelta_rad_s[3];
  input Real accelerometerBiasDelta_m_s2[3];
  output Real worst[3] "position [m], velocity [m/s], attitude [rad]";
protected
  Real angularVelocity_rad_s[3];
  Real specificForce_m_s2[3];
  Real previousAngularVelocity_rad_s[3];
  Real previousSpecificForce_m_s2[3];
  Estimation.FusionHorizon.Delta anchored;
  Estimation.FusionHorizon.Delta reintegrated;
  Estimation.FusionHorizon.Delta moved;
  Estimation.FusionHorizon.Delta tick;
  Real attitudeError_rad[3];
algorithm
  // The horizon never re-integrates when the filter's bias estimate moves; it
  // applies one first-order move through the Jacobians it accumulated. The
  // remainder is second order and bounded by the window length, not by mission
  // time (FOH paper Proposition 8, Sec. VI-A). This driver measures that
  // remainder directly by integrating the same stream twice, once at the
  // anchor and once at the moved bias, and comparing the second to the
  // Jacobian move of the first.
  anchored := Estimation.FusionHorizon.identityDelta();
  reintegrated := Estimation.FusionHorizon.identityDelta();
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
    anchored := Estimation.FusionHorizon.composeDelta(anchored, tick);
    tick := Estimation.FusionHorizon.integrateSample(
      angularVelocity_rad_s, specificForce_m_s2,
      previousAngularVelocity_rad_s, previousSpecificForce_m_s2,
      gyroscopeBiasDelta_rad_s, accelerometerBiasDelta_m_s2, dt, true);
    reintegrated := Estimation.FusionHorizon.composeDelta(reintegrated, tick);
    previousAngularVelocity_rad_s := angularVelocity_rad_s;
    previousSpecificForce_m_s2 := specificForce_m_s2;
  end for;
  moved := Estimation.FusionHorizon.rebiasDelta(
    anchored, gyroscopeBiasDelta_rad_s, accelerometerBiasDelta_m_s2);
  attitudeError_rad := LieGroups.SO3.Quat.log_map(
    LieGroups.SO3.Quat.product(
      LieGroups.SO3.Quat.inverse(reintegrated.deltaQuaternionBodyFlu),
      moved.deltaQuaternionBodyFlu));
  worst := {
    max(abs(moved.deltaPositionBodyFlu_m
      - reintegrated.deltaPositionBodyFlu_m)),
    max(abs(moved.deltaVelocityBodyFlu_m_s
      - reintegrated.deltaVelocityBodyFlu_m_s)),
    sqrt(attitudeError_rad * attitudeError_rad)};
end biasMoveResidual;
