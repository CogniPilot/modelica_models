within Estimation.StrapdownINS.ESKF;

function correctOpticalFlow
  "Convert co-timed flow and range to body velocity, then correct the state"
  input Estimation.StrapdownINS.ESKF.State predicted;
  input Avionics.OpticalFlowSample measurement;
  input Real innovationGate = 0.0
    "Per-degree-of-freedom NIS gate; non-positive disables";
  input Real measurementAge_s(unit = "s") = 0.0;
  input Real angularVelocityMeasuredBodyFlu_rad_s[3] = zeros(3);
  input Real specificForceMeasuredBodyFlu_m_s2[3] = zeros(3);
  input Real gravityWorldEnu_m_s2[3] = {0.0, 0.0, -9.81};
  input Real maximumAidingDelay_s(unit = "s") = 0.25;
  input Real minimumQuality = 0.2;
  input Real minimumGroundDistance_m(unit = "m") = 0.2;
  output Estimation.StrapdownINS.ESKF.State corrected;
  output Boolean accepted;
  output Integer rejectionReason
    "Estimation.StrapdownINS.Correction* outcome code";
  output Real normalizedInnovationSquared;
protected
  Real rotationWorldBody[3, 3];
  Real delayedVelocity[3];
  Real delayedQuaternion[4];
  Real delayedStateVector[16];
  Real currentToDelayed[TangentLength, TangentLength];
  Boolean delayAccepted;
  Real predictedVelocityBody[3];
  Real velocityCross[3, 3];
  Real velocityH[2, TangentLength];
  Real delayedH[2, TangentLength];
  Real compensatedFlow_rad[2];
  Real measuredVelocityBody_m_s[2];
  Real residual[2];
  Real H[2, TangentLength];
  Real flowCovariance_rad2[2, 2];
  Real measurementCovariance[2, 2];
  Real velocityRangeDerivative_s[2];
  Real safeIntegrationTime_s;
  Real safeGroundDistance_m;
  Boolean measurementFinite;
  Boolean covarianceUsable;
algorithm
  measurementFinite := abs(measurement.timestamp_s) < FiniteMagnitudeLimit
    and abs(measurement.integrationTime_s) < FiniteMagnitudeLimit
    and abs(measurement.groundDistance_m) < FiniteMagnitudeLimit
    and abs(measurement.groundDistanceVariance_m2) < FiniteMagnitudeLimit
    and abs(measurement.quality) < FiniteMagnitudeLimit;
  covarianceUsable := measurement.groundDistanceVariance_m2 >= 0.0;
  for row in 1:2 loop
    measurementFinite := measurementFinite
      and abs(measurement.integratedLineOfSight_rad[row])
        < FiniteMagnitudeLimit
      and abs(measurement.integratedGyroscopeBodyFlu_rad[row])
        < FiniteMagnitudeLimit;
    covarianceUsable := covarianceUsable
      and measurement.integratedLineOfSightCovariance_rad2[row, row] > 0.0
      and measurement.integratedGyroscopeCovariance_rad2[row, row] > 0.0;
    for column in 1:2 loop
      covarianceUsable := covarianceUsable
        and abs(measurement.integratedLineOfSightCovariance_rad2[row, column])
          < FiniteMagnitudeLimit
        and abs(measurement.integratedGyroscopeCovariance_rad2[row, column])
          < FiniteMagnitudeLimit;
    end for;
  end for;
  measurementFinite := measurementFinite
    and abs(measurement.integratedGyroscopeBodyFlu_rad[3])
      < FiniteMagnitudeLimit;
  covarianceUsable := covarianceUsable
    and measurement.integratedGyroscopeCovariance_rad2[3, 3] > 0.0;
  delayedStateVector := retrodict(predicted,
    angularVelocityMeasuredBodyFlu_rad_s,
    specificForceMeasuredBodyFlu_m_s2, gravityWorldEnu_m_s2,
    max(measurementAge_s, 0.0));
  delayedVelocity := delayedStateVector[4:6];
  delayedQuaternion := delayedStateVector[7:10];
  currentToDelayed := discreteTransition(continuousTransition(
    angularVelocityMeasuredBodyFlu_rad_s
      - predicted.gyroscopeBiasBodyFlu_rad_s,
    specificForceMeasuredBodyFlu_m_s2
      - predicted.accelerometerBiasBodyFlu_m_s2),
    -max(measurementAge_s, 0.0));
  delayAccepted := true;
  rotationWorldBody := LieGroups.SO3.Quat.to_DCM(
    delayedQuaternion);
  predictedVelocityBody := transpose(rotationWorldBody)
    * delayedVelocity;
  // The sensor reports right-handed scene rotations about camera x/y and a
  // co-timed distance along the nadir ray. After gyro compensation, their
  // direct body-FLU velocity observation is
  //   v_forward = range * flow_y / dt,
  //   v_left    = -range * flow_x / dt.
  // This is the flight-stack interface: terrain remains a separately filtered
  // range state and is not needed to manufacture a velocity observation.
  compensatedFlow_rad := measurement.integratedLineOfSight_rad
    + measurement.integratedGyroscopeBodyFlu_rad[1:2];
  safeIntegrationTime_s := max(abs(measurement.integrationTime_s), 1.0e-9);
  safeGroundDistance_m := max(measurement.groundDistance_m,
    minimumGroundDistance_m);
  measuredVelocityBody_m_s := safeGroundDistance_m
    / safeIntegrationTime_s
      * {compensatedFlow_rad[2], -compensatedFlow_rad[1]};
  residual := measuredVelocityBody_m_s - predictedVelocityBody[1:2];
  velocityCross := LieGroups.SO3.Quat.wedge(predictedVelocityBody);
  velocityH := zeros(2, TangentLength);
  velocityH[1:2, 4:6] :=
    [1.0, 0.0, 0.0; 0.0, 1.0, 0.0];
  velocityH[1:2, 7:9] := velocityCross[1:2, :];
  delayedH := velocityH;
  H := delayedH * currentToDelayed;
  flowCovariance_rad2 := measurement.integratedLineOfSightCovariance_rad2
    + measurement.integratedGyroscopeCovariance_rad2[1:2, 1:2];
  measurementCovariance := (safeGroundDistance_m
      / safeIntegrationTime_s)^2 * [
    flowCovariance_rad2[2, 2], -flowCovariance_rad2[2, 1];
    -flowCovariance_rad2[1, 2], flowCovariance_rad2[1, 1]];
  velocityRangeDerivative_s := {
    compensatedFlow_rad[2] / safeIntegrationTime_s,
    -compensatedFlow_rad[1] / safeIntegrationTime_s};
  for row in 1:2 loop
    for column in 1:2 loop
      measurementCovariance[row, column] :=
        measurementCovariance[row, column]
        + max(measurement.groundDistanceVariance_m2, 0.0)
          * velocityRangeDerivative_s[row]
          * velocityRangeDerivative_s[column];
    end for;
  end for;
  if not measurementFinite then
    corrected := Estimation.StrapdownINS.ESKF.State(
      positionWorldEnu_m=predicted.positionWorldEnu_m,
      velocityWorldEnu_m_s=predicted.velocityWorldEnu_m_s,
      quaternionWorldBody=predicted.quaternionWorldBody,
      gyroscopeBiasBodyFlu_rad_s=predicted.gyroscopeBiasBodyFlu_rad_s,
      accelerometerBiasBodyFlu_m_s2=predicted.accelerometerBiasBodyFlu_m_s2,
      covariance=predicted.covariance);
    accepted := false;
    rejectionReason := CorrectionRejectedNotFinite;
    normalizedInnovationSquared := 0.0;
  elseif measurementAge_s < -1.0e-6
      or measurementAge_s > maximumAidingDelay_s then
    corrected := Estimation.StrapdownINS.ESKF.State(
      positionWorldEnu_m=predicted.positionWorldEnu_m,
      velocityWorldEnu_m_s=predicted.velocityWorldEnu_m_s,
      quaternionWorldBody=predicted.quaternionWorldBody,
      gyroscopeBiasBodyFlu_rad_s=predicted.gyroscopeBiasBodyFlu_rad_s,
      accelerometerBiasBodyFlu_m_s2=predicted.accelerometerBiasBodyFlu_m_s2,
      covariance=predicted.covariance);
    accepted := false;
    rejectionReason := CorrectionRejectedTimestamp;
    normalizedInnovationSquared := 0.0;
  elseif not delayAccepted or not covarianceUsable
      or measurement.integrationTime_s <= 0.0
      or measurement.groundDistance_m < minimumGroundDistance_m
      or measurement.groundDistanceVariance_m2 < 0.0
      or measurement.quality < minimumQuality then
    corrected := Estimation.StrapdownINS.ESKF.State(
      positionWorldEnu_m=predicted.positionWorldEnu_m,
      velocityWorldEnu_m_s=predicted.velocityWorldEnu_m_s,
      quaternionWorldBody=predicted.quaternionWorldBody,
      gyroscopeBiasBodyFlu_rad_s=predicted.gyroscopeBiasBodyFlu_rad_s,
      accelerometerBiasBodyFlu_m_s2=predicted.accelerometerBiasBodyFlu_m_s2,
      covariance=predicted.covariance);
    accepted := false;
    rejectionReason := CorrectionRejectedCovarianceUnusable;
    normalizedInnovationSquared := 0.0;
  else
    (corrected, accepted, rejectionReason, normalizedInnovationSquared) :=
      correctLinear(predicted, residual, H, measurementCovariance,
        innovationGate);
  end if;
end correctOpticalFlow;
