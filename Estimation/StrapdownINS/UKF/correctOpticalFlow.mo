within Estimation.StrapdownINS.UKF;

function correctOpticalFlow
  "Unscented body-velocity correction from co-timed flow and range"
  input Estimation.StrapdownINS.UKF.State predicted;
  input Avionics.OpticalFlowSample measurement;
  input Real innovationGate = 0.0;
  input Real measurementAge_s(unit = "s") = 0.0;
  input Real angularVelocityMeasuredBodyFlu_rad_s[3] = zeros(3);
  input Real specificForceMeasuredBodyFlu_m_s2[3] = zeros(3);
  input Real gravityWorldEnu_m_s2[3] = {0.0, 0.0, -9.81};
  input Real maximumAidingDelay_s(unit = "s") = 0.25;
  input Real minimumQuality = 0.2;
  input Real minimumGroundDistance_m(unit = "m") = 0.2;
  output Estimation.StrapdownINS.UKF.State corrected;
  output Boolean accepted;
  output Integer rejectionReason;
  output Real normalizedInnovationSquared;
protected
  Real nominal[16];
  Real sigmaState[16];
  Real delayedSigmaState[16];
  Real sigma[TangentLength, SigmaCount];
  Boolean sigmaUsable;
  Boolean geometryUsable;
  Real rotationWorldBody[3, 3];
  Real velocityBody[3];
  Real sigmaMeasurement[2, SigmaCount];
  Real compensatedFlow_rad[2];
  Real measured[2];
  Real flowCovariance_rad2[2, 2];
  Real measurementCovariance[2, 2];
  Real velocityRangeDerivative_s[2];
  Real safeIntegrationTime_s;
  Real safeGroundDistance_m;
  Boolean measurementFinite;
  Boolean covarianceUsable;
algorithm
  measurementFinite := abs(measurement.timestamp_s)
      < Estimation.StrapdownINS.ESKF.FiniteMagnitudeLimit
    and abs(measurement.integrationTime_s)
      < Estimation.StrapdownINS.ESKF.FiniteMagnitudeLimit
    and abs(measurement.groundDistance_m)
      < Estimation.StrapdownINS.ESKF.FiniteMagnitudeLimit
    and abs(measurement.groundDistanceVariance_m2)
      < Estimation.StrapdownINS.ESKF.FiniteMagnitudeLimit
    and abs(measurement.quality)
      < Estimation.StrapdownINS.ESKF.FiniteMagnitudeLimit;
  covarianceUsable := measurement.groundDistanceVariance_m2 >= 0.0;
  for row in 1:2 loop
    measurementFinite := measurementFinite
      and abs(measurement.integratedLineOfSight_rad[row])
        < Estimation.StrapdownINS.ESKF.FiniteMagnitudeLimit
      and abs(measurement.integratedGyroscopeBodyFlu_rad[row])
        < Estimation.StrapdownINS.ESKF.FiniteMagnitudeLimit;
    covarianceUsable := covarianceUsable
      and measurement.integratedLineOfSightCovariance_rad2[row, row] > 0.0
      and measurement.integratedGyroscopeCovariance_rad2[row, row] > 0.0;
    for column in 1:2 loop
      covarianceUsable := covarianceUsable
        and abs(measurement.integratedLineOfSightCovariance_rad2[row, column])
          < Estimation.StrapdownINS.ESKF.FiniteMagnitudeLimit
        and abs(measurement.integratedGyroscopeCovariance_rad2[row, column])
          < Estimation.StrapdownINS.ESKF.FiniteMagnitudeLimit;
    end for;
  end for;
  measurementFinite := measurementFinite
    and abs(measurement.integratedGyroscopeBodyFlu_rad[3])
      < Estimation.StrapdownINS.ESKF.FiniteMagnitudeLimit;
  covarianceUsable := covarianceUsable
    and measurement.integratedGyroscopeCovariance_rad2[3, 3] > 0.0;
  nominal := stateVector(predicted);
  (sigma, sigmaUsable) := sigmaTangents(predicted.covariance);
  geometryUsable := measurement.integrationTime_s > 0.0
    and measurement.groundDistance_m >= minimumGroundDistance_m
    and measurement.groundDistanceVariance_m2 >= 0.0;
  for index in 1:SigmaCount loop
    sigmaState := injectVector(nominal, sigma[:, index]);
    delayedSigmaState := predictNominalVector(sigmaState,
      angularVelocityMeasuredBodyFlu_rad_s,
      specificForceMeasuredBodyFlu_m_s2, gravityWorldEnu_m_s2,
      -max(measurementAge_s, 0.0));
    rotationWorldBody := LieGroups.SO3.Quat.to_DCM(
      delayedSigmaState[7:10]);
    velocityBody := transpose(rotationWorldBody)
      * delayedSigmaState[4:6];
    sigmaMeasurement[:, index] := velocityBody[1:2];
  end for;
  compensatedFlow_rad := measurement.integratedLineOfSight_rad
    + measurement.integratedGyroscopeBodyFlu_rad[1:2];
  safeIntegrationTime_s := max(abs(measurement.integrationTime_s), 1.0e-9);
  safeGroundDistance_m := max(measurement.groundDistance_m,
    minimumGroundDistance_m);
  measured := safeGroundDistance_m / safeIntegrationTime_s
    * {compensatedFlow_rad[2], -compensatedFlow_rad[1]};
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
    corrected := Estimation.StrapdownINS.UKF.State(
      positionWorldEnu_m=predicted.positionWorldEnu_m,
      velocityWorldEnu_m_s=predicted.velocityWorldEnu_m_s,
      quaternionWorldBody=predicted.quaternionWorldBody,
      gyroscopeBiasBodyFlu_rad_s=predicted.gyroscopeBiasBodyFlu_rad_s,
      accelerometerBiasBodyFlu_m_s2=predicted.accelerometerBiasBodyFlu_m_s2,
      covariance=predicted.covariance);
    accepted := false;
    rejectionReason := Estimation.StrapdownINS.CorrectionRejectedNotFinite;
    normalizedInnovationSquared := 0.0;
  elseif measurementAge_s < -1.0e-6
      or measurementAge_s > maximumAidingDelay_s then
    corrected := Estimation.StrapdownINS.UKF.State(
      positionWorldEnu_m=predicted.positionWorldEnu_m,
      velocityWorldEnu_m_s=predicted.velocityWorldEnu_m_s,
      quaternionWorldBody=predicted.quaternionWorldBody,
      gyroscopeBiasBodyFlu_rad_s=predicted.gyroscopeBiasBodyFlu_rad_s,
      accelerometerBiasBodyFlu_m_s2=predicted.accelerometerBiasBodyFlu_m_s2,
      covariance=predicted.covariance);
    accepted := false;
    rejectionReason := Estimation.StrapdownINS.CorrectionRejectedTimestamp;
    normalizedInnovationSquared := 0.0;
  elseif not sigmaUsable or not geometryUsable or not covarianceUsable
      or measurement.quality < minimumQuality then
    corrected := Estimation.StrapdownINS.UKF.State(
      positionWorldEnu_m=predicted.positionWorldEnu_m,
      velocityWorldEnu_m_s=predicted.velocityWorldEnu_m_s,
      quaternionWorldBody=predicted.quaternionWorldBody,
      gyroscopeBiasBodyFlu_rad_s=predicted.gyroscopeBiasBodyFlu_rad_s,
      accelerometerBiasBodyFlu_m_s2=predicted.accelerometerBiasBodyFlu_m_s2,
      covariance=predicted.covariance);
    accepted := false;
    rejectionReason :=
      Estimation.StrapdownINS.CorrectionRejectedCovarianceUnusable;
    normalizedInnovationSquared := 0.0;
  else
    (corrected, accepted, rejectionReason, normalizedInnovationSquared) :=
      correctUnscented(predicted, sigmaMeasurement,
        measured, measurementCovariance, innovationGate);
  end if;
end correctOpticalFlow;
