within Estimation.StrapdownINS.ESKF;

function correctOpticalFlow
  "Fuse image angular rates with co-timed range uncertainty"
  input State predicted;
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
  input Boolean useSemiDirectBias = false;
  output State corrected;
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
  Real predictedVelocityBody[3];
  Real velocityCross[3, 3];
  Real flowProjection[2, 3];
  Real flowH[2, TangentLength];
  Real compensatedFlow_rad[2];
  Real predictedFlow_rad_s[2];
  Real residual[2];
  Real H[2, TangentLength];
  Real flowCovariance_rad2[2, 2];
  Real measurementCovariance[2, 2];
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
  rotationWorldBody := LieGroups.SO3.Quat.to_DCM(
    delayedQuaternion);
  predictedVelocityBody := transpose(rotationWorldBody)
    * delayedVelocity;
  compensatedFlow_rad := measurement.integratedLineOfSight_rad
    + measurement.integratedGyroscopeBodyFlu_rad[1:2];
  safeIntegrationTime_s := max(abs(measurement.integrationTime_s), 1.0e-9);
  safeGroundDistance_m := max(measurement.groundDistance_m,
    minimumGroundDistance_m);
  flowProjection := [0.0, -1.0, 0.0; 1.0, 0.0, 0.0];
  predictedFlow_rad_s := flowProjection * predictedVelocityBody
    / safeGroundDistance_m;
  residual := compensatedFlow_rad / safeIntegrationTime_s - predictedFlow_rad_s;
  velocityCross := LieGroups.SO3.Quat.wedge(predictedVelocityBody);
  flowH := flowProjection * cat(2, zeros(3, 3), identity(3),
    velocityCross, zeros(3, 6)) / safeGroundDistance_m;
  H := flowH * currentToDelayed;
  flowCovariance_rad2 := measurement.integratedLineOfSightCovariance_rad2
    + measurement.integratedGyroscopeCovariance_rad2[1:2, 1:2];
  measurementCovariance := flowCovariance_rad2 / safeIntegrationTime_s^2
    + max(measurement.groundDistanceVariance_m2, 0.0) / safeGroundDistance_m^2
      * transpose({predictedFlow_rad_s}) * {predictedFlow_rad_s};
  corrected := copyState(predicted);
  accepted := false;
  normalizedInnovationSquared := 0.0;
  if not measurementFinite then
    rejectionReason := CorrectionRejectedNotFinite;
  elseif measurementAge_s < -1.0e-6
      or measurementAge_s > maximumAidingDelay_s then
    rejectionReason := CorrectionRejectedTimestamp;
  elseif not covarianceUsable
      or measurement.integrationTime_s <= 0.0
      or measurement.groundDistance_m < minimumGroundDistance_m
      or measurement.groundDistanceVariance_m2 < 0.0
      or measurement.quality < minimumQuality then
    rejectionReason := CorrectionRejectedCovarianceUnusable;
  else
    (corrected, accepted, rejectionReason, normalizedInnovationSquared) :=
      correctLinear(predicted, residual, H, measurementCovariance,
        innovationGate, zeros(3), zeros(TangentLength, size(residual, 1)),
        false, useSemiDirectBias);
  end if;
end correctOpticalFlow;
