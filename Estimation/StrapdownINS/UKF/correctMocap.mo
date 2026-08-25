within Estimation.StrapdownINS.UKF;

function correctMocap "Unscented motion-capture pose correction"
  input Estimation.StrapdownINS.UKF.State predicted;
  input Avionics.MocapSample measurement;
  input Real innovationGate = 0.0;
  output Estimation.StrapdownINS.UKF.State corrected;
  output Boolean accepted;
  output Integer rejectionReason;
  output Real normalizedInnovationSquared;
protected
  Real nominal[16];
  Real sigmaState[16];
  Real sigma[TangentLength, SigmaCount];
  Boolean sigmaUsable;
  Real attitudeError[4];
  Real sigmaMeasurement[6, SigmaCount];
  Real measured[6];
  Real covariance[6, 6];
algorithm
  nominal := stateVector(predicted);
  (sigma, sigmaUsable) := sigmaTangents(predicted.covariance);
  for index in 1:SigmaCount loop
    sigmaState := injectVector(nominal, sigma[:, index]);
    attitudeError := LieGroups.SO3.Quat.product(
      LieGroups.SO3.Quat.inverse(predicted.quaternionWorldBody),
      sigmaState[7:10]);
    sigmaMeasurement[:, index] := cat(1,
      sigmaState[1:3],
      LieGroups.SO3.Quat.log_map(attitudeError));
  end for;
  attitudeError := LieGroups.SO3.Quat.product(
    LieGroups.SO3.Quat.inverse(predicted.quaternionWorldBody),
    LieGroups.SO3.Quat.normalize(measurement.quaternionWorldBody));
  measured := cat(1, measurement.positionWorldEnu_m,
    LieGroups.SO3.Quat.log_map(attitudeError));
  covariance := cat(1,
    cat(2, measurement.positionCovarianceWorld_m2, zeros(3, 3)),
    cat(2, zeros(3, 3), measurement.attitudeCovarianceBody_rad2));
  (corrected, accepted, rejectionReason, normalizedInnovationSquared) :=
    correctUnscented(predicted, sigmaMeasurement, measured, covariance,
      innovationGate);
  accepted := accepted and sigmaUsable;
end correctMocap;
