within Estimation.StrapdownINS.UKF;

function correctGps "Unscented joint GPS position/velocity correction"
  input Estimation.StrapdownINS.UKF.State predicted;
  input Avionics.GpsSample measurement;
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
  Real sigmaMeasurement[6, SigmaCount];
  Real measured[6];
  Real covariance[6, 6];
algorithm
  nominal := stateVector(predicted);
  (sigma, sigmaUsable) := sigmaTangents(predicted.covariance);
  for index in 1:SigmaCount loop
    sigmaState := injectVector(nominal, sigma[:, index]);
    sigmaMeasurement[:, index] := cat(1,
      sigmaState[1:3], sigmaState[4:6]);
  end for;
  measured := cat(1, measurement.positionWorldEnu_m,
    measurement.velocityWorldEnu_m_s);
  covariance := cat(1,
    cat(2, measurement.positionCovarianceWorld_m2, zeros(3, 3)),
    cat(2, zeros(3, 3), measurement.velocityCovarianceWorld_m2_s2));
  (corrected, accepted, rejectionReason, normalizedInnovationSquared) :=
    correctUnscented(predicted, sigmaMeasurement, measured, covariance,
      innovationGate);
  accepted := accepted and sigmaUsable;
end correctGps;
