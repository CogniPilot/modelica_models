within Estimation.StrapdownINS.UKF;

function correctOpticalFlow
  "Unscented planar body-velocity and ray-to-plane range correction"
  input Estimation.StrapdownINS.UKF.State predicted;
  input Avionics.OpticalFlowSample measurement;
  input Real innovationGate = 0.0;
  input Real groundNormalWorldEnu[3] = {0.0, 0.0, 1.0};
  input Real groundPlaneOffset_m = 0.0;
  output Estimation.StrapdownINS.UKF.State corrected;
  output Boolean accepted;
  output Integer rejectionReason;
  output Real normalizedInnovationSquared;
protected
  Real nominal[16];
  Real sigmaState[16];
  Real sigma[TangentLength, SigmaCount];
  Boolean sigmaUsable;
  Real rotationWorldBody[3, 3];
  Real velocityBody[3];
  Real sigmaMeasurement[3, SigmaCount];
  Real measured[3];
  Real measurementCovariance[3, 3];
algorithm
  nominal := stateVector(predicted);
  (sigma, sigmaUsable) := sigmaTangents(predicted.covariance);
  for index in 1:SigmaCount loop
    sigmaState := injectVector(nominal, sigma[:, index]);
    rotationWorldBody := LieGroups.SO3.Quat.to_DCM(
      sigmaState[7:10]);
    velocityBody := transpose(rotationWorldBody)
      * sigmaState[4:6];
    sigmaMeasurement[:, index] := cat(1, velocityBody[1:2],
      {Estimation.StrapdownINS.rangeToPlane(
        sigmaState[1:3], sigmaState[7:10],
        groundNormalWorldEnu, groundPlaneOffset_m)});
  end for;
  measured := cat(1, measurement.velocityBodyFlu_m_s,
    {measurement.groundDistance_m});
  measurementCovariance := zeros(3, 3);
  measurementCovariance[1:2, 1:2] :=
    measurement.velocityCovarianceBody_m2_s2;
  measurementCovariance[3, 3] := measurement.groundDistanceVariance_m2;
  (corrected, accepted, rejectionReason, normalizedInnovationSquared) :=
    correctUnscented(predicted, sigmaMeasurement,
      measured, measurementCovariance, innovationGate);
  accepted := accepted and sigmaUsable;
end correctOpticalFlow;
