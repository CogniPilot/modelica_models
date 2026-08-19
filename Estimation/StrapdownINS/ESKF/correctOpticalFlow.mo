within Estimation.StrapdownINS.ESKF;

function correctOpticalFlow
  "Correct planar body velocity and nadir ray-to-plane range"
  input Estimation.StrapdownINS.ESKF.State predicted;
  input Avionics.OpticalFlowSample measurement;
  input Real innovationGate = 0.0
    "Per-degree-of-freedom NIS gate; non-positive disables";
  input Real groundNormalWorldEnu[3] = {0.0, 0.0, 1.0};
  input Real groundPlaneOffset_m = 0.0;
  output Estimation.StrapdownINS.ESKF.State corrected;
  output Boolean accepted;
  output Integer rejectionReason
    "Estimation.StrapdownINS.Correction* outcome code";
  output Real normalizedInnovationSquared;
protected
  Real rotationWorldBody[3, 3];
  Real predictedVelocityBody[3];
  Real velocityCross[3, 3];
  Real predictedRange_m;
  Real rangeH[TangentLength];
  Real residual[3];
  Real H[3, TangentLength];
  Real measurementCovariance[3, 3];
algorithm
  rotationWorldBody := LieGroups.SO3.Quat.to_DCM(
    predicted.quaternionWorldBody);
  predictedVelocityBody := transpose(rotationWorldBody)
    * predicted.velocityWorldEnu_m_s;
  (predictedRange_m, rangeH) := opticalFlowRangeJacobian(
    predicted, groundNormalWorldEnu, groundPlaneOffset_m);
  residual := cat(1,
    measurement.velocityBodyFlu_m_s - predictedVelocityBody[1:2],
    {measurement.groundDistance_m - predictedRange_m});
  velocityCross := LieGroups.SO3.Quat.wedge(predictedVelocityBody);
  H := zeros(3, TangentLength);
  H[1:2, 4:6] := [1.0, 0.0, 0.0; 0.0, 1.0, 0.0];
  H[1:2, 7:9] := velocityCross[1:2, :];
  H[3, :] := rangeH;
  measurementCovariance := zeros(3, 3);
  measurementCovariance[1:2, 1:2] :=
    measurement.velocityCovarianceBody_m2_s2;
  measurementCovariance[3, 3] := measurement.groundDistanceVariance_m2;
  (corrected, accepted, rejectionReason, normalizedInnovationSquared) := correctLinear(
    predicted, residual, H, measurementCovariance,
    innovationGate);
end correctOpticalFlow;
