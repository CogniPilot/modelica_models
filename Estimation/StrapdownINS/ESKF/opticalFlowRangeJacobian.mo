within Estimation.StrapdownINS.ESKF;

function opticalFlowRangeJacobian
  "Nadir ray-to-plane range and Jacobian in the ESKF right-error tangent"
  input Estimation.StrapdownINS.ESKF.State predicted;
  input Real planeNormalWorldEnu[3] = {0.0, 0.0, 1.0};
  input Real planeOffset_m = 0.0;
  output Real predictedRange_m;
  output Real H[TangentLength];
protected
  Real rotationWorldBody[3, 3];
  Real boresightBody[3];
  Real denominator;
algorithm
  boresightBody := {0.0, 0.0, -1.0};
  rotationWorldBody := LieGroups.SO3.Quat.to_DCM(
    predicted.quaternionWorldBody);
  denominator := planeNormalWorldEnu
    * (rotationWorldBody * boresightBody);
  predictedRange_m := (planeOffset_m
    - planeNormalWorldEnu * predicted.positionWorldEnu_m) / denominator;
  H := zeros(TangentLength);
  H[1:3] := -(planeNormalWorldEnu * rotationWorldBody) / denominator;
  H[7:9] := predictedRange_m / denominator
    * planeNormalWorldEnu * rotationWorldBody
    * LieGroups.SO3.Quat.wedge(boresightBody);
end opticalFlowRangeJacobian;
