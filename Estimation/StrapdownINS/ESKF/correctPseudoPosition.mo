within Estimation.StrapdownINS.ESKF;

function correctPseudoPosition
  "Fuse a synthetic measurement that the vehicle is still at its held position"
  input Estimation.StrapdownINS.ESKF.State predicted;
  input Real holdPositionWorldEnu_m[3]
    "Position the filter is held to: the last position it had while an
     anchor source was live, or the initialization position";
  input Real variance_m2(unit = "m2")
    "Per-axis measurement variance. Large by design, so the update only
     bounds the position and velocity error a tilt error would otherwise
     integrate without limit, exactly as the fake-position fusion of the
     PX4 and ArduPilot filters does";
  input Real innovationGate = 0.0;
  output Estimation.StrapdownINS.ESKF.State corrected;
  output Boolean accepted;
  output Integer rejectionReason;
  output Real normalizedInnovationSquared;
protected
  Real rotationWorldBody[3, 3];
  Real residual[3];
  Real H[3, TangentLength];
  Real measurementCovariance[3, 3];
algorithm
  // Same body-frame position residual convention as correctMocap and
  // correctGpsPosition, so the position rows of H are the identity.
  rotationWorldBody := LieGroups.SO3.Quat.to_DCM(
    predicted.quaternionWorldBody);
  residual := transpose(rotationWorldBody)
    * (holdPositionWorldEnu_m - predicted.positionWorldEnu_m);
  H := cat(2, identity(3), zeros(3, TangentLength - 3));
  measurementCovariance := identity(3) * variance_m2;
  (corrected, accepted, rejectionReason, normalizedInnovationSquared) :=
    correctLinear(predicted, residual, H, measurementCovariance,
      innovationGate);
end correctPseudoPosition;
