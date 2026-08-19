within Estimation.MocapExternalOdometryErrorState;
function poseMeasurementMatrix "Pose observation map in tangent coordinates"
  output MeasurementMatrix H;
algorithm
  H := zeros(MeasurementLength, TangentLength);
  H[1:3, 1:3] := identity(3);
  H[4:6, 7:9] := identity(3);
end poseMeasurementMatrix;
