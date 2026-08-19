within Estimation.MocapExternalOdometryErrorState;
function poseMeasurementCovariance "Pose measurement covariance in residual ordering"
  input PoseMeasurementNoise measurementNoise;
  output MeasurementCovariance covariance;
algorithm
  covariance := zeros(MeasurementLength, MeasurementLength);
  for axis in 1:3 loop
    covariance[axis, axis] := measurementNoise.attitude;
    covariance[axis + 3, axis + 3] := measurementNoise.position;
  end for;
end poseMeasurementCovariance;
