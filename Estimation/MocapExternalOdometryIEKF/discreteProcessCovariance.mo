within Estimation.MocapExternalOdometryIEKF;
function discreteProcessCovariance
  "Discrete tangent-space process covariance"
  input Real dt "Prediction interval [s]";
  input ProcessNoise processNoise;
  output Covariance covariance;
algorithm
  covariance := zeros(TangentLength, TangentLength);
  for axis in 1:3 loop
    covariance[axis, axis] := processNoise.attitude * dt;
    covariance[axis + 3, axis + 3] := processNoise.velocity * dt;
    covariance[axis + 6, axis + 6] := processNoise.position * dt;
    covariance[axis + 9, axis + 9] := processNoise.angularVelocity * dt;
  end for;
end discreteProcessCovariance;
