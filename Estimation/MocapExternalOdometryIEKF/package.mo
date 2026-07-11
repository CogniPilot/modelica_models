within Estimation;
package MocapExternalOdometryIEKF
  constant Integer TangentLength = 12
    "Tangent covariance dimension";
  constant Integer MeasurementLength = 6
    "Pose measurement tangent dimension";

  type Quaternion = Real[4] "Scalar-first Hamilton quaternion {w,x,y,z}";
  type Vector3 = Real[3] "Three-dimensional coordinate vector";
  type Covariance = Real[12, 12]
    "Full tangent covariance; entry units follow the documented tangent ordering";
  type TangentVector = Real[12];
  type MeasurementVector = Real[6];
  type MeasurementMatrix = Real[6, 12];
  type MeasurementCovariance = Real[6, 6];
  type CrossCovariance = Real[12, 6];
  type Gain = Real[12, 6];
end MocapExternalOdometryIEKF;
