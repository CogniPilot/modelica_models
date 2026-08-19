within Estimation.StrapdownINS;

package UKF
  "Manifold unscented Kalman filter for aided strapdown navigation"
  constant Integer TangentLength = 15;
  constant Integer SigmaCount = 31 "2*n + 1 state sigma points";
  constant Real SigmaScale = 3.872983346207417
    "sqrt(TangentLength), for alpha=1, beta=2, kappa=0";
  constant Real SigmaWeight = 0.03333333333333333
    "1/(2*TangentLength) for every noncentral sigma point";
  constant Real CentralCovarianceWeight = 2.0
    "w0 covariance weight for alpha=1, beta=2, kappa=0";

  type TangentVector = Real[15]
    "{position, velocity, attitude, gyro bias, accelerometer bias}";
  type Covariance = Real[15, 15];
  type NominalVector = Real[16]
    "{position(3),velocity(3),quaternion(4),gyro bias(3),accel bias(3)}";
end UKF;
