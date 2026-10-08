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
  annotation(Documentation(info = "<html>
    <h4>Use</h4>
<p><a href=\"modelica://Estimation.StrapdownINS.UKF.Estimator\">Estimator</a>
implements the shared sampled strapdown interface. It carries the same nominal
position, velocity, quaternion and IMU biases as the ESKF, with 31 sigma points
in a 15-dimensional tangent. The quaternion is not treated as four independent
Euclidean error coordinates.</p>
<p>Use the same sensor captures, covariance, terrain assumptions and initial
uncertainty when comparing filters. More sigma points do not by themselves
establish better consistency or robustness. Inspect prediction/correction
acceptance and runtime cost along with RMS error.</p>
<p>The nominal vector has 16 entries; its local covariance has 15 by 15 entries.
Do not interchange those layouts. Shared frames and timing are documented in
<a href=\"modelica://Estimation.StrapdownINS.PartialEstimator\">PartialEstimator</a>.</p>
    </html>"));
end UKF;
