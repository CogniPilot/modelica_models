within Estimation;

package MultiSensorInvariant
  "Bias-aware left-invariant EKF on SE_2(3) with multiple aiding sensors"
  constant Integer TangentLength = 15;
  constant Integer ProcessNoiseLength = 12;

  type TangentVector = Real[15]
    "{position,velocity,attitude,gyro bias,accelerometer bias} error";
  type Covariance = Real[15, 15];
  type ProcessNoiseCovariance = Real[12, 12]
    "{gyro,accelerometer,gyro-bias walk,accelerometer-bias walk}";

  annotation(Documentation(info = "<html>
    <p>The nominal extended pose is propagated by
    <code>LieGroups.SE23.Quat.exp_mixed</code>. Covariance lives in the
    left-invariant tangent error ordered as body-frame position, velocity,
    attitude, gyroscope bias, and accelerometer bias.</p>
    <p>Over one IMU interval the corrected IMU input is held constant. The
    continuous log-linear transition is discretized with a third-order matrix
    exponential polynomial; the process-noise integral is evaluated by
    Simpson quadrature. Both operations retain full cross-axis covariance and
    use matrix expressions rather than three scalar filters.</p>
  </html>"));
end MultiSensorInvariant;
