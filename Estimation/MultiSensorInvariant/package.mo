within Estimation;

package MultiSensorInvariant
  "Bias-aware geometric error-state EKF with an SE_2(3) navigation state"
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
    locally linearized error transition is discretized with a third-order matrix
    exponential polynomial; the process-noise integral is evaluated by
    Simpson quadrature. Both operations retain full cross-axis covariance and
    use matrix expressions rather than three scalar filters.</p>
    <p>The additive bias states are not part of a group-affine, exactly
    log-linear augmented system. The invariant pose error and manifold
    retraction are deliberate geometric choices inside an otherwise ordinary
    error-state EKF.</p>
  </html>"));
end MultiSensorInvariant;
