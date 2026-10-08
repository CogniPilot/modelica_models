within Estimation.StrapdownINS;

function correctPreintegratedImu
  "Move a stored IMU preintegral from its bias anchor to an estimated bias"
  // First-order bias-anchor correction: the preintegral was accumulated at a
  // linearization bias and is transported to the current bias estimate by the
  // Jacobians accumulated alongside it, rather than by reintegrating the
  // interval.  Forster, Carlone, Dellaert and Scaramuzza, "On-Manifold
  // Preintegration for Real-Time Visual-Inertial Odometry," IEEE T-RO
  // 33(1):1-21, 2017, doi:10.1109/TRO.2016.2597321, Section VII-B;
  // predecessor Lupton and Sukkarieh, IEEE T-RO 28(1):61-76, 2012.  The
  // Jacobians themselves come from Estimation.StrapdownINS.preintegrateImuStep
  // and differentiate the retained held or FOH exponent through the mixed
  // exponential and composition. They are exact derivatives of that nominal
  // approximation, not of the continuous physical trajectory. References:
  // Estimation.StrapdownINS.
  input Avionics.ImuSample imu;
  input Real gyroscopeBiasBodyFlu_rad_s[3];
  input Real accelerometerBiasBodyFlu_m_s2[3];
  output Real deltaPositionBodyFlu_m[3];
  output Real deltaVelocityBodyFlu_m_s[3];
  output Real deltaQuaternionBodyFlu[4];
protected
  Real gyroscopeBiasDelta_rad_s[3];
  Real accelerometerBiasDelta_m_s2[3];
  Real deltaRotationCorrection_rad[3];
algorithm
  gyroscopeBiasDelta_rad_s := gyroscopeBiasBodyFlu_rad_s
    - imu.gyroscopeBiasLinearizationBodyFlu_rad_s;
  accelerometerBiasDelta_m_s2 := accelerometerBiasBodyFlu_m_s2
    - imu.accelerometerBiasLinearizationBodyFlu_m_s2;
  deltaRotationCorrection_rad :=
    imu.deltaRotationGyroscopeBiasJacobian_s
      * gyroscopeBiasDelta_rad_s;
  deltaQuaternionBodyFlu := LieGroups.SO3.Quat.normalize(
    LieGroups.SO3.Quat.product(
      imu.deltaQuaternionBodyFlu,
      LieGroups.SO3.Quat.exp_map(deltaRotationCorrection_rad)));
  deltaVelocityBodyFlu_m_s := imu.deltaVelocityBodyFlu_m_s
    + imu.deltaVelocityGyroscopeBiasJacobian_m
      * gyroscopeBiasDelta_rad_s
    + imu.deltaVelocityAccelerometerBiasJacobian_s
      * accelerometerBiasDelta_m_s2;
  deltaPositionBodyFlu_m := imu.deltaPositionBodyFlu_m
    + imu.deltaPositionGyroscopeBiasJacobian_m_s
      * gyroscopeBiasDelta_rad_s
    + imu.deltaPositionAccelerometerBiasJacobian_s2
      * accelerometerBiasDelta_m_s2;
  annotation(Documentation(info = "<html>
    <h4>Bias-anchor transport</h4>
<p>The stored preintegral is evaluated at its two bias anchors. This function
multiplies each bias change by the corresponding stored Jacobian, right-injects
the rotation correction, and adds the velocity and position corrections. It
leaves the input packet unchanged.</p>
<p>Supply deltas, anchors and Jacobians from the same accumulation window, such
as the outputs of
<a href=\"modelica://Estimation.StrapdownINS.preintegrateImuStep\">preintegrateImuStep</a>.
The transport is first order in bias change. It is exact neither for arbitrary
bias changes nor for the underlying continuous IMU trajectory; large changes
require reassessing the anchor or reintegrating available raw samples.</p>
    </html>"));
end correctPreintegratedImu;
