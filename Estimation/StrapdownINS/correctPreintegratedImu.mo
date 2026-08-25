within Estimation.StrapdownINS;

function correctPreintegratedImu
  "Move a stored IMU preintegral from its bias anchor to an estimated bias"
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
end correctPreintegratedImu;
