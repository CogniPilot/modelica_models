within Estimation.StrapdownINS.ESKF;

function correctStationaryImu
  "Observe gyro bias and coupled gravity/accelerometer bias during declared rest"
  input State predicted;
  input Avionics.ImuSample imu;
  input Real gravityWorldEnu_m_s2[3];
  input Estimation.StrapdownINS.ProcessNoise processNoise;
  input Real innovationGate = 0.0;
  input Boolean useSemiDirectBias = false;
  output State corrected;
  output Boolean accepted;
  output Integer rejectionReason;
  output Real normalizedInnovationSquared;
protected
  Real dt;
  Real rotationIncrement[3];
  Real measuredAngularVelocity[3];
  Real measuredSpecificForce[3];
  Real gravityBodyFlu[3];
  Real residual[6];
  Real H[6, TangentLength];
  Real covariance[6, 6];
  Real stateNoiseCovariance[TangentLength, 6];
algorithm
  dt := imu.integrationTime_s;
  rotationIncrement := LieGroups.SO3.Quat.log_map(
    imu.deltaQuaternionBodyFlu);
  measuredAngularVelocity := rotationIncrement / dt
    + imu.gyroscopeBiasLinearizationBodyFlu_rad_s;
  measuredSpecificForce :=
    LieGroups.SO3.Quat.left_jacobian_inv(rotationIncrement)
      * imu.deltaVelocityBodyFlu_m_s / dt
    + imu.accelerometerBiasLinearizationBodyFlu_m_s2;
  gravityBodyFlu := -transpose(LieGroups.SO3.Quat.to_DCM(
    predicted.quaternionWorldBody)) * gravityWorldEnu_m_s2;
  residual := cat(1,
    measuredAngularVelocity - predicted.gyroscopeBiasBodyFlu_rad_s,
    measuredSpecificForce - gravityBodyFlu
      - predicted.accelerometerBiasBodyFlu_m_s2);
  H := cat(1,
    cat(2, zeros(3, 9), identity(3), zeros(3, 3)),
    cat(2, zeros(3, 6), LieGroups.SO3.Quat.wedge(gravityBodyFlu),
      zeros(3, 3), identity(3)));
  covariance := cat(1,
    cat(2, processNoise.gyroscope_rad2_s / dt
      + dt / 3.0 * processNoise.gyroscopeBias_rad2_s3, zeros(3, 3)),
    cat(2, zeros(3, 3), processNoise.accelerometer_m2_s3 / dt
      + dt / 3.0 * processNoise.accelerometerBias_m2_s5));
  stateNoiseCovariance := zeros(TangentLength, 6);
  stateNoiseCovariance[10:12, 1:3] :=
    -0.5 * dt * processNoise.gyroscopeBias_rad2_s3;
  stateNoiseCovariance[13:15, 4:6] :=
    -0.5 * dt * processNoise.accelerometerBias_m2_s5;
  (corrected, accepted, rejectionReason, normalizedInnovationSquared) :=
    correctLinear(predicted, residual, H, covariance, innovationGate,
      zeros(3), stateNoiseCovariance, false, useSemiDirectBias);
end correctStationaryImu;
