within Estimation.FusionHorizon;

function rebiasDelta
  "Move a composed delta from its integration anchor to an estimated bias"
  input Estimation.FusionHorizon.Delta delta;
  input Real gyroscopeBiasDeltaBodyFlu_rad_s[3]
    "Estimated gyroscope bias minus the anchor the delta was integrated at";
  input Real accelerometerBiasDeltaBodyFlu_m_s2[3]
    "Estimated accelerometer bias minus the anchor the delta was integrated at";
  output Estimation.FusionHorizon.Delta rebiased;
protected
  Real rotationCorrection_rad[3];
algorithm
  // The estimator supplies a bias VALUE; the horizon supplies the Jacobians and
  // performs the move. Nothing about how the estimator arrived at that value
  // enters here, which is what keeps the same path usable by an additive-bias
  // ESKF, a manifold UKF, and any later filter: none of them get to inject an
  // error state into the buffer.
  //
  // This is the same first-order move Estimation.StrapdownINS.correctPreintegratedImu
  // performs on one packet, applied once to the whole composed window. Its
  // remainder is second order and bounded by the FOH paper Proposition 8
  // (Sec. VI-A): theta_remainder <= (T_D * ||db_g||)^2 with T_D the window span,
  // NOT the mission length. At a 200 ms horizon and a 0.05 rad/s bias offset
  // that is about 1e-4 rad, and it does not grow with flight time.
  rotationCorrection_rad := delta.deltaRotationGyroscopeBiasJacobian_s
    * gyroscopeBiasDeltaBodyFlu_rad_s;
  rebiased := Estimation.FusionHorizon.Delta(
    deltaPositionBodyFlu_m=delta.deltaPositionBodyFlu_m
      + delta.deltaPositionGyroscopeBiasJacobian_m_s
        * gyroscopeBiasDeltaBodyFlu_rad_s
      + delta.deltaPositionAccelerometerBiasJacobian_s2
        * accelerometerBiasDeltaBodyFlu_m_s2,
    deltaVelocityBodyFlu_m_s=delta.deltaVelocityBodyFlu_m_s
      + delta.deltaVelocityGyroscopeBiasJacobian_m
        * gyroscopeBiasDeltaBodyFlu_rad_s
      + delta.deltaVelocityAccelerometerBiasJacobian_s
        * accelerometerBiasDeltaBodyFlu_m_s2,
    deltaQuaternionBodyFlu=LieGroups.SO3.Quat.normalize(
      LieGroups.SO3.Quat.product(
        delta.deltaQuaternionBodyFlu,
        LieGroups.SO3.Quat.exp_map(rotationCorrection_rad))),
    integrationTime_s=delta.integrationTime_s,
    deltaRotationGyroscopeBiasJacobian_s=
      delta.deltaRotationGyroscopeBiasJacobian_s,
    deltaVelocityGyroscopeBiasJacobian_m=
      delta.deltaVelocityGyroscopeBiasJacobian_m,
    deltaVelocityAccelerometerBiasJacobian_s=
      delta.deltaVelocityAccelerometerBiasJacobian_s,
    deltaPositionGyroscopeBiasJacobian_m_s=
      delta.deltaPositionGyroscopeBiasJacobian_m_s,
    deltaPositionAccelerometerBiasJacobian_s2=
      delta.deltaPositionAccelerometerBiasJacobian_s2);
end rebiasDelta;
