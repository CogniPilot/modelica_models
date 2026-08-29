within Estimation.StrapdownINS.ESKF;

function predictPreintegrated
  "Apply one closed-form IMU preintegral and its exact local transition"
  // The preintegral applied here is the closed-form SE_2(3) mixed exponential
  // of Lin, Pant, Perseghetti and Goppert, "On Closed-Form Preintegration for
  // a Class of Mixed-Invariant Systems in SE_n(3)," IEEE L-CSS 2025, under the
  // first-order-hold extension of the companion manuscript; the bias columns
  // of the transition are its bias-sensitivity Jacobians, transported by the
  // preintegrated rotation.  The right-perturbation error convention and the
  // Phi P Phi' + Q discretization follow Barfoot, State Estimation for
  // Robotics, 2nd ed., Section 9.4.  References: Estimation.StrapdownINS.
  input Estimation.StrapdownINS.ESKF.State previous;
  input Avionics.ImuSample imu;
  input Real gravityWorldEnu_m_s2[3];
  input Estimation.StrapdownINS.ProcessNoise processNoise;
  output Estimation.StrapdownINS.ESKF.State predicted;
protected
  Real dt;
  Real deltaPositionBodyFlu_m[3];
  Real deltaVelocityBodyFlu_m_s[3];
  Real deltaQuaternionBodyFlu[4];
  Real deltaRotationBodyFlu[3, 3];
  Real previousRotationWorldBody[3, 3];
  Real transition[TangentLength, TangentLength];
  Real rotationIncrement_rad[3];
  Real equivalentAngularVelocity_rad_s[3];
  Real equivalentSpecificForce_m_s2[3];
  Real A[TangentLength, TangentLength];
  Real G[TangentLength, ProcessNoiseLength];
  Estimation.StrapdownINS.ProcessNoiseCovariance continuousNoise;
  Estimation.StrapdownINS.ESKF.Covariance discreteNoise;
algorithm
  dt := imu.integrationTime_s;
  (deltaPositionBodyFlu_m,
   deltaVelocityBodyFlu_m_s,
   deltaQuaternionBodyFlu) :=
    Estimation.StrapdownINS.correctPreintegratedImu(
      imu,
      previous.gyroscopeBiasBodyFlu_rad_s,
      previous.accelerometerBiasBodyFlu_m_s2);
  deltaRotationBodyFlu :=
    LieGroups.SO3.Quat.to_DCM(deltaQuaternionBodyFlu);
  previousRotationWorldBody :=
    LieGroups.SO3.Quat.to_DCM(previous.quaternionWorldBody);

  transition := zeros(TangentLength, TangentLength);
  transition[1:3, 1:3] := transpose(deltaRotationBodyFlu);
  transition[1:3, 4:6] := transpose(deltaRotationBodyFlu) * dt;
  transition[1:3, 7:9] := -transpose(deltaRotationBodyFlu)
    * LieGroups.SO3.Quat.wedge(deltaPositionBodyFlu_m);
  transition[1:3, 10:12] := transpose(deltaRotationBodyFlu)
    * imu.deltaPositionGyroscopeBiasJacobian_m_s;
  transition[1:3, 13:15] := transpose(deltaRotationBodyFlu)
    * imu.deltaPositionAccelerometerBiasJacobian_s2;
  transition[4:6, 4:6] := transpose(deltaRotationBodyFlu);
  transition[4:6, 7:9] := -transpose(deltaRotationBodyFlu)
    * LieGroups.SO3.Quat.wedge(deltaVelocityBodyFlu_m_s);
  transition[4:6, 10:12] := transpose(deltaRotationBodyFlu)
    * imu.deltaVelocityGyroscopeBiasJacobian_m;
  transition[4:6, 13:15] := transpose(deltaRotationBodyFlu)
    * imu.deltaVelocityAccelerometerBiasJacobian_s;
  transition[7:9, 7:9] := transpose(deltaRotationBodyFlu);
  transition[7:9, 10:12] :=
    imu.deltaRotationGyroscopeBiasJacobian_s;
  transition[10:12, 10:12] := 1.0 * identity(3);
  transition[13:15, 13:15] := 1.0 * identity(3);

  rotationIncrement_rad :=
    LieGroups.SO3.Quat.log_map(deltaQuaternionBodyFlu);
  equivalentAngularVelocity_rad_s := rotationIncrement_rad / dt;
  equivalentSpecificForce_m_s2 :=
    LieGroups.SO3.Quat.left_jacobian_inv(rotationIncrement_rad)
      * deltaVelocityBodyFlu_m_s / dt;
  A := continuousTransition(
    equivalentAngularVelocity_rad_s, equivalentSpecificForce_m_s2);
  G := noiseInputMatrix();
  continuousNoise := processNoiseMatrix(processNoise);
  discreteNoise := discreteProcessCovariance(
    A, G, continuousNoise, dt);

  predicted := Estimation.StrapdownINS.ESKF.State(
    positionWorldEnu_m=previous.positionWorldEnu_m
      + previous.velocityWorldEnu_m_s * dt
      + 0.5 * gravityWorldEnu_m_s2 * dt * dt
      + previousRotationWorldBody * deltaPositionBodyFlu_m,
    velocityWorldEnu_m_s=previous.velocityWorldEnu_m_s
      + gravityWorldEnu_m_s2 * dt
      + previousRotationWorldBody * deltaVelocityBodyFlu_m_s,
    quaternionWorldBody=LieGroups.SO3.Quat.normalize(
      LieGroups.SO3.Quat.product(
        previous.quaternionWorldBody, deltaQuaternionBodyFlu)),
    gyroscopeBiasBodyFlu_rad_s=previous.gyroscopeBiasBodyFlu_rad_s,
    accelerometerBiasBodyFlu_m_s2=
      previous.accelerometerBiasBodyFlu_m_s2,
    covariance=LinearAlgebra.symmetrize(
      transition * previous.covariance * transpose(transition)
        + discreteNoise));
end predictPreintegrated;
