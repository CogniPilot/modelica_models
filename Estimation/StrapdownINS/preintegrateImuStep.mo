within Estimation.StrapdownINS;

function preintegrateImuStep
  "Compose one piecewise-constant IMU interval into a closed-form SE_2(3) preintegral"
  input Real previousDeltaPosition_m[3];
  input Real previousDeltaVelocity_m_s[3];
  input Real previousDeltaQuaternion[4];
  input Real previousRotationGyroscopeBiasJacobian_s[3, 3];
  input Real previousVelocityGyroscopeBiasJacobian_m[3, 3];
  input Real previousVelocityAccelerometerBiasJacobian_s[3, 3];
  input Real previousPositionGyroscopeBiasJacobian_m_s[3, 3];
  input Real previousPositionAccelerometerBiasJacobian_s2[3, 3];
  input Real angularVelocityMeasuredBodyFlu_rad_s[3];
  input Real specificForceMeasuredBodyFlu_m_s2[3];
  input Real gyroscopeBiasLinearizationBodyFlu_rad_s[3];
  input Real accelerometerBiasLinearizationBodyFlu_m_s2[3];
  input Real dt(unit = "s");
  output Real deltaPosition_m[3];
  output Real deltaVelocity_m_s[3];
  output Real deltaQuaternion[4];
  output Real rotationGyroscopeBiasJacobian_s[3, 3];
  output Real velocityGyroscopeBiasJacobian_m[3, 3];
  output Real velocityAccelerometerBiasJacobian_s[3, 3];
  output Real positionGyroscopeBiasJacobian_m_s[3, 3];
  output Real positionAccelerometerBiasJacobian_s2[3, 3];
protected
  Real correctedAngularVelocity_rad_s[3];
  Real correctedSpecificForce_m_s2[3];
  Real rotationIncrement_rad[3];
  Real halfRotationIncrement_rad[3];
  Real previousExtendedPose[10];
  Real updatedExtendedPose[10];
  Real rotationIncrement[3, 3];
  Real halfRotationIncrement[3, 3];
  Real rotationAtMidpoint[3, 3];
  Real rotationJacobianAtMidpoint_s[3, 3];
algorithm
  correctedAngularVelocity_rad_s := angularVelocityMeasuredBodyFlu_rad_s
    - gyroscopeBiasLinearizationBodyFlu_rad_s;
  correctedSpecificForce_m_s2 := specificForceMeasuredBodyFlu_m_s2
    - accelerometerBiasLinearizationBodyFlu_m_s2;
  rotationIncrement_rad := correctedAngularVelocity_rad_s * dt;
  halfRotationIncrement_rad := 0.5 * rotationIncrement_rad;
  previousExtendedPose := cat(1, previousDeltaPosition_m,
    previousDeltaVelocity_m_s, previousDeltaQuaternion);

  // The mixed exponential is the closed-form solution from Lin, Pant,
  // Perseghetti, and Goppert, "On Closed-Form Preintegration for a Class of
  // Mixed-Invariant Systems in SE_n(3)", IEEE L-CSS, 2025.  With no world
  // input this integrates one held body-rate/specific-force observation and
  // includes velocity-to-position coupling through the nilpotent B block.
  updatedExtendedPose := LieGroups.SE23.Quat.exp_mixed(
    previousExtendedPose,
    cat(1, zeros(3), correctedSpecificForce_m_s2 * dt,
      rotationIncrement_rad),
    zeros(9), [0.0, dt; 0.0, 0.0]);
  deltaPosition_m := updatedExtendedPose[1:3];
  deltaVelocity_m_s := updatedExtendedPose[4:6];
  deltaQuaternion := LieGroups.SO3.Quat.normalize(
    updatedExtendedPose[7:10]);

  // Bias sensitivities are the first variation of the same composition.
  // Evaluating the sensitivity at the interval midpoint keeps the Jacobians
  // second-order accurate while the nominal preintegral remains closed-form.
  rotationIncrement := LieGroups.SO3.Quat.to_DCM(
    LieGroups.SO3.Quat.exp_map(rotationIncrement_rad));
  halfRotationIncrement := LieGroups.SO3.Quat.to_DCM(
    LieGroups.SO3.Quat.exp_map(halfRotationIncrement_rad));
  rotationAtMidpoint := LieGroups.SO3.Quat.to_DCM(
    previousDeltaQuaternion) * halfRotationIncrement;
  rotationJacobianAtMidpoint_s := transpose(halfRotationIncrement)
      * previousRotationGyroscopeBiasJacobian_s
    - 0.5 * LieGroups.SO3.Quat.right_jacobian(
        halfRotationIncrement_rad) * dt;
  rotationGyroscopeBiasJacobian_s := transpose(rotationIncrement)
      * previousRotationGyroscopeBiasJacobian_s
    - LieGroups.SO3.Quat.right_jacobian(rotationIncrement_rad) * dt;
  velocityGyroscopeBiasJacobian_m :=
    previousVelocityGyroscopeBiasJacobian_m
      - rotationAtMidpoint
        * LieGroups.SO3.Quat.wedge(correctedSpecificForce_m_s2)
        * rotationJacobianAtMidpoint_s * dt;
  velocityAccelerometerBiasJacobian_s :=
    previousVelocityAccelerometerBiasJacobian_s
      - rotationAtMidpoint * dt;
  positionGyroscopeBiasJacobian_m_s :=
    previousPositionGyroscopeBiasJacobian_m_s
      + previousVelocityGyroscopeBiasJacobian_m * dt
      - 0.5 * rotationAtMidpoint
        * LieGroups.SO3.Quat.wedge(correctedSpecificForce_m_s2)
        * rotationJacobianAtMidpoint_s * dt * dt;
  positionAccelerometerBiasJacobian_s2 :=
    previousPositionAccelerometerBiasJacobian_s2
      + previousVelocityAccelerometerBiasJacobian_s * dt
      - 0.5 * rotationAtMidpoint * dt * dt;
end preintegrateImuStep;
