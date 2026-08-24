within Estimation.StrapdownINS;

function preintegrateImuStep
  "Compose one IMU interval into a closed-form SE_2(3) preintegral"
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
  input Boolean useFirstOrderHold = false
    "True treats the interval input as linear between the previous and the current sample; false holds the current sample";
  input Real previousAngularVelocityMeasuredBodyFlu_rad_s[3] = zeros(3)
    "Measured angular velocity at the interval start (first-order hold only)";
  input Real previousSpecificForceMeasuredBodyFlu_m_s2[3] = zeros(3)
    "Measured specific force at the interval start (first-order hold only)";
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
  Real startAngularVelocity_rad_s[3];
  Real startSpecificForce_m_s2[3];
  Real angularVelocityDelta_rad_s[3];
  Real specificForceDelta_m_s2[3];
  Real rotationIncrement_rad[3];
  Real velocityIncrement_m_s[3];
  Real positionIncrement_m[3];
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
  if useFirstOrderHold then
    // First-order hold: the body input is linear between the interval-start
    // and interval-end samples.  The right increment is the truncated Magnus
    // exponent T*N0 + (T^2/2)*N1 + (T^3/12)*[N0, N1] whose single Lie
    // bracket decomposes into the classical coning, sculling, and position
    // (scrolling) corrections.  The deltas are formed as sample differences
    // (the constant bias anchor cancels) and the cross terms use those
    // differences directly, never two nearly parallel consecutive samples,
    // to avoid cancellation in single-precision generated code.
    startAngularVelocity_rad_s :=
      previousAngularVelocityMeasuredBodyFlu_rad_s
        - gyroscopeBiasLinearizationBodyFlu_rad_s;
    startSpecificForce_m_s2 := previousSpecificForceMeasuredBodyFlu_m_s2
      - accelerometerBiasLinearizationBodyFlu_m_s2;
    angularVelocityDelta_rad_s := angularVelocityMeasuredBodyFlu_rad_s
      - previousAngularVelocityMeasuredBodyFlu_rad_s;
    specificForceDelta_m_s2 := specificForceMeasuredBodyFlu_m_s2
      - previousSpecificForceMeasuredBodyFlu_m_s2;
    rotationIncrement_rad := (startAngularVelocity_rad_s
        + 0.5 * angularVelocityDelta_rad_s) * dt
      + (dt * dt / 12.0)
        * LieGroups.SO3.Quat.wedge(startAngularVelocity_rad_s)
        * angularVelocityDelta_rad_s;
    velocityIncrement_m_s := (startSpecificForce_m_s2
        + 0.5 * specificForceDelta_m_s2) * dt
      + (dt * dt / 12.0)
        * (LieGroups.SO3.Quat.wedge(startAngularVelocity_rad_s)
             * specificForceDelta_m_s2
           - LieGroups.SO3.Quat.wedge(angularVelocityDelta_rad_s)
             * startSpecificForce_m_s2);
    positionIncrement_m := -(dt * dt / 12.0) * specificForceDelta_m_s2;
  else
    startAngularVelocity_rad_s := correctedAngularVelocity_rad_s;
    startSpecificForce_m_s2 := correctedSpecificForce_m_s2;
    angularVelocityDelta_rad_s := zeros(3);
    specificForceDelta_m_s2 := zeros(3);
    rotationIncrement_rad := correctedAngularVelocity_rad_s * dt;
    velocityIncrement_m_s := correctedSpecificForce_m_s2 * dt;
    positionIncrement_m := zeros(3);
  end if;
  halfRotationIncrement_rad := 0.5 * rotationIncrement_rad;
  previousExtendedPose := cat(1, previousDeltaPosition_m,
    previousDeltaVelocity_m_s, previousDeltaQuaternion);

  // The mixed exponential is the closed-form solution from Lin, Pant,
  // Perseghetti, and Goppert, "On Closed-Form Preintegration for a Class of
  // Mixed-Invariant Systems in SE_n(3)", IEEE L-CSS, 2025.  With no world
  // input this integrates one body-rate/specific-force interval and includes
  // velocity-to-position coupling through the nilpotent B block.  Under the
  // first-order hold the scrolling correction feeds the position slot of the
  // body tangent, which is identically zero under the zero-order hold; the
  // bracket has no time-block component, so the closed-form machinery
  // applies verbatim.
  updatedExtendedPose := LieGroups.SE23.Quat.exp_mixed(
    previousExtendedPose,
    cat(1, positionIncrement_m, velocityIncrement_m_s,
      rotationIncrement_rad),
    zeros(9), [0.0, dt; 0.0, 0.0]);
  deltaPosition_m := updatedExtendedPose[1:3];
  deltaVelocity_m_s := updatedExtendedPose[4:6];
  deltaQuaternion := LieGroups.SO3.Quat.normalize(
    updatedExtendedPose[7:10]);

  // Bias sensitivities are the first variation of the same composition.
  // Evaluating the sensitivity at the interval midpoint keeps the Jacobians
  // second-order accurate while the nominal preintegral remains closed-form.
  // Under the first-order hold the increment sensitivities gain one
  // (dt^2/12) cross term per channel: the sample differences are invariant
  // to a constant bias, so only the interval-start factor of each bracket
  // differentiates.
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
  if useFirstOrderHold then
    rotationGyroscopeBiasJacobian_s := transpose(rotationIncrement)
        * previousRotationGyroscopeBiasJacobian_s
      + LieGroups.SO3.Quat.right_jacobian(rotationIncrement_rad)
        * ((dt * dt / 12.0)
             * LieGroups.SO3.Quat.wedge(angularVelocityDelta_rad_s)
           - dt * identity(3));
    velocityGyroscopeBiasJacobian_m :=
      previousVelocityGyroscopeBiasJacobian_m
        - rotationAtMidpoint
          * LieGroups.SO3.Quat.wedge(startSpecificForce_m_s2
              + 0.5 * specificForceDelta_m_s2)
          * rotationJacobianAtMidpoint_s * dt
        + rotationAtMidpoint * (dt * dt / 12.0)
          * LieGroups.SO3.Quat.wedge(specificForceDelta_m_s2);
    velocityAccelerometerBiasJacobian_s :=
      previousVelocityAccelerometerBiasJacobian_s
        - rotationAtMidpoint * dt
        + rotationAtMidpoint * (dt * dt / 12.0)
          * LieGroups.SO3.Quat.wedge(angularVelocityDelta_rad_s);
    positionGyroscopeBiasJacobian_m_s :=
      previousPositionGyroscopeBiasJacobian_m_s
        + previousVelocityGyroscopeBiasJacobian_m * dt
        - 0.5 * rotationAtMidpoint
          * LieGroups.SO3.Quat.wedge(startSpecificForce_m_s2
              + 0.5 * specificForceDelta_m_s2)
          * rotationJacobianAtMidpoint_s * dt * dt
        + 0.5 * rotationAtMidpoint * (dt * dt / 12.0)
          * LieGroups.SO3.Quat.wedge(specificForceDelta_m_s2) * dt;
    positionAccelerometerBiasJacobian_s2 :=
      previousPositionAccelerometerBiasJacobian_s2
        + previousVelocityAccelerometerBiasJacobian_s * dt
        - 0.5 * rotationAtMidpoint * dt * dt
        + 0.5 * rotationAtMidpoint * (dt * dt / 12.0)
          * LieGroups.SO3.Quat.wedge(angularVelocityDelta_rad_s) * dt;
  else
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
  end if;
end preintegrateImuStep;
