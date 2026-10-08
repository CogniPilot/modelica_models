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
  Real bodyIncrement[9];
  Real coupling[2, 2];
  Real incrementTranslationJacobian[6, 9];
  Real rotationBiasJacobian[3, 3];
  Real velocityGyroBiasJacobian[3, 3];
  Real translationBiasJacobian[6, 6];
  Real translationIncrement[3, 2];
  Real previousRotation[3, 3];
  Real previousExtendedPose[10];
  Real updatedExtendedPose[10];
  Real rotationIncrement[3, 3];
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
    // (scrolling) corrections.
    //
    // Magnus expansion: Magnus, Comm. Pure Appl. Math. 7(4):649-673, 1954;
    // survey Blanes, Casas, Oteo and Ros, Phys. Rep. 470:151-238, 2009.  The
    // truncation to third order, its exactly-vanishing T^4 grade, and the
    // O(T^5) residual are Lemma 2 and Theorem 2 of Reynolds, Condie,
    // Perseghetti and Goppert, "First-Order-Hold Magnus Preintegration on
    // SE_n(3) with Computable Flow-Error Bounds" (ACC 2027 manuscript).
    // Proposition 3 gives the coning/sculling/scrolling split. The coning term
    // coincides with the classical two-sample correction of Bortz (1971) and
    // Savage (1998, parts 1 and 2).  See the References block on
    // Estimation.StrapdownINS for the full entries.  The deltas are formed as sample differences
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
  bodyIncrement := cat(1, positionIncrement_m, velocityIncrement_m_s,
    rotationIncrement_rad);
  coupling := [0.0, dt; 0.0, 0.0];
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
    bodyIncrement, zeros(9), coupling);
  deltaPosition_m := updatedExtendedPose[1:3];
  deltaVelocity_m_s := updatedExtendedPose[4:6];
  deltaQuaternion := LieGroups.SO3.Quat.normalize(
    updatedExtendedPose[7:10]);

  // Equation (11) differentiates the retained exponent, not its physical
  // translation increments. Apply the chain rule through the same closed-form
  // increment block used above. This also differentiates the position-column
  // correction through rotation and transports the previous right attitude
  // sensitivity through the full increment, without a midpoint approximation.
  // Columns are {gyro bias, accelerometer bias}; rows are {position, velocity,
  // rotation}. Sample differences and the time block are bias-invariant.
  rotationBiasJacobian := -dt * identity(3)
    + (dt * dt / 12.0) * LieGroups.SO3.Quat.wedge(angularVelocityDelta_rad_s);
  velocityGyroBiasJacobian := (dt * dt / 12.0)
    * LieGroups.SO3.Quat.wedge(specificForceDelta_m_s2);
  translationIncrement := LieGroups.SE23.Quat.mixed_increment_matrix(
    bodyIncrement, coupling);
  incrementTranslationJacobian :=
    LieGroups.SE23.Quat.mixed_increment_matrix_jacobian(
      bodyIncrement, coupling);
  translationBiasJacobian := cat(2,
    incrementTranslationJacobian[:, 4:6] * velocityGyroBiasJacobian
      + incrementTranslationJacobian[:, 7:9] * rotationBiasJacobian,
    incrementTranslationJacobian[:, 4:6] * rotationBiasJacobian);
  previousRotation := LieGroups.SO3.Quat.to_DCM(previousDeltaQuaternion);
  rotationIncrement := LieGroups.SO3.Quat.to_DCM(
    LieGroups.SO3.Quat.exp_map(rotationIncrement_rad));
  rotationGyroscopeBiasJacobian_s := transpose(rotationIncrement)
      * previousRotationGyroscopeBiasJacobian_s
    + LieGroups.SO3.Quat.right_jacobian_exact(rotationIncrement_rad)
      * rotationBiasJacobian;
  velocityGyroscopeBiasJacobian_m := previousVelocityGyroscopeBiasJacobian_m
    + previousRotation * (translationBiasJacobian[1:3, 1:3]
      - LieGroups.SO3.Quat.wedge(translationIncrement[:, 1])
        * previousRotationGyroscopeBiasJacobian_s);
  velocityAccelerometerBiasJacobian_s :=
    previousVelocityAccelerometerBiasJacobian_s
      + previousRotation * translationBiasJacobian[1:3, 4:6];
  positionGyroscopeBiasJacobian_m_s :=
    previousPositionGyroscopeBiasJacobian_m_s
      + previousVelocityGyroscopeBiasJacobian_m * dt
      + previousRotation * (translationBiasJacobian[4:6, 1:3]
        - LieGroups.SO3.Quat.wedge(translationIncrement[:, 2])
          * previousRotationGyroscopeBiasJacobian_s);
  positionAccelerometerBiasJacobian_s2 :=
    previousPositionAccelerometerBiasJacobian_s2
      + previousVelocityAccelerometerBiasJacobian_s * dt
      + previousRotation * translationBiasJacobian[4:6, 4:6];
  annotation(Documentation(info = "<html>
    <h4>Accumulate an interval</h4>
<p>Start a new preintegral with zero position and velocity, identity quaternion
{1,0,0,0}, and zero bias Jacobians. Choose fixed gyroscope and accelerometer bias
anchors for the accumulation window. For each positive <code>dt</code>, feed the
previous outputs back as the next inputs and supply calibrated body FLU IMU data.
Position and velocity increments are expressed in the body frame at the window
start; gravity is applied by navigation prediction, not included here.</p>
<h4>Zero-order or first-order hold</h4>
<p>The default holds the current sample over the interval.
<code>useFirstOrderHold=true</code> uses the supplied interval-start sample and
current interval-end sample. Both endpoints must describe that same interval.
It evaluates the manuscript's retained Magnus exponent and differentiates its
mixed exponential and composition; it does not evaluate the continuous-flow
error certificate or an exact stochastic covariance.</p>
<h4>Publish the accumulated sample</h4>
<p>Copy the deltas and all five Jacobians to
<a href=\"modelica://Avionics.ImuSample\">ImuSample</a>, together with the unchanged
bias anchors, total integration duration and interval-end timestamp. Consume the
packet once. Use
<a href=\"modelica://Estimation.StrapdownINS.correctPreintegratedImu\">correctPreintegratedImu</a>
when the estimated bias changes. See the references in
<a href=\"modelica://Estimation.StrapdownINS\">StrapdownINS</a> for the underlying
ZOH, FOH and bias-anchor results.</p>
    </html>"));
end preintegrateImuStep;
