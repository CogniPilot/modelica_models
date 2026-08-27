within Estimation.StrapdownINS.ESKF;

function correctMagnetometer
  "Tilt-compensate raw magnetic field and correct yaw only"
  input Estimation.StrapdownINS.ESKF.State predicted;
  input Avionics.MagnetometerSample measurement;
  input Real magneticFieldWorldEnu_T[3];
  input Real innovationGate = 0.0;
  input Real measurementAge_s(unit = "s") = 0.0;
  input Real angularVelocityMeasuredBodyFlu_rad_s[3] = zeros(3);
  input Real specificForceMeasuredBodyFlu_m_s2[3] = zeros(3);
  input Real gravityWorldEnu_m_s2[3] = {0.0, 0.0, -9.81};
  input Real maximumAidingDelay_s(unit = "s") = 0.25;
  output Estimation.StrapdownINS.ESKF.State corrected;
  output Boolean accepted;
  output Integer rejectionReason;
  output Real normalizedInnovationSquared;
protected
  Real delayedPosition[3];
  Real delayedVelocity[3];
  Real delayedQuaternion[4];
  Real delayedGyroscopeBias[3];
  Real delayedAccelerometerBias[3];
  Real delayedStateVector[16];
  Real currentToDelayed[TangentLength, TangentLength];
  Boolean delayAccepted;
  Boolean measurementUsable;
  Real measuredHeading;
  Real measuredHeadingVariance;
  Real euler[3];
  Real cosPitch;
  Real delayedH[1, TangentLength];
  Real H[1, TangentLength];
  Real yawSensitivityBodyFlu[3];
  Real tiltSensitivityBodyFlu[3];
  Real predictedRotationWorldBody[3, 3];
  Real verticalAxisBodyFlu[3];
  Real residual[1];
  Real covariance[1, 1];
algorithm
  delayedStateVector := retrodict(predicted,
    angularVelocityMeasuredBodyFlu_rad_s,
    specificForceMeasuredBodyFlu_m_s2, gravityWorldEnu_m_s2,
    max(measurementAge_s, 0.0));
  delayedPosition := delayedStateVector[1:3];
  delayedVelocity := delayedStateVector[4:6];
  delayedQuaternion := delayedStateVector[7:10];
  delayedGyroscopeBias := delayedStateVector[11:13];
  delayedAccelerometerBias := delayedStateVector[14:16];
  currentToDelayed := discreteTransition(continuousTransition(
    angularVelocityMeasuredBodyFlu_rad_s
      - predicted.gyroscopeBiasBodyFlu_rad_s,
    specificForceMeasuredBodyFlu_m_s2
      - predicted.accelerometerBiasBodyFlu_m_s2),
    -max(measurementAge_s, 0.0));
  delayAccepted := true;
  (measuredHeading, measuredHeadingVariance, measurementUsable,
   yawSensitivityBodyFlu, tiltSensitivityBodyFlu) :=
    Estimation.StrapdownINS.magnetometerYawObservation(
      delayedQuaternion, measurement.magneticFieldBodyFlu_T,
      measurement.covarianceBody_T2, magneticFieldWorldEnu_T);
  euler := LieGroups.SO3.EulerB321.from_Quat(
    delayedQuaternion);
  cosPitch := cos(euler[2]);
  residual[1] := MathUtilities.wrapAngle(measuredHeading - euler[1]);
  delayedH := zeros(1, TangentLength);
  if abs(cosPitch) > 0.1 then
    // THE ROW THE RESIDUAL ACTUALLY HAS.
    //
    // The previous row was the Euler yaw-rate mapping with literal zeros
    // on the two tilt entries, on the reasoning that magnetic data must
    // not move roll or pitch. But the quantity being differenced is not a
    // yaw measurement: it is a heading LEVELLED WITH THE FILTER'S OWN roll
    // and pitch, so a tilt error rotates the levelled field and moves the
    // reported heading by about tan(inclination) times that error -- 2.4
    // at the RDD2 test site. Declaring that sensitivity to be zero does not
    // make the residual insensitive to tilt; it only hides the term from
    // the innovation covariance and from the Joseph update, so the heading
    // variance converges to a floor set by magnetometer noise alone while
    // the heading error keeps the tilt-transfer part in full.
    //
    // Measured on both RDD2 qualification missions, five seeds each: the
    // attitude block ran ANEES 6.4 against 3 degrees of freedom with
    // essentially all of it in the heading axis (per-axis normalized error
    // squared about 0.45, 0.45, 5.6 for east, north, up), and the transfer
    // term alone was worth about 13 times the reported heading variance.
    // The magnetometer NIS, 1.32 against 1, is the same term seen from the
    // innovation side.
    //
    // Inflating the reported measurement variance cannot repair it: the
    // steady-state heading variance grows only as the SQUARE ROOT of R, so
    // removing a factor 6 of optimism would take a factor 36 of invented
    // magnetometer noise -- roughly 7 degrees of heading noise, which
    // discards the heading rather than describing it. The missing term is
    // a Jacobian, so it is supplied as one.
    //
    // Supplying the row does NOT mean the magnetometer may now correct
    // roll and pitch. It must not: an inclined field makes the heading
    // sensitive to tilt, but reading tilt back out of it would trust the
    // inclination model and the freedom from hard- and soft-iron error
    // that a magnetometer cannot promise, and it excites the tilt and
    // accelerometer-bias pair in exactly the low-dynamics regimes where
    // that pair is weakly observable. Measured with the tilt gain left
    // optimal: the heading axis came back to normalized error squared
    // 0.5 to 1.2 and the magnetometer NIS to 0.99, but the east tilt axis
    // went from 0.5 to 5.0 and to 9.2 while the vehicle sat still on the
    // ground -- the optimism moved axis instead of leaving.
    //
    // So the gain is projected onto the body vertical below: this
    // correction rotates the state about the vertical and about nothing
    // else, while the tilt sensitivity still enters the innovation
    // covariance and the Joseph update, which is what the covariance
    // needed all along.
    delayedH[1, 7] := yawSensitivityBodyFlu[1] + tiltSensitivityBodyFlu[1];
    delayedH[1, 8] := yawSensitivityBodyFlu[2] + tiltSensitivityBodyFlu[2];
    delayedH[1, 9] := yawSensitivityBodyFlu[3] + tiltSensitivityBodyFlu[3];
  end if;
  H := delayedH * currentToDelayed;
  // The gain acts in the CURRENT tangent, so the axis it is allowed to
  // rotate about is the world vertical resolved in the current body frame.
  predictedRotationWorldBody :=
    LieGroups.SO3.Quat.to_DCM(predicted.quaternionWorldBody);
  verticalAxisBodyFlu := predictedRotationWorldBody[3, :];
  covariance[1, 1] := measuredHeadingVariance;
  if measurementAge_s < -1.0e-6
      or measurementAge_s > maximumAidingDelay_s then
    corrected := Estimation.StrapdownINS.ESKF.State(
      positionWorldEnu_m=predicted.positionWorldEnu_m,
      velocityWorldEnu_m_s=predicted.velocityWorldEnu_m_s,
      quaternionWorldBody=predicted.quaternionWorldBody,
      gyroscopeBiasBodyFlu_rad_s=predicted.gyroscopeBiasBodyFlu_rad_s,
      accelerometerBiasBodyFlu_m_s2=predicted.accelerometerBiasBodyFlu_m_s2,
      covariance=predicted.covariance);
    accepted := false;
    rejectionReason := CorrectionRejectedTimestamp;
    normalizedInnovationSquared := 0.0;
  elseif not delayAccepted or not measurementUsable
      or abs(cosPitch) <= 0.1 then
    corrected := Estimation.StrapdownINS.ESKF.State(
      positionWorldEnu_m=predicted.positionWorldEnu_m,
      velocityWorldEnu_m_s=predicted.velocityWorldEnu_m_s,
      quaternionWorldBody=predicted.quaternionWorldBody,
      gyroscopeBiasBodyFlu_rad_s=predicted.gyroscopeBiasBodyFlu_rad_s,
      accelerometerBiasBodyFlu_m_s2=predicted.accelerometerBiasBodyFlu_m_s2,
      covariance=predicted.covariance);
    accepted := false;
    rejectionReason := CorrectionRejectedCovarianceUnusable;
    normalizedInnovationSquared := 0.0;
  else
    (corrected, accepted, rejectionReason, normalizedInnovationSquared) :=
      correctLinear(predicted, residual, H, covariance, innovationGate,
        verticalAxisBodyFlu);
  end if;
end correctMagnetometer;
