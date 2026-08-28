within Estimation.StrapdownINS.ESKF;

function correctMocap "Correct position and attitude from motion capture"
  input Estimation.StrapdownINS.ESKF.State predicted;
  input Avionics.MocapSample measurement;
  input Real innovationGate = 0.0
    "Per-degree-of-freedom NIS gate; non-positive disables";
  input Real measurementAge_s(unit = "s") = 0.0;
  input Real angularVelocityMeasuredBodyFlu_rad_s[3] = zeros(3);
  input Real specificForceMeasuredBodyFlu_m_s2[3] = zeros(3);
  input Real gravityWorldEnu_m_s2[3] = {0.0, 0.0, -9.81};
  input Real maximumAidingDelay_s(unit = "s") = 0.25;
  output Estimation.StrapdownINS.ESKF.State corrected;
  output Boolean accepted;
  output Integer rejectionReason
    "Estimation.StrapdownINS.Correction* outcome code";
  output Real normalizedInnovationSquared;
protected
  Real delayedPosition[3];
  Real delayedQuaternion[4];
  Real delayedStateVector[16];
  Real currentToDelayed[TangentLength, TangentLength];
  Real rotationWorldBody[3, 3];
  Real residual[6];
  Real delayedH[6, TangentLength];
  Real H[6, TangentLength];
  Real measurementCovariance[6, 6];
  Real attitudeError[4];
algorithm
  // AGE ALIGNMENT, on the same terms as every other source. This stanza used
  // to be absent here and present in the four correct* functions beside it,
  // which made mocap the one source that was aged like the others and aligned
  // like nothing else: its residual was formed against the state at the
  // fusion instant while its measurement described the state one age earlier,
  // and its Jacobian was never transported at all. Nothing in a closed-loop
  // test distinguishes that from slightly wrong tuning.
  //
  // What the age MEANS has changed, and it is worth saying which of the two
  // it now is. Behind Estimation.FusionHorizon.AidingBuffer the measurement is
  // delivered at the first fusion instant at or after its own timestamp, so
  // the age is a residual inside one release window -- 10 ms at the flight
  // lattice -- and this is sub-window alignment. On the live-edge path the
  // same argument runs over the whole transport latency of the sensor, up to
  // maximumAidingDelay_s. The arithmetic is identical; only the size of the
  // interval, and so the size of the cubic-Taylor truncation the transport
  // carries, differs, and it differs by a factor of twenty-five.
  delayedStateVector := retrodict(predicted,
    angularVelocityMeasuredBodyFlu_rad_s,
    specificForceMeasuredBodyFlu_m_s2, gravityWorldEnu_m_s2,
    max(measurementAge_s, 0.0));
  delayedPosition := delayedStateVector[1:3];
  delayedQuaternion := delayedStateVector[7:10];
  currentToDelayed := discreteTransition(continuousTransition(
    angularVelocityMeasuredBodyFlu_rad_s
      - predicted.gyroscopeBiasBodyFlu_rad_s,
    specificForceMeasuredBodyFlu_m_s2
      - predicted.accelerometerBiasBodyFlu_m_s2),
    -max(measurementAge_s, 0.0));
  rotationWorldBody := LieGroups.SO3.Quat.to_DCM(delayedQuaternion);
  attitudeError := LieGroups.SO3.Quat.product(
    LieGroups.SO3.Quat.inverse(delayedQuaternion),
    LieGroups.SO3.Quat.normalize(measurement.quaternionWorldBody));
  residual := cat(1,
    transpose(rotationWorldBody)
      * (measurement.positionWorldEnu_m - delayedPosition),
    LieGroups.SO3.Quat.log_map(attitudeError));
  delayedH := cat(1,
    cat(2, identity(3), zeros(3, TangentLength - 3)),
    cat(2, zeros(3, 6), identity(3),
      zeros(3, TangentLength - 9)));
  H := delayedH * currentToDelayed;
  measurementCovariance := cat(1,
    cat(2, transpose(rotationWorldBody)
      * measurement.positionCovarianceWorld_m2 * rotationWorldBody,
      zeros(3, 3)),
    cat(2, zeros(3, 3), measurement.attitudeCovarianceBody_rad2));
  // A measurement the fusion instant has already passed, or one stamped in
  // the future, is REFUSED by timestamp rather than transported to meet the
  // state. The named outcome is what a supervisor can act on; a transported
  // one is an answer with an error nobody bounded.
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
  else
    (corrected, accepted, rejectionReason, normalizedInnovationSquared) :=
      correctLinear(predicted, residual, H, measurementCovariance,
        innovationGate);
  end if;
end correctMocap;
