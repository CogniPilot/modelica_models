within Estimation.StrapdownINS.ESKF;

function correctGps "Jointly correct GPS position and velocity"
  input State predicted;
  input Avionics.GpsSample measurement;
  input Real innovationGate = 0.0
    "Per-degree-of-freedom NIS gate; non-positive disables";
  input Real measurementAge_s(unit = "s") = 0.0;
  input Real angularVelocityMeasuredBodyFlu_rad_s[3] = zeros(3);
  input Real specificForceMeasuredBodyFlu_m_s2[3] = zeros(3);
  input Real gravityWorldEnu_m_s2[3] = {0.0, 0.0, -9.81};
  input Real maximumAidingDelay_s(unit = "s") = 0.25;
  input Real heldImuCovariance[6, 6] = zeros(6, 6)
    "Covariance of the held packet mean, {gyroscope, accelerometer}";
  input Real predictionInterval_s(unit = "s") = 0.0
    "Interval over which this same packet just predicted the current state";
  input Boolean useSemiDirectBias = false;
  output State corrected;
  output Boolean accepted;
  output Integer rejectionReason
    "Estimation.StrapdownINS.Correction* outcome code";
  output Real normalizedInnovationSquared;
protected
  Real rotationWorldBody[3, 3];
  Real delayedPosition[3];
  Real delayedVelocity[3];
  Real delayedQuaternion[4];
  Real delayedStateVector[16];
  Real currentToDelayed[TangentLength, TangentLength];
  Real A[TangentLength, TangentLength];
  Real forwardInput[TangentLength, 6];
  Real backwardInput[TangentLength, 6];
  Real observationInput[6, 6];
  Real measurementStateCrossCovariance[TangentLength, 6];
  Real residual[6];
  Real delayedH[6, TangentLength];
  Real H[6, TangentLength];
  Real measurementCovariance[6, 6];
algorithm
  delayedStateVector := retrodict(predicted,
    angularVelocityMeasuredBodyFlu_rad_s,
    specificForceMeasuredBodyFlu_m_s2, gravityWorldEnu_m_s2,
    max(measurementAge_s, 0.0));
  delayedPosition := delayedStateVector[1:3];
  delayedVelocity := delayedStateVector[4:6];
  delayedQuaternion := delayedStateVector[7:10];
  A := continuousTransition(
    angularVelocityMeasuredBodyFlu_rad_s
      - predicted.gyroscopeBiasBodyFlu_rad_s,
    specificForceMeasuredBodyFlu_m_s2
      - predicted.accelerometerBiasBodyFlu_m_s2);
  currentToDelayed := discreteTransition(A, -max(measurementAge_s, 0.0));
  rotationWorldBody := LieGroups.SO3.Quat.to_DCM(
    delayedQuaternion);
  residual := cat(1,
    transpose(rotationWorldBody)
      * (measurement.positionWorldEnu_m - delayedPosition),
    transpose(rotationWorldBody)
      * (measurement.velocityWorldEnu_m_s - delayedVelocity));
  delayedH := cat(1,
    cat(2, identity(3), zeros(3, TangentLength - 3)),
    cat(2, zeros(3, 3), identity(3),
      zeros(3, TangentLength - 6)));
  H := delayedH * currentToDelayed;
  measurementCovariance := cat(1,
    cat(2, transpose(rotationWorldBody)
      * measurement.positionCovarianceWorld_m2 * rotationWorldBody,
      zeros(3, 3)),
    cat(2, zeros(3, 3), transpose(rotationWorldBody)
      * measurement.velocityCovarianceWorld_m2_s2 * rotationWorldBody));
  measurementStateCrossCovariance := zeros(TangentLength, 6);
  if predictionInterval_s > 0.0 and measurementAge_s > 0.0 then
    // Retrodiction reuses the noisy mean of the newest IMU packet. It is
    // neither exact nor independent of the state that packet just predicted.
    // Marginalize that same sample in both the observation covariance and
    // the state/observation cross covariance rather than counting it twice.
    forwardInput := heldInputJacobian(A, predictionInterval_s);
    backwardInput := heldInputJacobian(A, -measurementAge_s);
    observationInput := delayedH * backwardInput;
    measurementCovariance := measurementCovariance
      + observationInput * heldImuCovariance * transpose(observationInput);
    measurementStateCrossCovariance := forwardInput * heldImuCovariance
      * transpose(observationInput);
  end if;
  corrected := copyState(predicted);
  accepted := false;
  normalizedInnovationSquared := 0.0;
  if measurementAge_s < -1.0e-6
      or measurementAge_s > maximumAidingDelay_s then
    rejectionReason := CorrectionRejectedTimestamp;
  else
    (corrected, accepted, rejectionReason, normalizedInnovationSquared) :=
      correctLinear(predicted, residual, H, measurementCovariance,
        innovationGate, zeros(3), measurementStateCrossCovariance, false,
          useSemiDirectBias);
  end if;
  annotation(Documentation(info = "<html>
    <p>Retrodict the nominal state to GPS capture time using the latest held
    IMU input. That input is noisy and correlated with the current state it
    just predicted. When packet uncertainty is supplied, the update accounts
    for both observation variance and state/observation cross-covariance.</p>
    <pre>
H = Hd Phi(-age)
J = Hd B(-age)
R = Rgps + J Q J'
C = B(dt) Q J'
S = H P H' + H C + C' H' + R
K = (P H' + C) S^-1
F = I - K H
Pposterior = F P F' + K R K' - F C K' - K C' F'
    </pre>
    <p><code>B</code> is
    <a href=\"modelica://Estimation.StrapdownINS.ESKF.heldInputJacobian\">heldInputJacobian</a>;
    <code>Q</code> is the covariance of the packet's mean gyro/accelerometer
    input. The generalized Joseph form also supports a constrained gain.
    Optional packet covariance defaults to zero.</p>
    <p>Retrodiction assumes constant input during the delay. Use a delayed
    fusion wrapper when motion varies substantially within that interval.
    <a href=\"modelica://Tests.CorrelatedGpsTests\">CorrelatedGpsTests</a>
    checks the covariance, gain and innovation statistic against independent
    linear-Gaussian values.</p>
    </html>"));
end correctGps;
