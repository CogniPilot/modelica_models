within Estimation.StrapdownINS.ESKF;

function predict "Mixed SE_2(3) nominal and local error covariance prediction"
  input State previous;
  input Real angularVelocityMeasuredBodyFlu_rad_s[3];
  input Real specificForceMeasuredBodyFlu_m_s2[3];
  input Real gravityWorldEnu_m_s2[3];
  input Real dt(unit = "s");
  input Estimation.StrapdownINS.ProcessNoise processNoise;
  output State predicted;
protected
  NominalState previousNominal;
  NominalState predictedNominal;
  Real correctedAngularVelocity[3];
  Real correctedSpecificForce[3];
  Real A[TangentLength, TangentLength];
  Real transition[TangentLength, TangentLength];
algorithm
  previousNominal := NominalState(
    positionWorldEnu_m=previous.positionWorldEnu_m,
    velocityWorldEnu_m_s=previous.velocityWorldEnu_m_s,
    quaternionWorldBody=previous.quaternionWorldBody,
    gyroscopeBiasBodyFlu_rad_s=previous.gyroscopeBiasBodyFlu_rad_s,
    accelerometerBiasBodyFlu_m_s2=previous.accelerometerBiasBodyFlu_m_s2);
  predictedNominal := predictNominal(
    previousNominal,
    angularVelocityMeasuredBodyFlu_rad_s,
    specificForceMeasuredBodyFlu_m_s2,
    gravityWorldEnu_m_s2,
    dt);
  correctedAngularVelocity := angularVelocityMeasuredBodyFlu_rad_s
    - previous.gyroscopeBiasBodyFlu_rad_s;
  correctedSpecificForce := specificForceMeasuredBodyFlu_m_s2
    - previous.accelerometerBiasBodyFlu_m_s2;
  A := continuousTransition(correctedAngularVelocity, correctedSpecificForce);
  transition := discreteTransition(A, dt);
  predicted := predictCovariance(previous, transition, A, processNoise, dt);
  predicted := State(
    positionWorldEnu_m=predictedNominal.positionWorldEnu_m,
    velocityWorldEnu_m_s=predictedNominal.velocityWorldEnu_m_s,
    quaternionWorldBody=predictedNominal.quaternionWorldBody,
    gyroscopeBiasBodyFlu_rad_s=
      predictedNominal.gyroscopeBiasBodyFlu_rad_s,
    accelerometerBiasBodyFlu_m_s2=
      predictedNominal.accelerometerBiasBodyFlu_m_s2,
    covariance=predicted.covariance,
    covarianceRoot=predicted.covarianceRoot,
    barometerBiasCrossCovariance=predicted.barometerBiasCrossCovariance,
    useSquareRootCovariance=previous.useSquareRootCovariance);
end predict;
