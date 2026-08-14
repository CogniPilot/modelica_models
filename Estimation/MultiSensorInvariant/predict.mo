within Estimation.MultiSensorInvariant;

function predict "Mixed SE_2(3) nominal and local error covariance prediction"
  input Estimation.MultiSensorInvariant.State previous;
  input Real angularVelocityMeasuredBodyFlu_rad_s[3];
  input Real specificForceMeasuredBodyFlu_m_s2[3];
  input Real gravityWorldEnu_m_s2[3];
  input Real dt(unit = "s");
  input Estimation.MultiSensorInvariant.ProcessNoise processNoise;
  output Estimation.MultiSensorInvariant.State predicted;
protected
  Estimation.MultiSensorInvariant.NominalState previousNominal;
  Estimation.MultiSensorInvariant.NominalState predictedNominal;
  Real correctedAngularVelocity[3];
  Real correctedSpecificForce[3];
  Real A[TangentLength, TangentLength];
  Real G[TangentLength, ProcessNoiseLength];
  Real transition[TangentLength, TangentLength];
  Estimation.MultiSensorInvariant.ProcessNoiseCovariance continuousNoise;
  Estimation.MultiSensorInvariant.Covariance discreteNoise;
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
  G := noiseInputMatrix();
  transition := discreteTransition(A, dt);
  continuousNoise := processNoiseMatrix(processNoise);
  discreteNoise := discreteProcessCovariance(
    A, G, continuousNoise, dt);
  predicted := Estimation.MultiSensorInvariant.State(
    positionWorldEnu_m=predictedNominal.positionWorldEnu_m,
    velocityWorldEnu_m_s=predictedNominal.velocityWorldEnu_m_s,
    quaternionWorldBody=predictedNominal.quaternionWorldBody,
    gyroscopeBiasBodyFlu_rad_s=
      predictedNominal.gyroscopeBiasBodyFlu_rad_s,
    accelerometerBiasBodyFlu_m_s2=
      predictedNominal.accelerometerBiasBodyFlu_m_s2,
    covariance=LinearAlgebra.symmetrize(
      transition * previous.covariance * transpose(transition)
        + discreteNoise));
end predict;
