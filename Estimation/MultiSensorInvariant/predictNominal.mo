within Estimation.MultiSensorInvariant;

function predictNominal
  "Propagate the nominal state with the existing mixed SE_2(3) exponential"
  input Estimation.MultiSensorInvariant.NominalState previous;
  input Real angularVelocityMeasuredBodyFlu_rad_s[3];
  input Real specificForceMeasuredBodyFlu_m_s2[3];
  input Real gravityWorldEnu_m_s2[3];
  input Real dt(unit = "s");
  output Estimation.MultiSensorInvariant.NominalState predicted;
protected
  Real correctedAngularVelocity[3];
  Real correctedSpecificForce[3];
  Real previousExtendedPose[10];
  Real leftIncrement[9];
  Real rightIncrement[9];
  Real predictedExtendedPose[10];
algorithm
  correctedAngularVelocity := angularVelocityMeasuredBodyFlu_rad_s
    - previous.gyroscopeBiasBodyFlu_rad_s;
  correctedSpecificForce := specificForceMeasuredBodyFlu_m_s2
    - previous.accelerometerBiasBodyFlu_m_s2;
  previousExtendedPose := cat(1,
    previous.positionWorldEnu_m,
    previous.velocityWorldEnu_m_s,
    previous.quaternionWorldBody);
  leftIncrement := cat(1,
    zeros(3), correctedSpecificForce * dt, correctedAngularVelocity * dt);
  rightIncrement := cat(1,
    zeros(3), gravityWorldEnu_m_s2 * dt, zeros(3));
  predictedExtendedPose := LieGroups.SE23.Quat.exp_mixed(
    previousExtendedPose,
    leftIncrement,
    rightIncrement,
    [0.0, dt; 0.0, 0.0]);
  predicted := Estimation.MultiSensorInvariant.NominalState(
    positionWorldEnu_m=predictedExtendedPose[1:3],
    velocityWorldEnu_m_s=predictedExtendedPose[4:6],
    quaternionWorldBody=LieGroups.SO3.Quat.normalize(
      predictedExtendedPose[7:10]),
    gyroscopeBiasBodyFlu_rad_s=previous.gyroscopeBiasBodyFlu_rad_s,
    accelerometerBiasBodyFlu_m_s2=previous.accelerometerBiasBodyFlu_m_s2);
end predictNominal;
