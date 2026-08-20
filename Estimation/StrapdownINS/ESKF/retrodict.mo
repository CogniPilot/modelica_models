within Estimation.StrapdownINS.ESKF;

function retrodict
  "Retrodict the nominal state with one held IMU sample"
  input Estimation.StrapdownINS.ESKF.State current;
  input Real angularVelocityMeasuredBodyFlu_rad_s[3];
  input Real specificForceMeasuredBodyFlu_m_s2[3];
  input Real gravityWorldEnu_m_s2[3];
  input Real age_s(unit = "s");
  output Real delayedStateVector[16]
    "{position,velocity,quaternion,gyro bias,accelerometer bias}";
protected
  Real correctedAngularVelocity[3];
  Real correctedSpecificForce[3];
  Real currentExtendedPose[10];
  Real leftIncrement[9];
  Real rightIncrement[9];
  Real delayedExtendedPose[10];
algorithm
  correctedAngularVelocity := angularVelocityMeasuredBodyFlu_rad_s
    - current.gyroscopeBiasBodyFlu_rad_s;
  correctedSpecificForce := specificForceMeasuredBodyFlu_m_s2
    - current.accelerometerBiasBodyFlu_m_s2;
  currentExtendedPose := cat(1, current.positionWorldEnu_m,
    current.velocityWorldEnu_m_s, current.quaternionWorldBody);
  leftIncrement := cat(1, zeros(3), -correctedSpecificForce * age_s,
    -correctedAngularVelocity * age_s);
  rightIncrement := cat(1, zeros(3), -gravityWorldEnu_m_s2 * age_s,
    zeros(3));
  delayedExtendedPose := LieGroups.SE23.Quat.exp_mixed(
    currentExtendedPose, leftIncrement, rightIncrement,
    [0.0, -age_s; 0.0, 0.0]);
  delayedStateVector := cat(1, delayedExtendedPose[1:6],
    LieGroups.SO3.Quat.normalize(delayedExtendedPose[7:10]),
    current.gyroscopeBiasBodyFlu_rad_s,
    current.accelerometerBiasBodyFlu_m_s2);
end retrodict;
