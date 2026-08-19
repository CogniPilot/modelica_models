within Estimation.StrapdownINS.UKF;

function predictNominalVector
  "Propagate a flattened nominal state through the strapdown mechanization"
  input Estimation.StrapdownINS.UKF.NominalVector previous;
  input Real angularVelocityMeasuredBodyFlu_rad_s[3];
  input Real specificForceMeasuredBodyFlu_m_s2[3];
  input Real gravityWorldEnu_m_s2[3];
  input Real dt(unit = "s");
  output Estimation.StrapdownINS.UKF.NominalVector predicted;
protected
  Real correctedAngularVelocity[3];
  Real correctedSpecificForce[3];
  Real leftIncrement[9];
  Real rightIncrement[9];
  Real predictedExtendedPose[10];
algorithm
  correctedAngularVelocity := angularVelocityMeasuredBodyFlu_rad_s
    - previous[11:13];
  correctedSpecificForce := specificForceMeasuredBodyFlu_m_s2
    - previous[14:16];
  leftIncrement := cat(1, zeros(3), correctedSpecificForce * dt,
    correctedAngularVelocity * dt);
  rightIncrement := cat(1, zeros(3), gravityWorldEnu_m_s2 * dt, zeros(3));
  predictedExtendedPose := LieGroups.SE23.Quat.exp_mixed(
    previous[1:10], leftIncrement, rightIncrement,
    [0.0, dt; 0.0, 0.0]);
  predicted := cat(1, predictedExtendedPose[1:6],
    LieGroups.SO3.Quat.normalize(predictedExtendedPose[7:10]),
    previous[11:16]);
end predictNominalVector;
