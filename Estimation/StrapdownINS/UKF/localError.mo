within Estimation.StrapdownINS.UKF;

function localError
  "Body-local tangent carrying reference to target"
  input Estimation.StrapdownINS.ESKF.NominalState reference;
  input Estimation.StrapdownINS.ESKF.NominalState target;
  output Estimation.StrapdownINS.UKF.TangentVector error;
protected
  Real referencePose[10];
  Real targetPose[10];
  Real groupError[10];
algorithm
  referencePose := cat(1, reference.positionWorldEnu_m,
    reference.velocityWorldEnu_m_s, reference.quaternionWorldBody);
  targetPose := cat(1, target.positionWorldEnu_m,
    target.velocityWorldEnu_m_s, target.quaternionWorldBody);
  groupError := LieGroups.SE23.Quat.product(
    LieGroups.SE23.Quat.inverse(referencePose), targetPose);
  error := cat(1,
    LieGroups.SE23.Quat.log_map(groupError),
    target.gyroscopeBiasBodyFlu_rad_s
      - reference.gyroscopeBiasBodyFlu_rad_s,
    target.accelerometerBiasBodyFlu_m_s2
      - reference.accelerometerBiasBodyFlu_m_s2);
end localError;
