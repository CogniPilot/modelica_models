within Estimation.StrapdownINS.ESKF;

function inject "Right-inject a local tangent correction"
  input Estimation.StrapdownINS.ESKF.NominalState nominal;
  input Estimation.StrapdownINS.ESKF.TangentVector correction;
  output Estimation.StrapdownINS.ESKF.NominalState corrected;
protected
  Real extendedPose[10];
  Real groupCorrection[10];
  Real correctedExtendedPose[10];
algorithm
  extendedPose := cat(1,
    nominal.positionWorldEnu_m,
    nominal.velocityWorldEnu_m_s,
    nominal.quaternionWorldBody);
  groupCorrection := LieGroups.SE23.Quat.exp_map(correction[1:9]);
  correctedExtendedPose := LieGroups.SE23.Quat.product(
    extendedPose, groupCorrection);
  corrected := Estimation.StrapdownINS.ESKF.NominalState(
    positionWorldEnu_m=correctedExtendedPose[1:3],
    velocityWorldEnu_m_s=correctedExtendedPose[4:6],
    quaternionWorldBody=LieGroups.SO3.Quat.normalize(
      correctedExtendedPose[7:10]),
    gyroscopeBiasBodyFlu_rad_s=
      nominal.gyroscopeBiasBodyFlu_rad_s + correction[10:12],
    accelerometerBiasBodyFlu_m_s2=
      nominal.accelerometerBiasBodyFlu_m_s2 + correction[13:15]);
end inject;
