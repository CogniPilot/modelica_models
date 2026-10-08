within Estimation.StrapdownINS.ESKF.SemiDirectBias;

function inject
  "Inject pose and bias through the semi-direct group action"
  input NominalState nominal;
  input TangentVector correction;
  output NominalState corrected;
protected
  NominalState poseCorrected;
  Real biasCorrection[6];
algorithm
  poseCorrected := Estimation.StrapdownINS.ESKF.inject(nominal, correction);
  biasCorrection := LieGroups.SE3.Quat.right_jacobian(correction[4:9])
    * cat(1, correction[13:15], correction[10:12]);
  corrected := NominalState(
    positionWorldEnu_m=poseCorrected.positionWorldEnu_m,
    velocityWorldEnu_m_s=poseCorrected.velocityWorldEnu_m_s,
    quaternionWorldBody=poseCorrected.quaternionWorldBody,
    gyroscopeBiasBodyFlu_rad_s=
      nominal.gyroscopeBiasBodyFlu_rad_s + biasCorrection[4:6],
    accelerometerBiasBodyFlu_m_s2=
      nominal.accelerometerBiasBodyFlu_m_s2 + biasCorrection[1:3]);
end inject;
