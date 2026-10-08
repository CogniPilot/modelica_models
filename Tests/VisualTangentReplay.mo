within Tests;

block VisualTangentReplay "Visual error-basis and actual Lie injection qualification"
  input Real quaternionWorldBody[4] = {1.0, 0.0, 0.0, 0.0};
  input Real correction[15] = zeros(15);
  output Real transform[15, 15];
  output Real injected[16];
protected
  Estimation.StrapdownINS.ESKF.NominalState nominal(
    positionWorldEnu_m=zeros(3), velocityWorldEnu_m_s=zeros(3),
    quaternionWorldBody=quaternionWorldBody,
    gyroscopeBiasBodyFlu_rad_s=zeros(3), accelerometerBiasBodyFlu_m_s2=zeros(3));
  Estimation.StrapdownINS.ESKF.NominalState corrected;
algorithm
  when sample(0.0, 0.01) then
    transform := Estimation.StrapdownINS.ESKF.Visual.worldErrorTransform(
      LieGroups.SO3.Quat.to_DCM(quaternionWorldBody));
    corrected := Estimation.StrapdownINS.ESKF.inject(nominal, correction);
    injected := cat(1, corrected.positionWorldEnu_m, corrected.velocityWorldEnu_m_s,
      corrected.quaternionWorldBody, corrected.gyroscopeBiasBodyFlu_rad_s,
      corrected.accelerometerBiasBodyFlu_m_s2);
  end when;
end VisualTangentReplay;
