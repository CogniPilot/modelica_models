within Tests;

block SemiDirectBiasReplay
  "Deployment-export probe for the experimental semi-direct correction"
  input Real nominal[16];
  input Real correction[15];
  output Real corrected[16];
  output Real resetJacobian[15, 15];
protected
  Estimation.StrapdownINS.ESKF.NominalState correctedState;
algorithm
  when sample(0.0, 0.01) then
    correctedState := Estimation.StrapdownINS.ESKF.SemiDirectBias.inject(
      Estimation.StrapdownINS.ESKF.NominalState(
        positionWorldEnu_m=nominal[1:3], velocityWorldEnu_m_s=nominal[4:6],
        quaternionWorldBody=nominal[7:10], gyroscopeBiasBodyFlu_rad_s=nominal[11:13],
        accelerometerBiasBodyFlu_m_s2=nominal[14:16]), correction);
    corrected := cat(1, correctedState.positionWorldEnu_m,
      correctedState.velocityWorldEnu_m_s, correctedState.quaternionWorldBody,
      correctedState.gyroscopeBiasBodyFlu_rad_s,
      correctedState.accelerometerBiasBodyFlu_m_s2);
    resetJacobian := Estimation.StrapdownINS.ESKF.SemiDirectBias.resetJacobian(correction);
  end when;
end SemiDirectBiasReplay;
