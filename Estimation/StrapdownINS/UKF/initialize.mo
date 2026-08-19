within Estimation.StrapdownINS.UKF;

function initialize
  input Real positionWorldEnu_m[3];
  input Real velocityWorldEnu_m_s[3];
  input Real quaternionWorldBody[4];
  input Real gyroscopeBiasBodyFlu_rad_s[3];
  input Real accelerometerBiasBodyFlu_m_s2[3];
  input Estimation.StrapdownINS.InitialVariances variances;
  output Estimation.StrapdownINS.UKF.State state;
protected
  Estimation.StrapdownINS.ESKF.State initialized;
algorithm
  initialized := Estimation.StrapdownINS.ESKF.initialize(
    positionWorldEnu_m=positionWorldEnu_m,
    quaternionWorldBody=quaternionWorldBody,
    variances=variances,
    initialVelocityWorldEnu_m_s=velocityWorldEnu_m_s,
    initialGyroscopeBiasBodyFlu_rad_s=gyroscopeBiasBodyFlu_rad_s,
    initialAccelerometerBiasBodyFlu_m_s2=accelerometerBiasBodyFlu_m_s2);
  state := Estimation.StrapdownINS.UKF.State(
    positionWorldEnu_m=initialized.positionWorldEnu_m,
    velocityWorldEnu_m_s=initialized.velocityWorldEnu_m_s,
    quaternionWorldBody=initialized.quaternionWorldBody,
    gyroscopeBiasBodyFlu_rad_s=initialized.gyroscopeBiasBodyFlu_rad_s,
    accelerometerBiasBodyFlu_m_s2=initialized.accelerometerBiasBodyFlu_m_s2,
    covariance=initialized.covariance);
end initialize;
