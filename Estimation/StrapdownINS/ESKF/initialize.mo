within Estimation.StrapdownINS.ESKF;

function initialize "Initialize the geometric error-state filter"
  input Real positionWorldEnu_m[3];
  input Real quaternionWorldBody[4];
  input Estimation.StrapdownINS.InitialVariances variances;
  input Real initialVelocityWorldEnu_m_s[3] = zeros(3);
  input Real initialGyroscopeBiasBodyFlu_rad_s[3] = zeros(3);
  input Real initialAccelerometerBiasBodyFlu_m_s2[3] = zeros(3);
  input Real initialPositionCovarianceWorld_m2[3, 3] = zeros(3, 3)
    "Covariance of an aiding position used as the seed; zero keeps configured priors";
  output State state;
protected
  Covariance initialCovariance;
algorithm
  initialCovariance := diagonal(cat(1,
    seedPositionVariances(initialPositionCovarianceWorld_m2,
      quaternionWorldBody, variances.position_m2),
    variances.velocity_m2_s2,
    variances.attitude_rad2,
    variances.gyroscopeBias_rad2_s2,
    variances.accelerometerBias_m2_s4));
  state := State(
    positionWorldEnu_m=positionWorldEnu_m,
    velocityWorldEnu_m_s=initialVelocityWorldEnu_m_s,
    quaternionWorldBody=LieGroups.SO3.Quat.normalize(quaternionWorldBody),
    gyroscopeBiasBodyFlu_rad_s=initialGyroscopeBiasBodyFlu_rad_s,
    accelerometerBiasBodyFlu_m_s2=initialAccelerometerBiasBodyFlu_m_s2,
    covariance=initialCovariance);
end initialize;
