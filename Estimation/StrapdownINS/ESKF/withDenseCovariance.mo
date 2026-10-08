within Estimation.StrapdownINS.ESKF;

function withDenseCovariance "Replace a dense covariance and preserve the state"
  input State previous;
  input Covariance covariance;
  input Real barometerBiasCrossCovariance[TangentLength];
  output State updated;
algorithm
  updated := State(positionWorldEnu_m=previous.positionWorldEnu_m,
    velocityWorldEnu_m_s=previous.velocityWorldEnu_m_s,
    quaternionWorldBody=previous.quaternionWorldBody,
    gyroscopeBiasBodyFlu_rad_s=previous.gyroscopeBiasBodyFlu_rad_s,
    accelerometerBiasBodyFlu_m_s2=previous.accelerometerBiasBodyFlu_m_s2,
    covariance=covariance, covarianceRoot=previous.covarianceRoot,
    useSquareRootCovariance=previous.useSquareRootCovariance,
    barometerBiasCrossCovariance=barometerBiasCrossCovariance,
    barometerBias_m=previous.barometerBias_m,
    barometerBiasVariance_m2=previous.barometerBiasVariance_m2,
    useJointBarometerBias=previous.useJointBarometerBias);
end withDenseCovariance;
