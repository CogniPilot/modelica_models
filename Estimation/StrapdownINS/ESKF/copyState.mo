within Estimation.StrapdownINS.ESKF;

function copyState "Copy a filter state and its covariance representation"
  input State previous;
  output State copied;
algorithm
  copied := State(positionWorldEnu_m=previous.positionWorldEnu_m,
    velocityWorldEnu_m_s=previous.velocityWorldEnu_m_s,
    quaternionWorldBody=previous.quaternionWorldBody,
    gyroscopeBiasBodyFlu_rad_s=previous.gyroscopeBiasBodyFlu_rad_s,
    accelerometerBiasBodyFlu_m_s2=previous.accelerometerBiasBodyFlu_m_s2,
    covariance=previous.covariance,
    useSquareRootCovariance=previous.useSquareRootCovariance,
    covarianceRoot=previous.covarianceRoot,
    barometerBiasCrossCovariance=previous.barometerBiasCrossCovariance);
end copyState;
