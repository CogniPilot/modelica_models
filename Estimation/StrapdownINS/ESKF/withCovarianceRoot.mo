within Estimation.StrapdownINS.ESKF;

function withCovarianceRoot "Set the authoritative root and its dense diagnostic"
  input State previous;
  input Covariance root;
  input Real barometerBiasCrossCovariance[TangentLength];
  output State updated;
algorithm
  updated := State(positionWorldEnu_m=previous.positionWorldEnu_m,
    velocityWorldEnu_m_s=previous.velocityWorldEnu_m_s,
    quaternionWorldBody=previous.quaternionWorldBody,
    gyroscopeBiasBodyFlu_rad_s=previous.gyroscopeBiasBodyFlu_rad_s,
    accelerometerBiasBodyFlu_m_s2=previous.accelerometerBiasBodyFlu_m_s2,
    covariance=LinearAlgebra.symmetrize(root * transpose(root)),
    covarianceRoot=root,
    useSquareRootCovariance=previous.useSquareRootCovariance,
    barometerBiasCrossCovariance=barometerBiasCrossCovariance);
end withCovarianceRoot;
