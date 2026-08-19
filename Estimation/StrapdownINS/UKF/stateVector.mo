within Estimation.StrapdownINS.UKF;

function stateVector
  "Flatten a public UKF state into its nominal numeric representation"
  input Estimation.StrapdownINS.UKF.State state;
  output Estimation.StrapdownINS.UKF.NominalVector nominal;
algorithm
  nominal := cat(1, state.positionWorldEnu_m,
    state.velocityWorldEnu_m_s, state.quaternionWorldBody,
    state.gyroscopeBiasBodyFlu_rad_s,
    state.accelerometerBiasBodyFlu_m_s2);
end stateVector;
