within Estimation.StrapdownINS.UKF;

function stateVector
  "Flatten a public UKF state into its nominal numeric representation"
  input State state;
  output NominalVector nominal;
algorithm
  nominal := cat(1, state.positionWorldEnu_m,
    state.velocityWorldEnu_m_s, state.quaternionWorldBody,
    state.gyroscopeBiasBodyFlu_rad_s,
    state.accelerometerBiasBodyFlu_m_s2);
end stateVector;
