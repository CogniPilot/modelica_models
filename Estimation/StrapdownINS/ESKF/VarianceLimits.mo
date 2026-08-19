within Estimation.StrapdownINS.ESKF;

record VarianceLimits "Mission-envelope upper bounds on the tangent variances"
  Real position_m2[3];
  Real velocity_m2_s2[3];
  Real attitude_rad2[3];
  Real gyroscopeBias_rad2_s2[3];
  Real accelerometerBias_m2_s4[3];
end VarianceLimits;
