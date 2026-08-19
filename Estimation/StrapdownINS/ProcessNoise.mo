within Estimation.StrapdownINS;

record ProcessNoise "Continuous-time process-noise spectral-density matrices"
  Real gyroscope_rad2_s[3, 3];
  Real accelerometer_m2_s3[3, 3];
  Real gyroscopeBias_rad2_s3[3, 3];
  Real accelerometerBias_m2_s5[3, 3];
end ProcessNoise;
