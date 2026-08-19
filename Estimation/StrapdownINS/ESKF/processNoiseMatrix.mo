within Estimation.StrapdownINS.ESKF;

function processNoiseMatrix "Assemble the full continuous process covariance"
  input Estimation.StrapdownINS.ProcessNoise noise;
  output Estimation.StrapdownINS.ProcessNoiseCovariance covariance;
algorithm
  covariance := cat(1,
    cat(2, noise.gyroscope_rad2_s, zeros(3, 9)),
    cat(2, zeros(3, 3), noise.accelerometer_m2_s3, zeros(3, 6)),
    cat(2, zeros(3, 6), noise.gyroscopeBias_rad2_s3, zeros(3, 3)),
    cat(2, zeros(3, 9), noise.accelerometerBias_m2_s5));
end processNoiseMatrix;
