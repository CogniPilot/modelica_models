within Estimation.StrapdownINS;

record ProcessNoise "Continuous-time process-noise spectral-density matrices"
  Real gyroscope_rad2_s[3, 3];
  Real accelerometer_m2_s3[3, 3];
  Real gyroscopeBias_rad2_s3[3, 3];
  Real accelerometerBias_m2_s5[3, 3];
  annotation(Documentation(info = "<html>
    <h4>Continuous noise specification</h4>
<p>Each 3 by 3 matrix is a continuous-time covariance spectral density, not a
sample variance or a standard deviation. Gyroscope, accelerometer and their
bias random walks have different physical units, stated in the field names.</p>
<p>For an independent axis with amplitude density n, enter n squared on the
corresponding diagonal. Prediction performs the time integration; do not
multiply by the filter interval again when constructing this record. Off-diagonal
entries represent declared cross-axis correlations and must form a valid
covariance. Initial state uncertainty is configured separately through
<a href=\"modelica://Estimation.StrapdownINS.InitialVariances\">InitialVariances</a>.</p>
    </html>"));
end ProcessNoise;
