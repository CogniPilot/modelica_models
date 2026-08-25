within Estimation.MocapExternalOdometryErrorState;
record PoseMeasurementNoise "Diagonal pose measurement variances"
  Real attitude "Attitude measurement variance [rad2]";
  Real position "Position measurement variance [m2]";
end PoseMeasurementNoise;
