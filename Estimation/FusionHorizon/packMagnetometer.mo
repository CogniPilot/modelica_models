within Estimation.FusionHorizon;

function packMagnetometer "Flatten one magnetometer sample into a queue row"
  input Avionics.MagnetometerSample measurement;
  output Real row[MagnetometerMeasurementLength];
algorithm
  row := cat(1,
    {measurement.timestamp_s},
    measurement.magneticFieldBodyFlu_T,
    measurement.covarianceBody_T2[1, :],
    measurement.covarianceBody_T2[2, :],
    measurement.covarianceBody_T2[3, :]);
end packMagnetometer;
