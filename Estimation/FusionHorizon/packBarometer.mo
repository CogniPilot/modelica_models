within Estimation.FusionHorizon;

function packBarometer "Flatten one barometer sample into a queue row"
  input Avionics.BarometerSample measurement;
  output Real row[BarometerMeasurementLength];
algorithm
  row := {measurement.timestamp_s,
    measurement.altitudeWorldEnu_m,
    measurement.variance_m2};
end packBarometer;
