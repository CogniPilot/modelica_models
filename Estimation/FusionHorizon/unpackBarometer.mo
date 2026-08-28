within Estimation.FusionHorizon;

function unpackBarometer
  "Expand one barometer queue row into connector fields"
  input Real row[BarometerMeasurementLength];
  output Real timestamp_s;
  output Real altitudeWorldEnu_m;
  output Real variance_m2;
algorithm
  timestamp_s := row[1];
  altitudeWorldEnu_m := row[2];
  variance_m2 := row[3];
end unpackBarometer;
