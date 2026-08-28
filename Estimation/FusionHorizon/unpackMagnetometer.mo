within Estimation.FusionHorizon;

function unpackMagnetometer
  "Expand one magnetometer queue row into connector fields"
  input Real row[MagnetometerMeasurementLength];
  output Real timestamp_s;
  output Real magneticFieldBodyFlu_T[3];
  output Real covarianceBody_T2[3, 3];
algorithm
  timestamp_s := row[1];
  magneticFieldBodyFlu_T := row[2:4];
  covarianceBody_T2 :=
    [row[5], row[6], row[7];
     row[8], row[9], row[10];
     row[11], row[12], row[13]];
end unpackMagnetometer;
