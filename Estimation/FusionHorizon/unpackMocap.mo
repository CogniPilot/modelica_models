within Estimation.FusionHorizon;

function unpackMocap "Expand one mocap queue row into connector fields"
  input Real row[MocapMeasurementLength];
  output Real timestamp_s;
  output Real positionWorldEnu_m[3];
  output Real quaternionWorldBody[4];
  output Real positionCovarianceWorld_m2[3, 3];
  output Real attitudeCovarianceBody_rad2[3, 3];
algorithm
  // Separate outputs rather than one Avionics.MocapSample, and the caller
  // assigns the connector field by field. That is how every estimator and
  // every horizon block in this library publishes: a whole record assigned to
  // a connector is not something OpenModelica generates code for, and where
  // the connector is aliased it silently publishes zeros instead.
  timestamp_s := row[1];
  positionWorldEnu_m := row[2:4];
  quaternionWorldBody := row[5:8];
  positionCovarianceWorld_m2 :=
    [row[9], row[10], row[11];
     row[12], row[13], row[14];
     row[15], row[16], row[17]];
  attitudeCovarianceBody_rad2 :=
    [row[18], row[19], row[20];
     row[21], row[22], row[23];
     row[24], row[25], row[26]];
end unpackMocap;
