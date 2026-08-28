within Estimation.FusionHorizon;

function unpackOpticalFlow
  "Expand one optical-flow queue row into connector fields"
  input Real row[OpticalFlowMeasurementLength];
  output Real timestamp_s;
  output Real integratedLineOfSight_rad[2];
  output Real integratedLineOfSightCovariance_rad2[2, 2];
  output Real integratedGyroscopeBodyFlu_rad[3];
  output Real integratedGyroscopeCovariance_rad2[3, 3];
  output Real integrationTime_s;
  output Real groundDistance_m;
  output Real groundDistanceVariance_m2;
  output Real quality;
algorithm
  timestamp_s := row[1];
  integratedLineOfSight_rad := row[2:3];
  integratedLineOfSightCovariance_rad2 :=
    [row[4], row[5];
     row[6], row[7]];
  integratedGyroscopeBodyFlu_rad := row[8:10];
  integratedGyroscopeCovariance_rad2 :=
    [row[11], row[12], row[13];
     row[14], row[15], row[16];
     row[17], row[18], row[19]];
  integrationTime_s := row[20];
  groundDistance_m := row[21];
  groundDistanceVariance_m2 := row[22];
  quality := row[23];
end unpackOpticalFlow;
