within Estimation.FusionHorizon;

function packOpticalFlow "Flatten one optical-flow sample into a queue row"
  input Avionics.OpticalFlowSample measurement;
  output Real row[OpticalFlowMeasurementLength];
algorithm
  // integrationTime_s is carried like any other payload field and is NOT the
  // queue's own span. The queue delays a measurement; it does not change what
  // the measurement integrated over, and a flow sample fused at the horizon
  // still covers the exposure it was taken across.
  row := cat(1,
    {measurement.timestamp_s},
    measurement.integratedLineOfSight_rad,
    measurement.integratedLineOfSightCovariance_rad2[1, :],
    measurement.integratedLineOfSightCovariance_rad2[2, :],
    measurement.integratedGyroscopeBodyFlu_rad,
    measurement.integratedGyroscopeCovariance_rad2[1, :],
    measurement.integratedGyroscopeCovariance_rad2[2, :],
    measurement.integratedGyroscopeCovariance_rad2[3, :],
    {measurement.integrationTime_s,
     measurement.groundDistance_m,
     measurement.groundDistanceVariance_m2,
     measurement.quality});
end packOpticalFlow;
