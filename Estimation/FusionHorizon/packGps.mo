within Estimation.FusionHorizon;

function packGps "Flatten one GPS sample into a queue row"
  input Avionics.GpsSample measurement;
  output Real row[GpsMeasurementLength];
algorithm
  // positionValid and velocityValid ARE carried, as one and zero. Unlike
  // valid and fresh they are not statements about delivery, they say which
  // half of the fix the solution actually contains, and the filter's
  // correction dispatch branches on them: dropping them would offer a
  // position-only fix as though it carried a velocity.
  row := cat(1,
    {measurement.timestamp_s,
     if measurement.positionValid then 1.0 else 0.0,
     if measurement.velocityValid then 1.0 else 0.0},
    measurement.geodetic_deg_m,
    measurement.positionWorldEnu_m,
    measurement.velocityWorldEnu_m_s,
    measurement.positionCovarianceWorld_m2[1, :],
    measurement.positionCovarianceWorld_m2[2, :],
    measurement.positionCovarianceWorld_m2[3, :],
    measurement.velocityCovarianceWorld_m2_s2[1, :],
    measurement.velocityCovarianceWorld_m2_s2[2, :],
    measurement.velocityCovarianceWorld_m2_s2[3, :]);
end packGps;
