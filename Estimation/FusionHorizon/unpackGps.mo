within Estimation.FusionHorizon;

function unpackGps "Expand one GPS queue row into connector fields"
  input Real row[GpsMeasurementLength];
  output Real timestamp_s;
  output Boolean positionValid;
  output Boolean velocityValid;
  output Real geodetic_deg_m[3];
  output Real positionWorldEnu_m[3];
  output Real velocityWorldEnu_m_s[3];
  output Real positionCovarianceWorld_m2[3, 3];
  output Real velocityCovarianceWorld_m2_s2[3, 3];
algorithm
  timestamp_s := row[1];
  // Recovered by comparison rather than by equality with one, because the row
  // came back out of a masked multiply-add walk: the stored value is exactly
  // one or exactly zero in exact arithmetic and a half-way test says the same
  // thing without depending on that.
  positionValid := row[2] > 0.5;
  velocityValid := row[3] > 0.5;
  geodetic_deg_m := row[4:6];
  positionWorldEnu_m := row[7:9];
  velocityWorldEnu_m_s := row[10:12];
  positionCovarianceWorld_m2 :=
    [row[13], row[14], row[15];
     row[16], row[17], row[18];
     row[19], row[20], row[21]];
  velocityCovarianceWorld_m2_s2 :=
    [row[22], row[23], row[24];
     row[25], row[26], row[27];
     row[28], row[29], row[30]];
end unpackGps;
