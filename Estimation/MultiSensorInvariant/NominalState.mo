within Estimation.MultiSensorInvariant;

record NominalState "Nominal navigation state"
  Real positionWorldEnu_m[3];
  Real velocityWorldEnu_m_s[3];
  Real quaternionWorldBody[4];
  Real gyroscopeBiasBodyFlu_rad_s[3];
  Real accelerometerBiasBodyFlu_m_s2[3];
end NominalState;
