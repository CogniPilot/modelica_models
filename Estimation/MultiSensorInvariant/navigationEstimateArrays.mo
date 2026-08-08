within Estimation.MultiSensorInvariant;

function navigationEstimateArrays
  "Flatten the canonical navigation record for sampled external boundaries"
  input Real position[3];
  input Real velocity[3];
  input Real quaternion[4];
  input Real gyroscopeBias[3];
  input Real accelerometerBias[3];
  input Estimation.MultiSensorInvariant.Covariance covariance;
  input Boolean imuValid;
  input Boolean imuFresh;
  input Real imuTimestamp_s;
  input Real imuAngularVelocity[3];
  input Real imuSpecificForce[3];
  input Real gravityWorldEnu_m_s2[3];
  input Boolean validInput;
  output Boolean valid;
  output Real timestamp_s;
  output Real positionWorldEnu_m[3];
  output Real velocityWorldEnu_m_s[3];
  output Real accelerationWorldEnu_m_s2[3];
  output Real quaternionWorldBody[4];
  output Real rotationWorldBody[3, 3];
  output Real eulerRpy_rad[3];
  output Real angularVelocityBodyFlu_rad_s[3];
  output Real angularVelocityWorldEnu_rad_s[3];
protected
  Avionics.NavigationEstimate navigation;
  Avionics.ImuSample imu;
algorithm
  imu := Avionics.ImuSample(
    valid=imuValid,
    fresh=imuFresh,
    timestamp_s=imuTimestamp_s,
    angularVelocityBodyFlu_rad_s=imuAngularVelocity,
    specificForceBodyFlu_m_s2=imuSpecificForce);
  navigation := navigationEstimate(
    State(
      positionWorldEnu_m=position,
      velocityWorldEnu_m_s=velocity,
      quaternionWorldBody=quaternion,
      gyroscopeBiasBodyFlu_rad_s=gyroscopeBias,
      accelerometerBiasBodyFlu_m_s2=accelerometerBias,
      covariance=covariance),
    imu,
    gravityWorldEnu_m_s2,
    validInput);
  valid := navigation.valid;
  timestamp_s := navigation.timestamp_s;
  positionWorldEnu_m := navigation.positionWorldEnu_m;
  velocityWorldEnu_m_s := navigation.velocityWorldEnu_m_s;
  accelerationWorldEnu_m_s2 := navigation.accelerationWorldEnu_m_s2;
  quaternionWorldBody := navigation.quaternionWorldBody;
  rotationWorldBody := navigation.rotationWorldBody;
  eulerRpy_rad := navigation.eulerRpy_rad;
  angularVelocityBodyFlu_rad_s :=
    navigation.angularVelocityBodyFlu_rad_s;
  angularVelocityWorldEnu_rad_s :=
    navigation.angularVelocityWorldEnu_rad_s;
end navigationEstimateArrays;
