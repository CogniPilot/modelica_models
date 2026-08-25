within Estimation.StrapdownINS.ESKF;

function navigationEstimate
  "Publish a canonical estimate with consistent attitude representations"
  input Estimation.StrapdownINS.ESKF.State state;
  input Avionics.ImuSample imu;
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
  Real eulerB321_rad[3];
  Real correctedAngularVelocityBody[3];
  Real correctedSpecificForceBody[3];
algorithm
  rotationWorldBody := LieGroups.SO3.Quat.to_DCM(
    state.quaternionWorldBody);
  eulerB321_rad := LieGroups.SO3.EulerB321.from_Quat(
    state.quaternionWorldBody);
  correctedAngularVelocityBody := imu.angularVelocityBodyFlu_rad_s
    - state.gyroscopeBiasBodyFlu_rad_s;
  correctedSpecificForceBody := imu.specificForceBodyFlu_m_s2
    - state.accelerometerBiasBodyFlu_m_s2;
  valid := validInput;
  timestamp_s := imu.timestamp_s;
  positionWorldEnu_m := state.positionWorldEnu_m;
  velocityWorldEnu_m_s := state.velocityWorldEnu_m_s;
  accelerationWorldEnu_m_s2 := rotationWorldBody
    * correctedSpecificForceBody + gravityWorldEnu_m_s2;
  quaternionWorldBody := state.quaternionWorldBody;
  eulerRpy_rad := {
    eulerB321_rad[3], eulerB321_rad[2], eulerB321_rad[1]};
  angularVelocityBodyFlu_rad_s := correctedAngularVelocityBody;
  angularVelocityWorldEnu_rad_s :=
    rotationWorldBody * correctedAngularVelocityBody;
end navigationEstimate;
