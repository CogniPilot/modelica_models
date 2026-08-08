within Estimation.MultiSensorInvariant;

function navigationEstimate
  "Publish a canonical estimate with consistent attitude representations"
  input Estimation.MultiSensorInvariant.State state;
  input Avionics.ImuSample imu;
  input Real gravityWorldEnu_m_s2[3];
  input Boolean valid;
  output Avionics.NavigationEstimate estimate;
protected
  Real rotationWorldBody[3, 3];
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
  estimate := Avionics.NavigationEstimate(
    valid=valid,
    timestamp_s=imu.timestamp_s,
    positionWorldEnu_m=state.positionWorldEnu_m,
    velocityWorldEnu_m_s=state.velocityWorldEnu_m_s,
    accelerationWorldEnu_m_s2=
      rotationWorldBody * correctedSpecificForceBody + gravityWorldEnu_m_s2,
    quaternionWorldBody=state.quaternionWorldBody,
    rotationWorldBody=rotationWorldBody,
    eulerRpy_rad={eulerB321_rad[3], eulerB321_rad[2], eulerB321_rad[1]},
    angularVelocityBodyFlu_rad_s=correctedAngularVelocityBody,
    angularVelocityWorldEnu_rad_s=
      rotationWorldBody * correctedAngularVelocityBody);
end navigationEstimate;
