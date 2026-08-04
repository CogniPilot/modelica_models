within Vehicles.Rdd2;
model PlantAdapter
  "Firmware-facing command and sensor adapter around the RDD2 physical plant"
  parameter Real omegaMax(unit = "rad/s") = 1100.0;
  Plant dynamics;

  input Real motorCommand[4](each start = 0.0)
    "normalized commands in motor order [front right, rear left, front left, rear right]";

  output Real time_s;
  output Real position_m[3] "world-frame position ENU [x, y, z]";
  output Real velocity_m_s[3] "world-frame velocity ENU [x, y, z]";
  output Real quaternion[4] "world-to-body quaternion [w, x, y, z]";
  output Real euler_rad[3] "attitude [roll, pitch, yaw]";
  output Real gyro_rad_s[3] "body angular rate [x, y, z]";
  output Real acceleration_m_s2[3] "specific force in body axes [x, y, z]";
  output Real motorOmega_rad_s[4] "motor angular speeds in motorCommand order";

protected
  Real eulerB321_rad[3] "Lie-group B321 coordinates [yaw, pitch, roll]";

equation
  dynamics.omega_cmd = omegaMax * {
    MathUtilities.clip(motorCommand[motorIndex], 0.0, 1.0)
      for motorIndex in 1:4};
  time_s = time;
  position_m = dynamics.position;
  velocity_m_s = dynamics.velocity;
  quaternion = dynamics.quat;
  eulerB321_rad = LieGroups.SO3.EulerB321.from_Quat(quaternion);
  euler_rad = {eulerB321_rad[3], eulerB321_rad[2], eulerB321_rad[1]};
  gyro_rad_s = dynamics.omega;
  acceleration_m_s2 = dynamics.a_b;
  motorOmega_rad_s = dynamics.omega_m;
end PlantAdapter;
