within Vehicles.Cubs2;
block InnerLoop "Cascaded attitude-command fly-by-wire controller"
  import Components = Vehicles.Cubs2.OuterLoopComponents;

  parameter Real samplePeriod(unit="s") = 0.005;
  parameter Real phi_sp_max = 0.90 "Bank setpoint limit [rad]";
  parameter Real theta_sp_max = 0.45 "Pitch setpoint limit [rad]";
  parameter Real yaw_rate_max = 1.0 "Full-stick commanded yaw rate [rad/s]";
  parameter Real Kp_phi = 5.0 "Bank-error to roll-rate gain [1/s]";
  parameter Real Kp_theta = 5.0 "Pitch-error to pitch-rate gain [1/s]";
  parameter Real p_rate_max = 4.0 "Roll-rate setpoint limit [rad/s]";
  parameter Real q_rate_max = 2.5 "Pitch-rate setpoint limit [rad/s]";
  parameter Control.PidParameters rollRatePid = Control.PidParameters(
    samplePeriod = samplePeriod,
    kp = 0.45,
    ki = 0.30,
    integralLimit = 1.0);
  parameter Control.PidParameters pitchRatePid = Control.PidParameters(
    samplePeriod = samplePeriod,
    kp = 0.55,
    ki = 0.40,
    integralLimit = 1.0);
  parameter Control.PidParameters yawRatePid = Control.PidParameters(
    samplePeriod = samplePeriod,
    kp = 0.40,
    ki = 0.10,
    integralLimit = 0.6);
  parameter Real v_prot_lo = 2.6 "Airspeed below which nose-up authority fades [m/s]";
  parameter Real v_prot_hi = 3.6 "Airspeed for full nose-up authority [m/s]";
  parameter Real dive_slope = 0.06 "Protective dive slope [rad/(m/s)]";
  parameter Real protectionBlendSpeed(unit="m/s") = 0.2
    "Speed interval over which the protective dive command blends in";

  input Real stick[4](start = {0.0, 0.0, 0.0, 0.0})
    "{roll, pitch, yaw, throttle} normalized commands";
  input Real armed(start = 0) "Arm signal [0, 1]";
  input Real gyro[3](start = {0, 0, 0}) "Body rate FLU [rad/s]";
  input Real up_body[3](start = {0, 0, 1}) "World up in body FLU";
  input Real airspeed(start = 15) "True airspeed [m/s]";

  output Real actuator[4]
    "{aileron, elevator, rudder, throttle} normalized commands";
  output Real attitudeCommand_rad[2] "{roll, pitch} protected setpoints";

  Control.PidController rollRateController(params = rollRatePid);
  Control.PidController pitchRateController(params = pitchRatePid);
  Control.PidController yawRateController(params = yawRatePid);

protected
  Real tiltRateSetpoint_rad_s[2] "{roll, nose-up} geometric rate demand";
  Real p_sp "Roll-rate setpoint [rad/s]";
  Real q_up_sp "Nose-up-rate setpoint [rad/s]";
  Real r_sp "Yaw-rate setpoint [rad/s]";
  Real p_meas "Measured roll rate [rad/s]";
  Real q_up "Measured nose-up rate [rad/s]";
  Real r_meas "Measured yaw rate [rad/s]";
  Real phi_sp "Commanded bank [rad]";
  Real theta_sp "Commanded pitch [rad]";
  Real climb_auth "Nose-up authority [0, 1]";
  Real lowSpeedProtectionWeight "Continuous protective-command blend [0, 1]";
  Real protectiveDiveCommand "Nose-down command at low airspeed [rad]";
  Real theta_eff "Protected pitch setpoint [rad]";
  Real armFactor "Bounded arm command [0, 1]";

equation
  assert(v_prot_hi > v_prot_lo,
    "The high airspeed-protection threshold must exceed the low threshold");
  assert(protectionBlendSpeed > 0.0,
    "The airspeed-protection blend interval must be positive");
  armFactor = MathUtilities.clip(armed, 0.0, 1.0);

  climb_auth = MathUtilities.clip(
    (airspeed - v_prot_lo) / (v_prot_hi - v_prot_lo), 0.0, 1.0);
  phi_sp = armFactor * MathUtilities.clip(
    stick[1] * phi_sp_max, -phi_sp_max, phi_sp_max);
  theta_sp = armFactor * MathUtilities.clip(
    theta_sp_max
      * (min(stick[2], 0.0) + max(stick[2], 0.0) * climb_auth),
    -theta_sp_max,
    theta_sp_max);
  lowSpeedProtectionWeight = MathUtilities.clip(
    (v_prot_lo - airspeed) / protectionBlendSpeed, 0.0, 1.0);
  protectiveDiveCommand = -dive_slope * max(v_prot_lo - airspeed, 0.0);
  theta_eff = theta_sp + lowSpeedProtectionWeight
    * (min(theta_sp, protectiveDiveCommand) - theta_sp);

  tiltRateSetpoint_rad_s = Control.tiltRateSetpoint(
    {phi_sp, theta_eff},
    up_body,
    {Kp_phi, Kp_theta},
    {p_rate_max, q_rate_max});
  p_sp = tiltRateSetpoint_rad_s[1];
  q_up_sp = tiltRateSetpoint_rad_s[2];
  r_sp = stick[3] * yaw_rate_max;

  p_meas = gyro[1];
  q_up = -gyro[2];
  r_meas = gyro[3];
  rollRateController.setpoint = p_sp;
  rollRateController.measurement = p_meas;

  pitchRateController.setpoint = q_up_sp;
  pitchRateController.measurement = q_up;

  yawRateController.setpoint = r_sp;
  yawRateController.measurement = r_meas;

  actuator = armFactor * {
    rollRateController.command,
    pitchRateController.command,
    yawRateController.command,
    MathUtilities.clip(stick[4], 0.0, 1.0)};
  attitudeCommand_rad = {phi_sp, theta_sp};
end InnerLoop;
