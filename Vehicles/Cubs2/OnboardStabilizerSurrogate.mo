within Vehicles.Cubs2;

block OnboardStabilizerSurrogate
  "Behavioral surrogate for the proprietary onboard stabilizer"
  import Components = Vehicles.Cubs2.OuterLoopComponents;

  parameter Real phi_sp_max = 0.90 "Bank setpoint limit [rad]";
  parameter Real theta_sp_max = 0.45 "Pitch setpoint limit [rad]";
  parameter Real yaw_rate_max = 1.0 "Full-stick commanded yaw rate [rad/s]";
  parameter Real Kp_phi = 5.0 "Bank-error to roll-rate gain [1/s]";
  parameter Real Kp_theta = 5.0 "Pitch-error to pitch-rate gain [1/s]";
  parameter Real p_rate_max = 4.0 "Roll-rate setpoint limit [rad/s]";
  parameter Real q_rate_max = 2.5 "Pitch-rate setpoint limit [rad/s]";
  parameter Real rateGain[3] = {0.45, 0.55, 0.40}
    "proportional rate-error to surface-command gains";
  parameter Real v_prot_lo = 2.6 "Airspeed below which nose-up authority fades [m/s]";
  parameter Real v_prot_hi = 3.6 "Airspeed for full nose-up authority [m/s]";
  parameter Real dive_slope = 0.06 "Protective dive slope [rad/(m/s)]";
  parameter Real protectionBlendSpeed(unit="m/s") = 0.2
    "Speed interval over which the protective dive command blends in";

  input Real pilotCommand[4](start = {0.0, 0.0, 0.0, 0.0})
    "{roll, pitch, yaw, throttle} normalized commands";
  input Real armed(start = 0) "Arm signal [0, 1]";
  input Real gyro[3](start = {0, 0, 0}) "Body rate FLU [rad/s]";
  input Real up_body[3](start = {0, 0, 1}) "World up in body FLU";
  input Real airspeed(start = 15) "True airspeed [m/s]";

  output Real surfaceCommand[4]
    "{aileron, elevator, rudder, throttle} normalized commands";
  output Real attitudeCommand_rad[2] "{roll, pitch} protected setpoints";

protected
  Real tiltRateSetpoint_rad_s[2] "{roll, nose-up} geometric rate demand";
  Real rateSetpoint_rad_s[3] "{roll, nose-up, yaw} rate demand";
  Real rateMeasurement_rad_s[3] "{roll, nose-up, yaw} measured rate";
  Real rateError_rad_s[3] "{roll, nose-up, yaw} rate error";
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
    pilotCommand[1] * phi_sp_max, -phi_sp_max, phi_sp_max);
  theta_sp = armFactor * MathUtilities.clip(
    theta_sp_max
      * (min(pilotCommand[2], 0.0)
        + max(pilotCommand[2], 0.0) * climb_auth),
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
  rateSetpoint_rad_s = {
    tiltRateSetpoint_rad_s[1],
    tiltRateSetpoint_rad_s[2],
    pilotCommand[3] * yaw_rate_max};
  rateMeasurement_rad_s = {gyro[1], -gyro[2], gyro[3]};
  rateError_rad_s = rateSetpoint_rad_s - rateMeasurement_rad_s;

  surfaceCommand = armFactor * {
    MathUtilities.clip(rateGain[1] * rateError_rad_s[1], -1.0, 1.0),
    MathUtilities.clip(rateGain[2] * rateError_rad_s[2], -1.0, 1.0),
    MathUtilities.clip(rateGain[3] * rateError_rad_s[3], -1.0, 1.0),
    MathUtilities.clip(pilotCommand[4], 0.0, 1.0)};
  attitudeCommand_rad = {phi_sp, theta_sp};
  annotation(Documentation(info = "<html>
    <p>This block is an intentionally approximate simulation model. The real
    CUBS2 aircraft uses a proprietary onboard stabilization controller whose
    equations are unavailable to this project.</p>
    <p>The block exists only to close the plant loop when testing the deployable
    <code>Vehicles.Cubs2.OuterLoop</code>. Its continuous proportional response
    deliberately avoids inventing unobservable modes, integrators, or sample
    timing for the proprietary implementation. It is not generated as flight
    code, and test results using it do not establish equivalence to the
    proprietary controller.</p>
  </html>"));
end OnboardStabilizerSurrogate;
