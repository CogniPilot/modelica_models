within Vehicles.Cubs2;
model InnerLoop "Cascaded attitude-command fly-by-wire controller"
  parameter Real phi_sp_max = 0.90 "Bank setpoint limit [rad]";
  parameter Real theta_sp_max = 0.45 "Pitch setpoint limit [rad]";
  parameter Real yaw_rate_max = 1.0 "Full-stick commanded yaw rate [rad/s]";
  parameter Real Kp_phi = 5.0 "Bank-error to roll-rate gain [1/s]";
  parameter Real Kp_theta = 5.0 "Pitch-error to pitch-rate gain [1/s]";
  parameter Real p_rate_max = 4.0 "Roll-rate setpoint limit [rad/s]";
  parameter Real q_rate_max = 2.5 "Pitch-rate setpoint limit [rad/s]";
  parameter Real Kp_p = 0.45 "Roll-rate proportional gain";
  parameter Real Ki_p = 0.30 "Roll-rate integral gain";
  parameter Real ilim_p = 1.0 "Roll integrator limit";
  parameter Real Kp_q = 0.55 "Pitch-rate proportional gain";
  parameter Real Ki_q = 0.40 "Pitch-rate integral gain";
  parameter Real ilim_q = 1.0 "Pitch integrator limit";
  parameter Real Kp_r = 0.40 "Yaw-rate proportional gain";
  parameter Real Ki_r = 0.10 "Yaw-rate integral gain";
  parameter Real ilim_r = 0.6 "Yaw integrator limit";
  parameter Real v_prot_lo = 2.6 "Airspeed below which nose-up authority fades [m/s]";
  parameter Real v_prot_hi = 3.6 "Airspeed for full nose-up authority [m/s]";
  parameter Real dive_slope = 0.06 "Protective dive slope [rad/(m/s)]";

  input Real stick_roll(start = 0) "Roll stick [-1, 1]";
  input Real stick_pitch(start = 0) "Pitch stick [-1, 1]";
  input Real stick_yaw(start = 0) "Yaw stick [-1, 1]";
  input Real stick_throttle(start = 0) "Throttle stick [0, 1]";
  input Real armed(start = 0) "Arm signal [0, 1]";
  input Real gyro[3](start = {0, 0, 0}) "Body rate FLU [rad/s]";
  input Real up_body[3](start = {0, 0, 1}) "World up in body FLU";
  input Real airspeed(start = 15) "True airspeed [m/s]";

  output Real ail "Aileron command [-1, 1]";
  output Real elev "Elevator command [-1, 1]";
  output Real rud "Rudder command [-1, 1]";
  output Real thr "Throttle command [0, 1]";

protected
  Real phi "Estimated bank angle [rad]";
  Real theta "Estimated pitch angle [rad]";
  Real p_sp "Roll-rate setpoint [rad/s]";
  Real q_up_sp "Nose-up-rate setpoint [rad/s]";
  Real r_sp "Yaw-rate setpoint [rad/s]";
  Real p_meas "Measured roll rate [rad/s]";
  Real q_up "Measured nose-up rate [rad/s]";
  Real r_meas "Measured yaw rate [rad/s]";
  Real e_p "Roll-rate error [rad/s]";
  Real e_q "Pitch-rate error [rad/s]";
  Real e_r "Yaw-rate error [rad/s]";
  Real i_p(start = 0, fixed = true) "Roll integral";
  Real i_q(start = 0, fixed = true) "Pitch integral";
  Real i_r(start = 0, fixed = true) "Yaw integral";
  Real i_p_c "Limited roll integral";
  Real i_q_c "Limited pitch integral";
  Real i_r_c "Limited yaw integral";
  Real phi_sp "Commanded bank [rad]";
  Real theta_sp "Commanded pitch [rad]";
  Real climb_auth "Nose-up authority [0, 1]";
  Real theta_eff "Protected pitch setpoint [rad]";
  Real armFactor "Bounded arm command [0, 1]";
  Real ailRaw "Unsaturated aileron command";
  Real elevRaw "Unsaturated elevator command";
  Real rudRaw "Unsaturated rudder command";
  Real rollIntegralDrive "Roll integrator input";
  Real pitchIntegralDrive "Pitch integrator input";
  Real yawIntegralDrive "Yaw integrator input";

equation
  assert(v_prot_hi > v_prot_lo,
    "The high airspeed-protection threshold must exceed the low threshold");
  armFactor = MathUtilities.clip(armed, 0.0, 1.0);
  phi = atan2(up_body[2], up_body[3]);
  theta = atan2(up_body[1], up_body[3]);

  climb_auth = MathUtilities.clip(
    (airspeed - v_prot_lo) / (v_prot_hi - v_prot_lo), 0.0, 1.0);
  phi_sp = armFactor * MathUtilities.clip(
    stick_roll * phi_sp_max, -phi_sp_max, phi_sp_max);
  theta_sp = armFactor * MathUtilities.clip(
    noEvent(if stick_pitch > 0 then
      stick_pitch * theta_sp_max * climb_auth
    else
      stick_pitch * theta_sp_max),
    -theta_sp_max,
    theta_sp_max);
  theta_eff = noEvent(if airspeed < v_prot_lo then
    min(theta_sp, -dive_slope * (v_prot_lo - airspeed))
  else
    theta_sp);

  p_sp = MathUtilities.clip(Kp_phi * (phi_sp - phi), -p_rate_max, p_rate_max);
  q_up_sp = MathUtilities.clip(
    Kp_theta * (theta_eff - theta), -q_rate_max, q_rate_max);
  r_sp = stick_yaw * yaw_rate_max;

  p_meas = gyro[1];
  q_up = -gyro[2];
  r_meas = gyro[3];
  e_p = p_sp - p_meas;
  e_q = q_up_sp - q_up;
  e_r = r_sp - r_meas;

  i_p_c = MathUtilities.clip(i_p, -ilim_p, ilim_p);
  i_q_c = MathUtilities.clip(i_q, -ilim_q, ilim_q);
  i_r_c = MathUtilities.clip(i_r, -ilim_r, ilim_r);
  ailRaw = Kp_p * e_p + Ki_p * i_p_c;
  elevRaw = Kp_q * e_q + Ki_q * i_q_c;
  rudRaw = Kp_r * e_r + Ki_r * i_r_c;
  rollIntegralDrive = armFactor * e_p;
  pitchIntegralDrive = armFactor * e_q;
  yawIntegralDrive = armFactor * e_r;

  der(i_p) = noEvent(if
      (i_p >= ilim_p and rollIntegralDrive > 0.0)
      or (i_p <= -ilim_p and rollIntegralDrive < 0.0)
      or (ailRaw >= 1.0 and rollIntegralDrive > 0.0)
      or (ailRaw <= -1.0 and rollIntegralDrive < 0.0) then
      0.0
    else
      rollIntegralDrive);
  der(i_q) = noEvent(if
      (i_q >= ilim_q and pitchIntegralDrive > 0.0)
      or (i_q <= -ilim_q and pitchIntegralDrive < 0.0)
      or (elevRaw >= 1.0 and pitchIntegralDrive > 0.0)
      or (elevRaw <= -1.0 and pitchIntegralDrive < 0.0) then
      0.0
    else
      pitchIntegralDrive);
  der(i_r) = noEvent(if
      (i_r >= ilim_r and yawIntegralDrive > 0.0)
      or (i_r <= -ilim_r and yawIntegralDrive < 0.0)
      or (rudRaw >= 1.0 and yawIntegralDrive > 0.0)
      or (rudRaw <= -1.0 and yawIntegralDrive < 0.0) then
      0.0
    else
      yawIntegralDrive);

  ail = armFactor * MathUtilities.clip(ailRaw, -1.0, 1.0);
  elev = armFactor * MathUtilities.clip(elevRaw, -1.0, 1.0);
  rud = armFactor * MathUtilities.clip(rudRaw, -1.0, 1.0);
  thr = armFactor * MathUtilities.clip(stick_throttle, 0.0, 1.0);
end InnerLoop;
