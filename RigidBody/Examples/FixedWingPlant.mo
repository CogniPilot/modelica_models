within RigidBody.Examples;

model FixedWingPlant
  extends RigidBody.RigidBody6DOF(
    mass = 5.5,
    g = 9.8,
    ixx = 0.35,
    iyy = 0.80,
    izz = 1.10,
    p_start = {0, 0, 35},
    v_b_start = {16, 0, 0},
    qnorm_gain = 1.0
  );

  parameter Real rho = 1.225 "Air density [kg/m^3]";
  parameter Real S = 0.55 "Wing reference area [m^2]";
  parameter Real b = 2.1 "Wing span [m]";
  parameter Real c = 0.28 "Mean aerodynamic chord [m]";
  parameter Real thrust_max = 42.0 "Maximum propeller thrust [N]";
  parameter Real CL0 = 0.32 "Lift coefficient at zero alpha";
  parameter Real CL_alpha = 4.2 "Lift slope [1/rad]";
  parameter Real CL_elevator = 0.55 "Lift coefficient per elevator command";
  parameter Real CD0 = 0.045 "Zero-lift drag coefficient";
  parameter Real CDi = 0.055 "Induced drag factor";
  parameter Real Cm0 = 0.02 "Pitch trim moment coefficient";
  parameter Real Cm_alpha = -0.55 "Pitch static stability coefficient";
  parameter Real Cm_elevator = -0.85 "Pitch moment per elevator command";
  parameter Real Cl_aileron = 0.28 "Roll moment per aileron command";
  parameter Real Cn_rudder = 0.20 "Yaw moment per rudder command";
  parameter Real roll_damping = 0.70 "Roll-rate damping [N*m/(rad/s)]";
  parameter Real pitch_damping = 1.20 "Pitch-rate damping [N*m/(rad/s)]";
  parameter Real yaw_damping = 0.80 "Yaw-rate damping [N*m/(rad/s)]";
  parameter Real ground_k = 1200.0 "Soft ground stiffness [N/m]";
  parameter Real ground_c = 70.0 "Soft ground damping [N*s/m]";
  parameter Real mag_world_ned[3] = {0.21, 0.0, 0.45} "Mag field NED [Gauss]";
  parameter Real R_FRD_FLU[3, 3] = [
    1, 0, 0;
    0, -1, 0;
    0, 0, -1
  ] "Body FLU to body FRD transform";
  parameter Real R_NWU_NED[3, 3] = [
    1, 0, 0;
    0, -1, 0;
    0, 0, -1
  ] "World NED to internal N/W/U transform";

  input Real aileron(start = 0.0) "Normalized aileron command";
  input Real elevator(start = 0.0) "Normalized elevator command";
  input Real throttle(start = 0.0) "Normalized throttle command";
  input Real rudder(start = 0.0) "Normalized rudder command";

  output Real prop_omega(start = 0.0) "Propeller angular rate proxy [rad/s]";
  output Real accel[3] "Body FRD accelerometer [m/s^2] (specific force)";
  output Real gyro[3] "Body FRD gyroscope [rad/s]";
  output Real mag[3] "Body FRD magnetometer [Gauss]";

protected
  Real V;
  Real qbar;
  Real alpha;
  Real beta;
  Real CL;
  Real CD;
  Real lift;
  Real drag;
  Real side_force;
  Real thrust;
  Real ground_force_w[3];
  Real ground_force_b[3];
  Real mag_world_w[3];
  Real mag_b_flu[3];

equation
  V = sqrt(v_b[1] * v_b[1] + v_b[2] * v_b[2] + v_b[3] * v_b[3] + 1e-6);
  qbar = 0.5 * rho * V * V;
  alpha = atan2(v_b[3], sqrt(v_b[1] * v_b[1] + 0.01));
  beta = atan2(v_b[2], sqrt(v_b[1] * v_b[1] + v_b[3] * v_b[3] + 0.01));

  CL = CL0 + CL_alpha * alpha + CL_elevator * elevator;
  CD = CD0 + CDi * CL * CL;
  lift = qbar * S * CL;
  drag = qbar * S * CD;
  side_force = -qbar * S * 1.2 * beta;
  thrust = thrust_max * min(1.0, max(0.0, throttle));

  ground_force_w = if p[3] < 0.0
                   then {0.0, 0.0, ground_k * (0.0 - p[3]) - ground_c * v_w[3]}
                   else {0.0, 0.0, 0.0};
  ground_force_b = transpose(R) * ground_force_w;

  F_b = {
    thrust - drag,
    side_force,
    lift
  } + ground_force_b;

  M_b = {
    qbar * S * b * Cl_aileron * aileron - roll_damping * omega[1],
    qbar * S * c * (Cm0 + Cm_alpha * alpha + Cm_elevator * elevator) - pitch_damping * omega[2],
    qbar * S * b * Cn_rudder * rudder - yaw_damping * omega[3]
  };

  prop_omega = 650.0 * min(1.0, max(0.0, throttle));
  accel = R_FRD_FLU * a_b;
  gyro = R_FRD_FLU * omega;
  mag_world_w = R_NWU_NED * mag_world_ned;
  mag_b_flu = transpose(R) * mag_world_w;
  mag = R_FRD_FLU * mag_b_flu;
end FixedWingPlant;
