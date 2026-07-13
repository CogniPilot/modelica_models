within RigidBody;
partial model RigidBody6DOF "Six-degree-of-freedom rigid body"
  parameter Real mass = 1.0 "Mass [kg]";
  parameter Real g = 9.8 "Gravity [m/s^2]";
  parameter Real ixx = 1.0 "Body inertia matrix xx entry [kg*m^2]";
  parameter Real iyy = 1.0 "Body inertia matrix yy entry [kg*m^2]";
  parameter Real izz = 1.0 "Body inertia matrix zz entry [kg*m^2]";
  parameter Real ixy = 0.0 "Body inertia matrix xy entry [kg*m^2]";
  parameter Real ixz = 0.0 "Body inertia matrix xz entry [kg*m^2]";
  parameter Real iyz = 0.0 "Body inertia matrix yz entry [kg*m^2]";
  parameter Real J[3, 3] = [
    ixx, ixy, ixz;
    ixy, iyy, iyz;
    ixz, iyz, izz
  ] "Body inertia matrix [kg*m^2]";
  parameter Real p_start[3] = {0, 0, 0} "Initial world position";
  parameter Real v_b_start[3] = {0, 0, 0} "Initial body velocity";
  parameter Real q_start[4] = {1, 0, 0, 0} "Initial quaternion w,x,y,z";
  parameter Real omega_start[3] = {0, 0, 0} "Initial body angular velocity";
  parameter Real qnorm_gain = 1.0 "Quaternion renormalization gain";

  Real F_b[3] "Total non-gravity force in body frame [N]";
  Real M_b[3] "Total moment in body frame [N*m]";
  output Real p[3](start = p_start, each fixed = true) "World position [m]";
  output Real v_b[3](start = v_b_start, each fixed = true) "Body velocity [m/s]";
  output Real q[4](start = q_start, each fixed = true) "Quaternion w,x,y,z";
  output Real omega[3](start = omega_start, each fixed = true)
    "Body angular velocity [rad/s]";
  output Real R[3, 3](start = identity(3))
    "Direction cosine matrix, body to world";
  output Real v_w[3](start = v_b_start) "World velocity [m/s]";
  output Real a_b[3](start = {0, 0, 0}) "Body specific force [m/s^2]";

protected
  RigidBody.State state;
  RigidBody.Wrench wrench;
  RigidBody.Parameters physicalParameters;
  Real worldPositionRate[3];
  Real bodyVelocityRate[3];
  Real attitudeRate[4];
  Real bodyAngularVelocityRate[3];

equation
  state.worldPosition = p;
  state.bodyVelocity = v_b;
  state.attitude = q;
  state.bodyAngularVelocity = omega;
  wrench.bodyForce = F_b;
  wrench.bodyTorque = M_b;
  physicalParameters = RigidBody.Parameters(
    mass=mass,
    gravity=g,
    inertia=J,
    quaternionNormGain=qnorm_gain);
  worldPositionRate = RigidBody.worldPositionRate(state);
  bodyVelocityRate = RigidBody.bodyVelocityRate(
    state, wrench, physicalParameters);
  attitudeRate = RigidBody.attitudeRate(state, physicalParameters);
  bodyAngularVelocityRate = RigidBody.bodyAngularVelocityRate(
    state, wrench, physicalParameters);

  R = LieGroups.SO3.Quat.to_DCM(state.attitude);
  v_w = worldPositionRate;
  a_b = F_b / mass;
  der(p) = worldPositionRate;
  der(v_b) = bodyVelocityRate;
  der(q) = attitudeRate;
  der(omega) = bodyAngularVelocityRate;
end RigidBody6DOF;
