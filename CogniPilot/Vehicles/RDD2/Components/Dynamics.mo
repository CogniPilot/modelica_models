within CogniPilot.Vehicles.RDD2.Components;
model Dynamics
  outer World world
    "world which contains gravity etc.";
  // parameters
  parameter Real m=1
    "mass";
  parameter Real Ixx=0.01
    "body forward axis inertia";
  parameter Real Iyy=0.01
    "body left axis inertia";
  parameter Real Izz=0.01
    "body up axis inertia";
  // assuming symmetry about x axis along body forward
  // assuming symmetry about y axis along body left
  // Ixz, Ixy, Iyz = 0
  parameter Real cf=0.00006135
    "motor/prop thrust coefficient";
  parameter Real ct=0.0000001
    "motor/prop torque coefficient";
  // data bus
  ZROS.Bus bus
    "data bus for ZBUS";
  // frame to attach to sensors
  OutFrame frame(
    r_op_i=r_op_i,
    v_ip_b=v_ip_b,
    w_ib_b=w_ib_b,
    C_ib=C_ib,
    a_ip_b=a_ip_b,
    alpha_ib_b=I_inv*m_b);
  //protected
  // data bus
  ZROS.SubActuators sub_actuators
    "actuators subscription";
  ZROS.PubOdometry pub_odometry
    "odometry publicatoin";
  // motor velocities
  Real prop_w_cmd[4]
    "motor angular velocity command";
  Real f_b[3]
    "force in body frame";
  Real m_b[3]
    "moment in body frame";
  // states
  Real r_op_i[3]
    "position from point o (fixed in frame i) to point p (fixed in body) expresssed in inertial frame";
  Real v_ip_b[3]
    "velocity of body wrt inertial frame expressed in body frame";
  Real a_ip_b[3]
    "acceleration of body wrt inertial frame expressed in body frame";
  Real w_ib_b[3]
    "angular velocity of body wrt inertial frame expressed in body frame";
  Lie.SO3.Mrp.Kinematics mrp(
    a0=Lie.SO3.Mrp.Element(
      r={0,0,0}),
    omega=w_ib_b);
  Real I[3,3]=diagonal(
    {Ixx,Iyy,Izz})
    "inertia matrix";
  Real I_inv[3,3]=diagonal(
    {1/Ixx,1/Iyy,1/Izz})
    "inverse of inertia matrix";
  Real C_ib[3,3];
  Real g_b[3]=transpose(
    C_ib)*world.g;
  constant Integer n_prop=4;
  constant Real prop_dir[n_prop]={1,1,-1,-1};
  constant Real prop_r[n_prop,3]={{0.12,-0.12,0},// fwd, right
  {-0.12,0.12,0},// rear, left
  {0.12,0.12,0},// fwd, left
  {-0.12,-0.12,0} // rear, right
  };
  // TODO: add motor lag for props
  Real prop_thrust[n_prop]=cf*prop_w_cmd .^ 2;
  Real prop_torque[n_prop]=ct*prop_dir .* prop_w_cmd .^ 2;
  constant Real[3] z_b={0,0,1};
equation
  // bottleneck for speed
  C_ib=Lie.SO3.Mrp.to_matrix(
    mrp.a);
  // bus connections
  connect(bus.actuators,sub_actuators);
  connect(bus.ground_truth,pub_odometry);
  // ground truth updates
  pub_odometry.updated=sub_actuators.updated;
  // compute force/moment in body frame
  m_b=sum(
    prop_torque[i]*z_b+cross(
      prop_r[i],
      prop_thrust[i]*z_b) for i in 1:n_prop);
  f_b=sum(
    prop_thrust[i]*z_b for i in 1:n_prop);
  // kinematics
  der(
    r_op_i)=v_ip_b;
  der(
    v_ip_b)=a_ip_b-cross(
    w_ib_b,
    v_ip_b);
  // dynamics
  a_ip_b=f_b/m+g_b;
  m_b=I*der(
    w_ib_b)+cross(
    w_ib_b,
    I*w_ib_b);
initial algorithm
  prop_w_cmd := sub_actuators.msg.vel;
  pub_odometry.msg.time := time;
  pub_odometry.msg.r_op_i := r_op_i;
  pub_odometry.msg.v_ip_b := v_ip_b;
  pub_odometry.msg.w_ib_b := w_ib_b;
  pub_odometry.msg.C_ib := C_ib;
algorithm
  when edge(
    sub_actuators.updated) then
    // update motors
    prop_w_cmd := sub_actuators.msg.vel;
    // publish ground truth
    pub_odometry.msg.time := time;
    pub_odometry.msg.r_op_i := r_op_i;
    pub_odometry.msg.v_ip_b := v_ip_b;
    pub_odometry.msg.w_ib_b := w_ib_b;
    pub_odometry.msg.C_ib := C_ib;
  end when;
end Dynamics;
