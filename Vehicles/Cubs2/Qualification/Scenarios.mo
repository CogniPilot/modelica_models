within Vehicles.Cubs2.Qualification;

model TakeoffOpenLoop
  Vehicles.Cubs2.Plant vehicle;
  Vehicles.Cubs2.InnerLoop innerLoop;

  output Real time_s;
  output Real x_m;
  output Real y_m;
  output Real z_m;
  output Real roll_rad;
  output Real pitch_rad;
  output Real yaw_rad;
  output Real airspeed_m_s;
  output Real throttle_cmd;
  output Real pitch_cmd;
  output Real roll_cmd;

protected
  Real euler_rad[3];

equation
  innerLoop.armed = 1.0;
  innerLoop.stick_roll = 0.0;
  innerLoop.stick_pitch = if time < 0.8 then 0.0 else 0.55;
  innerLoop.stick_yaw = 0.0;
  innerLoop.stick_throttle = 1.0;
  innerLoop.gyro = vehicle.gyro;
  innerLoop.up_body = vehicle.up_body;
  innerLoop.airspeed = vehicle.airspeed;

  vehicle.ail = innerLoop.ail;
  vehicle.elev = innerLoop.elev;
  vehicle.rud = innerLoop.rud;
  vehicle.thr = innerLoop.thr;

  euler_rad = Vehicles.Interfaces.eulerFromQuaternion(vehicle.quat);
  time_s = time;
  x_m = vehicle.position[1];
  y_m = vehicle.position[2];
  z_m = vehicle.position[3];
  roll_rad = euler_rad[1];
  pitch_rad = euler_rad[2];
  yaw_rad = euler_rad[3];
  airspeed_m_s = vehicle.airspeed;
  throttle_cmd = innerLoop.stick_throttle;
  pitch_cmd = innerLoop.stick_pitch;
  roll_cmd = innerLoop.stick_roll;
end TakeoffOpenLoop;

model Takeoff
  parameter Vehicles.Cubs2.RouteParameters takeoffRoute =
    Vehicles.Cubs2.RouteParameters(
      nSegments = 6,
      cruiseSpeed = 4.5,
      waypoints = [
        0.0,   0.0, 0.0;
        30.0,  0.0, 3.0;
        60.0,  0.0, 3.0;
        90.0,  0.0, 3.0;
        120.0, 0.0, 3.0;
        150.0, 0.0, 3.0;
        180.0, 0.0, 3.0
      ]
    );

  Vehicles.Cubs2.Plant vehicle;
  Vehicles.Cubs2.InnerLoop innerLoop;
  Vehicles.Cubs2.OuterLoop outerLoop(
    route = takeoffRoute,
    estimator(useMeasuredRates = 0.0)
  );

  output Real time_s;
  output Real x_m;
  output Real y_m;
  output Real z_m;
  output Real roll_rad;
  output Real pitch_rad;
  output Real yaw_rad;
  output Real airspeed_m_s;
  output Real throttle_cmd;
  output Real pitch_cmd;
  output Real roll_cmd;

protected
  Real euler_rad[3];

equation
  euler_rad = Vehicles.Interfaces.eulerFromQuaternion(vehicle.quat);
  outerLoop.position_m = vehicle.position;
  outerLoop.euler_rad = euler_rad;
  outerLoop.velocity_m_s = vehicle.velocity;
  outerLoop.eulerRate_rad_s = vehicle.gyro;
  outerLoop.engaged = 1.0;

  innerLoop.armed = 1.0;
  innerLoop.stick_roll = outerLoop.aileron;
  innerLoop.stick_pitch = outerLoop.elevator;
  innerLoop.stick_yaw = outerLoop.rudder;
  innerLoop.stick_throttle = outerLoop.throttle;
  innerLoop.gyro = vehicle.gyro;
  innerLoop.up_body = vehicle.up_body;
  innerLoop.airspeed = vehicle.airspeed;

  vehicle.ail = innerLoop.ail;
  vehicle.elev = innerLoop.elev;
  vehicle.rud = innerLoop.rud;
  vehicle.thr = innerLoop.thr;

  time_s = time;
  x_m = vehicle.position[1];
  y_m = vehicle.position[2];
  z_m = vehicle.position[3];
  roll_rad = euler_rad[1];
  pitch_rad = euler_rad[2];
  yaw_rad = euler_rad[3];
  airspeed_m_s = vehicle.airspeed;
  throttle_cmd = innerLoop.stick_throttle;
  pitch_cmd = innerLoop.stick_pitch;
  roll_cmd = innerLoop.stick_roll;
end Takeoff;

model AltitudeHold
  parameter Real targetAltitude_m = 3.0;
  parameter Vehicles.Cubs2.RouteParameters straightRoute =
    Vehicles.Cubs2.RouteParameters(
      nSegments = 6,
      cruiseSpeed = 4.0,
      waypointSwitchingDistance = 3.0,
      waypoints = [
        0.0,   0.0, 3.0;
        30.0,  0.0, 3.0;
        60.0,  0.0, 3.0;
        90.0,  0.0, 3.0;
        120.0, 0.0, 3.0;
        150.0, 0.0, 3.0;
        180.0, 0.0, 3.0
      ]
    );

  Vehicles.Cubs2.Plant vehicle(
    p_start = {0.0, 0.0, targetAltitude_m},
    v_b_start = {4.0, 0.0, 0.0}
  );
  Vehicles.Cubs2.InnerLoop innerLoop;
  Vehicles.Cubs2.OuterLoop outerLoop(route = straightRoute);

  output Real time_s;
  output Real x_m;
  output Real y_m;
  output Real z_m;
  output Real altitude_error_m;
  output Real airspeed_m_s;
  output Real pitch_cmd;
  output Real throttle_cmd;

protected
  Real euler_rad[3];

equation
  euler_rad = Vehicles.Interfaces.eulerFromQuaternion(vehicle.quat);

  outerLoop.position_m = vehicle.position;
  outerLoop.euler_rad = euler_rad;
  outerLoop.velocity_m_s = vehicle.velocity;
  outerLoop.eulerRate_rad_s = vehicle.gyro;
  outerLoop.engaged = 1.0;

  innerLoop.armed = 1.0;
  innerLoop.stick_roll = outerLoop.aileron;
  innerLoop.stick_pitch = outerLoop.elevator;
  innerLoop.stick_yaw = outerLoop.rudder;
  innerLoop.stick_throttle = outerLoop.throttle;
  innerLoop.gyro = vehicle.gyro;
  innerLoop.up_body = vehicle.up_body;
  innerLoop.airspeed = vehicle.airspeed;

  vehicle.ail = innerLoop.ail;
  vehicle.elev = innerLoop.elev;
  vehicle.rud = innerLoop.rud;
  vehicle.thr = innerLoop.thr;

  time_s = time;
  x_m = vehicle.position[1];
  y_m = vehicle.position[2];
  z_m = vehicle.position[3];
  altitude_error_m = targetAltitude_m - z_m;
  airspeed_m_s = vehicle.airspeed;
  pitch_cmd = innerLoop.stick_pitch;
  throttle_cmd = innerLoop.stick_throttle;
end AltitudeHold;

model HeadingHold
  parameter Real targetSpeed_m_s = 4.0;
  parameter Real targetAltitude_m = 3.0;
  parameter Vehicles.Cubs2.RouteParameters straightRoute =
    Vehicles.Cubs2.RouteParameters(
      nSegments = 6,
      cruiseSpeed = targetSpeed_m_s,
      waypointSwitchingDistance = 3.0,
      waypoints = [
        0.0,   0.0, targetAltitude_m;
        30.0,  0.0, targetAltitude_m;
        60.0,  0.0, targetAltitude_m;
        90.0,  0.0, targetAltitude_m;
        120.0, 0.0, targetAltitude_m;
        150.0, 0.0, targetAltitude_m;
        180.0, 0.0, targetAltitude_m
      ]
    );

  Vehicles.Cubs2.Plant vehicle(
    p_start = {0.0, 0.0, targetAltitude_m},
    v_b_start = {4.0, 0.0, 0.0},
    q_start = {0.9689124217106447, 0.0, 0.0, -0.24740395925452294}
  );
  Vehicles.Cubs2.InnerLoop innerLoop;
  Vehicles.Cubs2.OuterLoop outerLoop(route = straightRoute);

  output Real time_s;
  output Real x_m;
  output Real y_m;
  output Real z_m;
  output Real heading_error_rad;
  output Real roll_cmd;
  output Real pitch_cmd;
  output Real throttle_cmd;
  output Real airspeed_m_s;

protected
  Real euler_rad[3];

equation
  euler_rad = Vehicles.Interfaces.eulerFromQuaternion(vehicle.quat);

  outerLoop.position_m = vehicle.position;
  outerLoop.euler_rad = euler_rad;
  outerLoop.velocity_m_s = vehicle.velocity;
  outerLoop.eulerRate_rad_s = vehicle.gyro;
  outerLoop.engaged = 1.0;

  innerLoop.armed = 1.0;
  innerLoop.stick_roll = outerLoop.aileron;
  innerLoop.stick_pitch = outerLoop.elevator;
  innerLoop.stick_yaw = outerLoop.rudder;
  innerLoop.stick_throttle = outerLoop.throttle;
  innerLoop.gyro = vehicle.gyro;
  innerLoop.up_body = vehicle.up_body;
  innerLoop.airspeed = vehicle.airspeed;

  vehicle.ail = innerLoop.ail;
  vehicle.elev = innerLoop.elev;
  vehicle.rud = innerLoop.rud;
  vehicle.thr = innerLoop.thr;

  time_s = time;
  x_m = vehicle.position[1];
  y_m = vehicle.position[2];
  z_m = vehicle.position[3];
  heading_error_rad = MathUtilities.wrapAngle(outerLoop.desiredHeading - euler_rad[3]);
  roll_cmd = innerLoop.stick_roll;
  pitch_cmd = innerLoop.stick_pitch;
  throttle_cmd = innerLoop.stick_throttle;
  airspeed_m_s = vehicle.airspeed;
end HeadingHold;

model PatternMission
  parameter Vehicles.Cubs2.RouteParameters patternRoute =
    Vehicles.Cubs2.RouteParameters(
      nSegments = 6,
      cruiseSpeed = 4.0,
      waypointSwitchingDistance = 3.0,
      waypoints = [
        0.0,  0.0, 3.0;
        12.0, 0.0, 3.0;
        30.0, 0.0, 3.0;
        30.0, 20.0, 3.0;
        0.0,  20.0, 3.0;
        0.0,  0.0, 3.0;
        12.0, 0.0, 3.0
      ]
    );

  Vehicles.Cubs2.Plant vehicle(
    p_start = {0.0, 0.0, 3.0},
    v_b_start = {4.0, 0.0, 0.0}
  );
  Vehicles.Cubs2.InnerLoop innerLoop;
  Vehicles.Cubs2.OuterLoop outerLoop(route = patternRoute);

  output Real time_s;
  output Real x_m;
  output Real y_m;
  output Real z_m;
  output Real airspeed_m_s;
  output Real desired_heading_rad;
  output Real current_waypoint;
  output Real laps;
  output Real roll_cmd;
  output Real pitch_cmd;
  output Real throttle_cmd;
  output Real mission_phase;

protected
  Real euler_rad[3];
  discrete Integer lapCount(start = 0, fixed = true);
  discrete Integer previousWaypoint(start = 1, fixed = true);
  discrete Boolean landing(start = false, fixed = true);

algorithm
  when sample(0.0, 0.02) then
    if not pre(landing)
       and outerLoop.airborne
       and vehicle.position[3] > 2.0
       and pre(previousWaypoint) == 6
       and outerLoop.currentWaypoint == 1 then
      lapCount := pre(lapCount) + 1;
    else
      lapCount := pre(lapCount);
    end if;

    previousWaypoint := outerLoop.currentWaypoint;
    landing := pre(landing) or lapCount >= 2;
  end when;

equation
  euler_rad = Vehicles.Interfaces.eulerFromQuaternion(vehicle.quat);

  outerLoop.position_m = vehicle.position;
  outerLoop.euler_rad = euler_rad;
  outerLoop.velocity_m_s = vehicle.velocity;
  outerLoop.eulerRate_rad_s = vehicle.gyro;
  outerLoop.engaged = 1.0;

  innerLoop.armed = if landing and vehicle.position[3] < 0.4 then 0.0 else 1.0;
  innerLoop.stick_roll = if landing then 0.0 else outerLoop.aileron;
  innerLoop.stick_pitch = if landing then -0.25 else outerLoop.elevator;
  innerLoop.stick_yaw = if landing then 0.0 else outerLoop.rudder;
  innerLoop.stick_throttle = if landing then 0.12 else outerLoop.throttle;
  innerLoop.gyro = vehicle.gyro;
  innerLoop.up_body = vehicle.up_body;
  innerLoop.airspeed = vehicle.airspeed;

  vehicle.ail = innerLoop.ail;
  vehicle.elev = innerLoop.elev;
  vehicle.rud = innerLoop.rud;
  vehicle.thr = innerLoop.thr;

  time_s = time;
  x_m = vehicle.position[1];
  y_m = vehicle.position[2];
  z_m = vehicle.position[3];
  airspeed_m_s = vehicle.airspeed;
  desired_heading_rad = outerLoop.desiredHeading;
  current_waypoint = outerLoop.currentWaypoint;
  laps = lapCount;
  roll_cmd = innerLoop.stick_roll;
  pitch_cmd = innerLoop.stick_pitch;
  throttle_cmd = innerLoop.stick_throttle;
  mission_phase = if landing then 3.0 else if outerLoop.airborne then 2.0 else 1.0;
end PatternMission;
