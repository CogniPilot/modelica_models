within SLAM.Examples.Vehicles;
// Application references and ENU/FLU interface around the upstream controllers.
model LabQuadrotor
  import QuadrotorSIL = SLAM.Examples.Vehicles.QuadrotorSIL;

  constant Integer motorCount = 4;
  constant Real frameHalf = 0.7071067811865476;
  constant Real worldToEnu[3,3] = {{0,-1,0},{1,0,0},{0,0,1}};
  constant Real spinDirection[motorCount] = {1,1,-1,-1};
  parameter Real mass = 2.0;
  parameter Real gravity = 9.81;
  parameter Real Ix = 0.02166666666666667;
  parameter Real Iy = 0.02166666666666667;
  parameter Real Iz = 0.04000000000000001;
  parameter Real arm = 0.25;
  parameter Real k_thrust = 8.54858e-6;
  parameter Real k_torque = 0.016;
  parameter Real initialHeight = 1.5;
  parameter Real controlPeriod = 0.01 "Position integral sample period s";
  parameter Real rateGain[3] = {20,20,10};
  parameter Real maximumMoment[3] = {2.6,2.6,0.30};
  parameter Real maximumMotorSpeed = 1100 "Rotor speed at full command rad/s";
  input Real forward(start=0) "Body forward velocity setpoint m/s";
  input Real left(start=0) "Body left velocity setpoint m/s";
  input Real up(start=0) "World vertical velocity setpoint m/s";
  input Real yaw(start=0) "Yaw rate setpoint rad/s";
  input Real autopilot = 0.0;
  input Real indoorTour = 0.0;
  input Real commandTime = 0.0 "Held start-of-frame simulation timestamp";
  input Real positionMode = 0.0 "1 selects an explicit trajectory reference";
  input Real targetPosition[3] = {0,0,initialHeight} "World ENU position m";
  input Real targetVelocity[3] = zeros(3) "World ENU velocity m/s";
  input Real targetAcceleration[3] = zeros(3) "World ENU acceleration m/s2";
  input Real targetHeading = 0.0 "World ENU heading rad";
  output Real forwardSetpoint; output Real leftSetpoint;
  output Real upSetpoint; output Real yawSetpoint;
  output Real x; output Real y; output Real z;
  output Real vx; output Real vy; output Real vz;
  output Real qw; output Real qx; output Real qy; output Real qz;
  output Real p; output Real q; output Real r;
  output Real imu_ax; output Real imu_ay; output Real imu_az;
  output Real omega_m[motorCount] "Actual plant rotor speeds rad/s";
  output Real propellerAngles[motorCount](each start=0.0,each fixed=true)
    "Unwrapped actual rotor angle; CCW motors 1/2 positive, CW motors 3/4 negative";
  output Real positionSetpoint[3];
  output Real velocitySetpoint[3];
  output Real angularVelocitySetpoint[3];
  output Real motorCommand[motorCount] "Normalized allocated motor commands";
protected
  QuadrotorSIL vehicle(
    vehicle_mass=mass,vehicle_ixx=Ix,vehicle_iyy=Iy,vehicle_izz=Iz,g=gravity,
    arm_length=arm,Ct=k_thrust,Cm=k_torque,
    initial_ground_clearance=initialHeight-0.10,
    q_start={frameHalf,0,0,-frameHalf});
  Real command[4];
  Real position[3]; Real velocity[3]; Real quaternion[4]; Real orientation[3,3];
  Control.Multirotor.LogLinear.Controller controller(
    samplePeriod=controlPeriod,mass=mass,gravity=gravity,thrustTrim=mass*gravity);
  Real pathPosition[3](start={0,0,initialHeight},each fixed=true);
  Real pathHeading(start=0, fixed=true);
  Real pathVelocity[3];
  Real headingSetpoint;
  Real unboundedMoment[3]; Real desiredMoment[3];
  parameter Real momentArm = arm*0.7071067811865476;
  parameter Real wrenchToRotorThrust[motorCount,4] = {
    {0.25,-1/(4*momentArm),-1/(4*momentArm),-1/(4*k_torque)},
    {0.25, 1/(4*momentArm), 1/(4*momentArm),-1/(4*k_torque)},
    {0.25, 1/(4*momentArm),-1/(4*momentArm), 1/(4*k_torque)},
    {0.25,-1/(4*momentArm), 1/(4*momentArm), 1/(4*k_torque)}};
equation
  // Modelica owns both tours. commandTime is held for a complete lockstep
  // interval, preserving the acquisition-boundary command timing.
  command = if noEvent(autopilot > 0.5) then {
    if noEvent(indoorTour > 0.5) then
      (if noEvent(commandTime < 40.0) then 1.1 else if noEvent(commandTime < 50.0) then 0.0
       else if noEvent(commandTime < 90.0) then -1.1 else 0.0) else 0.6,
    0.0,
    if noEvent(indoorTour > 0.5) then 0.0 else 0.06*sin(commandTime*0.3),
    if noEvent(indoorTour > 0.5 or commandTime < 2.0) then 0.0 else 0.4
  } else {forward,left,up,yaw};
  forwardSetpoint=command[1]; leftSetpoint=command[2];
  upSetpoint=command[3]; yawSetpoint=command[4];

  position = worldToEnu*vehicle.position;
  velocity = worldToEnu*vehicle.velocity;
  orientation = worldToEnu*vehicle.R;
  quaternion = frameHalf*{
    vehicle.quat[1]-vehicle.quat[4],vehicle.quat[2]-vehicle.quat[3],
    vehicle.quat[3]+vehicle.quat[2],vehicle.quat[4]+vehicle.quat[1]};
  x=position[1]; y=position[2]; z=position[3];
  vx=velocity[1]; vy=velocity[2]; vz=velocity[3];
  qw=quaternion[1]; qx=quaternion[2]; qy=quaternion[3]; qz=quaternion[4];
  p=vehicle.omega[1]; q=vehicle.omega[2]; r=vehicle.omega[3];
  // Keep the existing FLU IMU contract, rather than the plant's FRD aliases.
  imu_ax=vehicle.a_b[1]; imu_ay=vehicle.a_b[2]; imu_az=vehicle.a_b[3];
  omega_m=vehicle.omega_m;

  pathVelocity = {
    cos(pathHeading)*command[1]-sin(pathHeading)*command[2],
    sin(pathHeading)*command[1]+cos(pathHeading)*command[2],command[3]};
  der(pathPosition) = pathVelocity;
  der(pathHeading) = command[4];
  positionSetpoint = if noEvent(positionMode > 0.5) then targetPosition else pathPosition;
  velocitySetpoint = if noEvent(positionMode > 0.5) then targetVelocity else pathVelocity;
  headingSetpoint = if noEvent(positionMode > 0.5) then targetHeading else pathHeading;

  controller.positionWorld = position;
  controller.velocityWorld = velocity;
  controller.quaternionWorldBody = quaternion;
  controller.positionReferenceWorld = positionSetpoint;
  controller.velocityReferenceWorld = velocitySetpoint;
  controller.accelerationReferenceWorld = if noEvent(positionMode > 0.5) then targetAcceleration else zeros(3);
  controller.headingQuaternionReference = {cos(headingSetpoint/2),0,0,sin(headingSetpoint/2)};
  controller.resetIntegral = false;
  angularVelocitySetpoint = controller.angularVelocitySetpoint;
  unboundedMoment = Control.Multirotor.RateLoop.bodyMoment(
    angularVelocitySetpoint,vehicle.omega,{Ix,Iy,Iz},rateGain);
  desiredMoment = {min(maximumMoment[axis],max(-maximumMoment[axis],unboundedMoment[axis])) for axis in 1:3};
  motorCommand = Control.Multirotor.Allocation.rotorCommands(
    motorCount,controller.thrust,desiredMoment,wrenchToRotorThrust,
    fill(k_thrust,motorCount),fill(maximumMotorSpeed,motorCount));
  vehicle.omega_cmd = maximumMotorSpeed*motorCommand;
  for motor in 1:motorCount loop
    der(propellerAngles[motor])=spinDirection[motor]*vehicle.omega_m[motor];
  end for;
end LabQuadrotor;
