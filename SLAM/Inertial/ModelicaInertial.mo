within SLAM.Inertial;
// Nominal INS without visual corrections. Body FLU inputs; world ENU outputs.
// Quaternion [w,x,y,z] rotates body to world. Rumoca integrates held IMU samples.
model ModelicaInertial
  parameter Real gravity = 9.81;
  parameter Real accel_tau = 0.03;
  parameter Real gyro_tau = 0.02;
  parameter Real accel_bias[3] = {0.0,0.0,0.0};
  parameter Real gyro_bias[3] = {0.0,0.0,0.0};
  input Real accel[3] = {0.0,0.0,9.81};
  input Real gyro[3] = {0.0,0.0,0.0};
  output Real position[3];
  output Real velocity[3];
  output Real quaternion[4];
  output Real filteredAccel[3];
  output Real filteredGyro[3];
protected
  Real integratedPosition[3](start={0.0,0.0,0.0},each fixed=true);
  Real integratedVelocity[3](start={0.0,0.0,0.0},each fixed=true);
  Real q[4](start={1.0,0.0,0.0,0.0},each fixed=true);
  Real specificForce[3](start={0.0,0.0,9.81},each fixed=true);
  Real angularVelocity[3](start={0.0,0.0,0.0},each fixed=true);
  Real quaternionNorm;
  Real worldAccel[3];
equation
  der(specificForce) = (accel-accel_bias-specificForce)/accel_tau;
  der(angularVelocity) = (gyro-gyro_bias-angularVelocity)/gyro_tau;
  der(integratedPosition) = integratedVelocity;
  der(integratedVelocity) = worldAccel;
  position = integratedPosition;
  velocity = integratedVelocity;
  filteredAccel = specificForce;
  filteredGyro = angularVelocity;

  der(q[1]) = -(q[2]*angularVelocity[1]+q[3]*angularVelocity[2]+q[4]*angularVelocity[3])/2;
  der(q[2]) = (q[1]*angularVelocity[1]+q[3]*angularVelocity[3]-q[4]*angularVelocity[2])/2;
  der(q[3]) = (q[1]*angularVelocity[2]-q[2]*angularVelocity[3]+q[4]*angularVelocity[1])/2;
  der(q[4]) = (q[1]*angularVelocity[3]+q[2]*angularVelocity[2]-q[3]*angularVelocity[1])/2;
  quaternionNorm = sqrt(q[1]^2+q[2]^2+q[3]^2+q[4]^2);
  quaternion = q/quaternionNorm;

  worldAccel[1] = (1-2*(quaternion[3]^2+quaternion[4]^2))*specificForce[1]
    +2*(quaternion[2]*quaternion[3]-quaternion[1]*quaternion[4])*specificForce[2]
    +2*(quaternion[2]*quaternion[4]+quaternion[1]*quaternion[3])*specificForce[3];
  worldAccel[2] = 2*(quaternion[2]*quaternion[3]+quaternion[1]*quaternion[4])*specificForce[1]
    +(1-2*(quaternion[2]^2+quaternion[4]^2))*specificForce[2]
    +2*(quaternion[3]*quaternion[4]-quaternion[1]*quaternion[2])*specificForce[3];
  worldAccel[3] = 2*(quaternion[2]*quaternion[4]-quaternion[1]*quaternion[3])*specificForce[1]
    +2*(quaternion[3]*quaternion[4]+quaternion[1]*quaternion[2])*specificForce[2]
    +(1-2*(quaternion[2]^2+quaternion[3]^2))*specificForce[3]-gravity;
end ModelicaInertial;
