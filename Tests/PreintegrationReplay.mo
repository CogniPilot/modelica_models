within Tests;

block PreintegrationReplay
  "800 Hz FOH preintegration for the external comparison harness"
  input Real rate[3];
  input Real force[3];
  input Real gyroBias[3];
  input Real accelBias[3];
  input Boolean clear;
  parameter Real samplePeriod = 0.00125;
  output Real position[3](each start=0, each fixed=true);
  output Real velocity[3](each start=0, each fixed=true);
  output Real quaternion[4](start={1,0,0,0}, each fixed=true);
  output Real rotationGyro[3,3](each start=0, each fixed=true);
  output Real velocityGyro[3,3](each start=0, each fixed=true);
  output Real velocityAccel[3,3](each start=0, each fixed=true);
  output Real positionGyro[3,3](each start=0, each fixed=true);
  output Real positionAccel[3,3](each start=0, each fixed=true);
protected
  Real previousRate[3](each start=0, each fixed=true);
  Real previousForce[3](start={0,0,9.81}, each fixed=true);
algorithm
  when sample(0,samplePeriod) then
    (position,velocity,quaternion,rotationGyro,velocityGyro,velocityAccel,
     positionGyro,positionAccel) := Estimation.StrapdownINS.preintegrateImuStep(
       if clear then zeros(3) else pre(position),
       if clear then zeros(3) else pre(velocity),
       if clear then {1,0,0,0} else pre(quaternion),
       if clear then zeros(3,3) else pre(rotationGyro),
       if clear then zeros(3,3) else pre(velocityGyro),
       if clear then zeros(3,3) else pre(velocityAccel),
       if clear then zeros(3,3) else pre(positionGyro),
       if clear then zeros(3,3) else pre(positionAccel),
       rate,force,gyroBias,accelBias,samplePeriod,true,
       pre(previousRate),pre(previousForce));
    previousRate := rate;
    previousForce := force;
  end when;
end PreintegrationReplay;
