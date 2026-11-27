within Vehicles.Ground;
model SprungBicycleStep "Low-speed SUV bicycle dynamics with four wheel contacts and sprung body"
  input Real previous[11] "East, north, yaw, speed, steering, wheel travel, heave, heave rate, gear, shift time, engine RPM";
  input Real ground[4](each unit="m") "Terrain heights at front left, front right, rear left, rear right";
  input Real drive "Accelerator pedal";
  input Real brake "Brake pedal";
  input Real steering "Positive right";
  input Real dt(unit="s",min=0) "Sample interval";
  input Real boost "One when high-rate driving is selected";
  output Real state[11];
  output Real wheelsENU[4,3](each unit="m");
  output Real wheelSpin[4](each unit="rad");
  output Real wheelSteer[2](each unit="rad");
  output Real chassisENU[3](each unit="m");
  output Real attitude[3](each unit="rad") "Heading, pitch up, roll right down";
  output Real engineRPM;
  output Real gear "Selected forward gear";
  output Real braking;
  parameter Real wheelbase(unit="m",min=1e-6)=2.568;
  parameter Real halfTrack(unit="m",min=1e-6)=0.794;
  parameter Real wheelRadius(unit="m",min=1e-6)=0.327;
  parameter Real finalDrive(min=1e-6)=3.73;
  parameter Real curbMass(unit="kg",min=1e-6)=1780;
  parameter Real drivelineEfficiency(min=0,max=1)=0.86;
protected
  Real goalSteer, speed, yaw, heaveGoal, heaveRate, bodyRoll, bodyPitch, midYaw, turnRate;
  parameter Real along[4]={wheelbase/2,wheelbase/2,-wheelbase/2,-wheelbase/2};
  parameter Real side[4]={-halfTrack,halfTrack,-halfTrack,halfTrack};
  Real acceleration, frontSlope, rightSlope, wheelCurvature, gearRatio, previousRatio, coupledRPM, goalRPM;
  Real pedal, driveTorque, tractiveForce, rollingForce, velocityLimit;
equation
  pedal = min(1,max(0,drive))*(1-min(1,max(0,brake)));
  previousRatio = noEvent(if previous[9] < 1.5 then 3.66 elseif previous[9] < 2.5 then 2.11
    elseif previous[9] < 3.5 then 1.42 else 1);
  coupledRPM = previous[4]*previousRatio*finalDrive*60/(6.283185307179586*wheelRadius);
  gear = noEvent(if previous[10] > 0 then previous[9]
    elseif coupledRPM > 2370 + 620*pedal + 180*boost then min(4,previous[9]+1)
    elseif coupledRPM < 1050 + 510*pedal then max(1,previous[9]-1) else previous[9]);
  gearRatio = noEvent(if gear < 1.5 then 3.66 elseif gear < 2.5 then 2.11 elseif gear < 3.5 then 1.42 else 1);
  driveTorque = pedal*(145 + 145*boost);
  tractiveForce = driveTorque*gearRatio*finalDrive*drivelineEfficiency/wheelRadius*
    noEvent(if previous[10] > 0.38 then 0.42 else 1);
  rollingForce = curbMass*(0.21 + 0.040*previous[4] + 0.003*previous[4]^2);
  velocityLimit = 8.5 + 7.5*min(1,max(0,boost));
  acceleration = (tractiveForce-rollingForce)/curbMass-7*min(1,max(0,brake))-
    1.6*max(0,previous[4]-velocityLimit);
  speed = max(0,previous[4]+dt*acceleration);
  braking = min(1,max(0,brake));
  goalSteer = 0.47*steering;
  state[5] = previous[5] + max(-1.8*dt,min(1.8*dt,goalSteer-previous[5]));
  wheelCurvature = tan(state[5])/wheelbase;
  turnRate = speed*wheelCurvature;
  yaw = previous[3]+dt*turnRate;
  midYaw = (yaw+previous[3])/2;
  state[1] = previous[1]+dt*speed*sin(midYaw);
  state[2] = previous[2]+dt*speed*cos(midYaw);
  state[3] = yaw;
  state[4] = speed;
  goalRPM = min(4600,max(850 + 650*pedal,speed*gearRatio*finalDrive*60/(6.283185307179586*wheelRadius)));
  engineRPM = previous[11] + min(1,dt*9)*(goalRPM-previous[11]);
  state[6] = previous[6]+dt*speed;
  heaveGoal = (ground[1]+ground[2]+ground[3]+ground[4])/4;
  heaveRate = previous[8]+dt*(95*(heaveGoal-previous[7])-13*previous[8]);
  state[7] = previous[7]+dt*heaveRate;
  state[8] = heaveRate;
  state[9] = gear;
  state[10] = noEvent(if abs(gear-previous[9]) > 0.5 then 0.50 else max(0,previous[10]-dt));
  state[11] = engineRPM;
  frontSlope = (ground[1]+ground[2]-ground[3]-ground[4])/(2*wheelbase);
  rightSlope = (ground[1]+ground[3]-ground[2]-ground[4])/(4*halfTrack);
  bodyPitch = max(-0.32,min(0.32,atan(frontSlope)+0.012*(speed-previous[4])/max(0.001,dt)));
  bodyRoll = max(-0.32,min(0.32,atan(rightSlope)-0.018*turnRate*speed));
  attitude = {yaw,bodyPitch,bodyRoll};
  chassisENU = {state[1],state[2],state[7]};
  for k in 1:4 loop
    wheelsENU[k,1] = state[1]+along[k]*sin(yaw)+side[k]*cos(yaw);
    wheelsENU[k,2] = state[2]+along[k]*cos(yaw)-side[k]*sin(yaw);
    wheelsENU[k,3] = ground[k]+wheelRadius;
    wheelSpin[k] = state[6]/wheelRadius;
  end for;
  wheelSteer = {atan(wheelbase*wheelCurvature/max(.25,1+halfTrack*wheelCurvature)),
    atan(wheelbase*wheelCurvature/max(.25,1-halfTrack*wheelCurvature))};
end SprungBicycleStep;
