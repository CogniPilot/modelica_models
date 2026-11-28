within Marine.Powerboats;
model BostonWhaler190 "Planing center-console boat with a steerable outboard"
  import Modelica.Units.SI;
  constant SI.Acceleration gravity=Modelica.Constants.g_n;

  parameter SI.Mass mass=1256 "Loaded mass with full tank";
  parameter SI.Area waterplaneArea=7.6 "Effective buoyancy area of the four stations";
  parameter SI.Density waterDensity=1025;
  parameter SI.Power shaftPower=112000 "Optional 150 hp outboard";
  parameter SI.Position initialEast=0,initialNorth=0,initialZ=.20;
  parameter SI.Angle initialHeading=0;
  final parameter SI.Length draft=mass/(waterDensity*waterplaneArea) "Hydrostatic offset of the station reference plane";
  final parameter SI.Position fuelCenter[3]={-.68,0,.10} "Tank centroid in forward, port, up axes";
  final parameter Real fuelCapacity(unit="l")=227;
  parameter Real initialFuelLitres(min=0,max=fuelCapacity,unit="l")=fuelCapacity;
  final parameter SI.Length propellerRadius=.186,propellerOffset=.39,propellerPivot=-3.08,propellerHeight=-.56;
  final parameter SI.Position stationX[4]={1.65,1.65,-1.65,-1.65},stationY[4]={.65,-.65,.65,-.65};
  parameter SI.Position bowRollerBody[3]={3.04,0,1.055} "Bow roller attachment in forward, port, up axes";

  input Real throttleLever(start=0) "Throttle from full astern (-1) to full ahead (1)";
  input Real helmLever(start=0) "Steering from full starboard (-1) to full port (1)";
  input SI.Position stationSeaElevation[4](each start=0) "Water surface at the four horizontal sample points";
  input SI.Position propellerSeaElevation(start=0) "Water surface directly above the propeller center";
  input SI.Velocity waterVelocityENU[3](each start=0) "Ambient velocity in east, north, up axes; horizontal components set hull slip";
  input Real externalWrench[6](each start=0,each unit="1") "Packed numerical ENU force [N] and moment [N.m] about the boat origin";

  output Real boatPose[6] "East, north, up [m] and roll, pitch, yaw [rad]";
  output Real engine[4] "Engine rpm, through-water speed [m/s], thrust [N], and outboard steering angle [rad]";
  output SI.Time starterRemaining=powerplant.starterRemaining,engineIdleTime=powerplant.idleTime;
  output SI.Time engineOffAfter=powerplant.engineOffAfter;
  output Real fuelState[6]=powerplant.fuelState "Litres remaining, capacity, litres per hour, and tank centroid [m]";
  output SI.Mass ladenMass "Instantaneous hull mass including fuel";
  output Real propellerWetFraction "Fraction of the propeller disk below the local water surface";
  output SI.Length propellerDepth "Positive depth below the local water surface";
  output SI.Position stationSamplePointsENU[4,2] "East and north positions of the flotation stations";
  output SI.Position propellerPositionENU[3],bowRollerPositionENU[3];
  output Real rotationBodyToENU[3,3] "Rotation from forward, port, up axes to east, north, up axes";
  output SI.Velocity velocityENU[3];
  output SI.AngularVelocity angularVelocityENU[3] "Small-roll and pitch approximation for loads applied to the hull";

  SI.Position east(start=initialEast,fixed=true),north(start=initialNorth,fixed=true),z(start=initialZ,fixed=true);
  SI.Angle yaw(start=initialHeading,fixed=true),roll(start=0,fixed=true),pitch(start=0,fixed=true);
  SI.Velocity surge(start=0,fixed=true),sway(start=0,fixed=true),vertical(start=0,fixed=true);
  SI.AngularVelocity yawRate(start=0,fixed=true),rollRate(start=0,fixed=true),pitchRate(start=0,fixed=true);
  EngineFuelSystem powerplant(capacity=fuelCapacity,tankCenter=fuelCenter,initialFuelLitres=initialFuelLitres,
    idleFuelRate=2.3,fullFuelRate=55,throttleLever=throttleLever);

protected
  constant SI.Time waterSamplePeriod=.1;
  constant SI.Force forceUnit=1;
  constant SI.Torque momentUnit=1;
  Real rpm(start=0,fixed=true);
  SI.Angle steering(start=0,fixed=true);
  Real forward[2],port[2],shaftAxis[3],diskPosition,throttle,dragCoefficient,planingFraction,waterFraction;
  SI.Velocity slip[2],stationRate[4];
  SI.Length planing;
  SI.Position stationHeight[4],propellerBody[3],propellerArm[3];
  SI.Force thrust,lift[4],externalForceENU[3];
  SI.Torque fuelMoment[3],driveMoment[3],externalMomentENU[3];
  SI.Mass massChange(min=-mass);
  discrete SI.Time waterStamp(start=0,fixed=true);
  discrete SI.Position sea[4](each start=0,each fixed=true),previousSea[4](each start=0,each fixed=true);

equation
  forward={cos(yaw),sin(yaw)};
  port={-sin(yaw),cos(yaw)};
  velocityENU=cat(1,surge*forward+sway*port,{vertical});
  der(east)=velocityENU[1];
  der(north)=velocityENU[2];
  der(z)=velocityENU[3];
  der(roll)=rollRate;
  der(pitch)=pitchRate;
  der(yaw)=yawRate;
  rotationBodyToENU=LieGroups.SO3.EulerB321.to_DCM({yaw,pitch,roll});
  angularVelocityENU=cat(1,rollRate*forward+pitchRate*port,{yawRate});
  bowRollerPositionENU={east,north,z}+rotationBodyToENU*bowRollerBody;
  externalForceENU=forceUnit*externalWrench[1:3];
  externalMomentENU=momentUnit*externalWrench[4:6];

  slip={surge-forward*waterVelocityENU[1:2],sway-port*waterVelocityENU[1:2]};
  throttle=powerplant.engineRunning*powerplant.throttle;
  der(rpm)=3*(powerplant.engineRunning*(600+5300*abs(throttle))-rpm);
  der(steering)=7*(.55*max(-1,min(1,helmLever))-steering);
  propellerBody={propellerPivot-propellerOffset*cos(steering),propellerOffset*sin(steering),propellerHeight};
  propellerArm=rotationBodyToENU*propellerBody;
  propellerPositionENU={east,north,z}+propellerArm;
  shaftAxis=rotationBodyToENU*{cos(steering),-sin(steering),0};
  propellerDepth=propellerSeaElevation-propellerPositionENU[3];
  diskPosition=noEvent(max(-1,min(1,propellerDepth/(propellerRadius*sqrt(max(.01,1-shaftAxis[3]^2))))));
  propellerWetFraction=.5+(asin(diskPosition)+diskPosition*sqrt(max(0,1-diskPosition^2)))/Modelica.Constants.pi;
  thrust=noEvent(if abs(throttle)<.008 or propellerWetFraction<=0 then 0 else
    (if throttle>=0 then 1 else -.52)*propellerWetFraction*max(0,(rpm-600)/5300)^2*
    min(4700,.56*shaftPower/max(4,abs(slip[1]))));
  driveMoment=cross(propellerArm,thrust*shaftAxis);

  planingFraction=noEvent(max(0,min(1,(abs(slip[1])-4)/7)));
  dragCoefficient=(1-planingFraction)*(75+16*exp(-((abs(slip[1])-4)/2)^2))+planingFraction*5.46;
  planing=min(.22,max(0,slip[1]-4)*.022);
  massChange=powerplant.fuelMass-powerplant.fuelDensity*fuelCapacity/1000;
  ladenMass=mass+massChange;
  fuelMoment=cross(rotationBodyToENU*fuelCenter,{0,0,-gravity*massChange});
  waterFraction=min(1,max(0,(time-waterStamp)/waterSamplePeriod));
  when sample(0,waterSamplePeriod) then
    sea=stationSeaElevation;
    previousSea=if initial() then sea else pre(sea);
    waterStamp=time;
  end when;
  for i in 1:4 loop
    stationSamplePointsENU[i,:]={east,north}+stationX[i]*forward+stationY[i]*port;
    stationHeight[i]=z+stationY[i]*sin(roll)-stationX[i]*sin(pitch);
    stationRate[i]=vertical+stationY[i]*cos(roll)*rollRate-stationX[i]*cos(pitch)*pitchRate;
    lift[i]=noEvent(max(0,waterDensity*gravity*waterplaneArea/4*
      min(.70,draft+planing+(1-waterFraction)*previousSea[i]+waterFraction*sea[i]-stationHeight[i])
      +1300*((sea[i]-previousSea[i])/waterSamplePeriod-stationRate[i])));
  end for;

  (1450+massChange)*der(surge)=thrust*(forward*shaftAxis[1:2])-(.65+abs(slip[1]))*dragCoefficient*slip[1]
    +(1450+massChange)*sway*yawRate+forward*externalForceENU[1:2];
  (1750+massChange)*der(sway)=thrust*(port*shaftAxis[1:2])-(420+590*abs(slip[2])+175*abs(slip[1]))*slip[2]
    -(1750+massChange)*surge*yawRate+port*externalForceENU[1:2];
  (1750+massChange)*der(vertical)=sum(lift)-ladenMass*gravity+externalForceENU[3]+thrust*shaftAxis[3];
  (670+massChange*fuelCenter[3]^2)*der(rollRate)=stationY*lift-700*rollRate-500*surge*yawRate
    +forward*(externalMomentENU[1:2]+driveMoment[1:2]+fuelMoment[1:2]);
  (3100+massChange*(fuelCenter[1]^2+fuelCenter[3]^2))*der(pitchRate)=-stationX*lift-1900*pitchRate
    +port*(externalMomentENU[1:2]+driveMoment[1:2]+fuelMoment[1:2]);
  (2350+massChange*fuelCenter[1]^2)*der(yawRate)=driveMoment[3]
    -(880+300*abs(slip[1])+430*abs(yawRate))*yawRate+externalMomentENU[3];

  boatPose={east,north,z,roll,pitch,yaw};
  engine={rpm,sqrt(slip*slip),thrust,steering};
end BostonWhaler190;
