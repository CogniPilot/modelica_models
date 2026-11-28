within Marine.Powerboats;
model BostonWhaler190 "Planing center-console boat with a steerable outboard"
  import Modelica.Units.SI;
  constant SI.Acceleration gravity=Modelica.Constants.g_n;

  parameter SI.Mass mass=1256 "Loaded mass with full tank";
  parameter SI.Area waterplaneArea=7.6 "Effective buoyancy area of the four stations";
  parameter SI.Density waterDensity=1025;
  parameter SI.Mass reserveFlotationMass(min=Modelica.Constants.small)=1814 "Equivalent mass supported by the fully immersed hull";
  parameter SI.Position loadedCenter[3]={0,0,.12} "Loaded center of mass relative to the body origin";
  parameter SI.Power shaftPower=112000 "Optional 150 hp outboard";
  parameter SI.Position initialEast=0,initialNorth=0,initialZ=.20;
  parameter SI.Angle initialHeading=0,initialRoll=0,initialPitch=0;
  parameter Real initialBilgeLitres(min=0,max=986,unit="l")=0;
  final parameter SI.Length draft=mass/(waterDensity*waterplaneArea) "Hydrostatic offset of the station reference plane";
  final parameter SI.Position fuelCenter[3]={-.68,0,.10} "Tank centroid in forward, port, up axes";
  final parameter Real fuelCapacity(unit="l")=227;
  parameter Real initialFuelLitres(min=0,max=fuelCapacity,unit="l")=fuelCapacity;
  final parameter SI.Length propellerRadius=.186,propellerOffset=.39,propellerPivot=-3.08,propellerHeight=-.56;
  final parameter SI.Position stationX[4]={1.65,1.65,-1.65,-1.65},stationY[4]={.65,-.65,.65,-.65};
  parameter SI.Position bowRollerBody[3]={3.04,0,1.055} "Bow roller attachment in forward, port, up axes";

  input Real throttleLever(start=0) "Throttle from full astern (-1) to full ahead (1)";
  input Real helmLever(start=0) "Steering from full starboard (-1) to full port (1)";
  input Real bilgePump(start=0) "Manual cockpit bilge pump switch (0 or 1)";
  input SI.Position stationSeaElevation[4](each start=0) "Water surface at the four horizontal sample points";
  input SI.Position propellerSeaElevation(start=0) "Water surface directly above the propeller center";
  input SI.Velocity waterVelocityENU[3](each start=0) "Ambient velocity in east, north, up axes; horizontal components set hull slip";
  input Real externalWrench[6](each start=0,each unit="1") "Packed numerical ENU force [N] and moment [N.m] about the boat origin";

  output Real boatPose[6] "East, north, up [m] and roll, pitch, yaw [rad]";
  output Real engine[4] "Engine rpm, through-water speed [m/s], thrust [N], and outboard steering angle [rad]";
  output SI.Time starterRemaining=powerplant.starterRemaining,engineIdleTime=powerplant.idleTime;
  output SI.Time engineOffAfter=powerplant.engineOffAfter;
  output Real fuelState[6]=powerplant.fuelState "Litres remaining, capacity, litres per hour, and tank centroid [m]";
  output SI.Mass ladenMass "Instantaneous hull inertial mass, including fuel and cockpit water";
  output Real bilgeState[4] "Retained water [L], inflow [L/s], pumped flow [L/s], passive drainage [L/s]";
  output Real propellerWetFraction "Fraction of the propeller disk below the local water surface";
  output SI.Length propellerDepth "Positive depth below the local water surface";
  output SI.Position stationSamplePointsENU[4,2] "East and north positions of the flotation stations";
  output SI.Position propellerPositionENU[3],bowRollerPositionENU[3];
  output Real rotationBodyToENU[3,3] "Rotation from forward, port, up axes to east, north, up axes";
  output SI.Velocity velocityENU[3];
  output SI.AngularVelocity angularVelocityENU[3] "World angular velocity for loads applied to the hull";

  SI.Position east(start=initialEast,fixed=true),north(start=initialNorth,fixed=true),z(start=initialZ,fixed=true);
  SI.Angle yaw(start=initialHeading,fixed=true),roll(start=initialRoll,fixed=true),pitch(start=initialPitch,fixed=true);
  SI.Velocity surge(start=0,fixed=true),sway(start=0,fixed=true),vertical(start=0,fixed=true);
  SI.AngularVelocity yawRate(start=0,fixed=true),rollRate(start=0,fixed=true),pitchRate(start=0,fixed=true);
  EngineFuelSystem powerplant(capacity=fuelCapacity,tankCenter=fuelCenter,initialFuelLitres=initialFuelLitres,
    idleFuelRate=2.3,fullFuelRate=55,throttleLever=throttleLever);

protected
  constant SI.Time waterSamplePeriod=.1;
  constant SI.Force forceUnit=1;
  constant SI.Torque momentUnit=1;
  constant SI.Position cockpitSole=.48;
  constant SI.Position cockpitRimBody[4,3]=[1.2,.95,.82;1.2,-.95,.82;-2.1,.95,.82;-2.1,-.95,.82];
  constant SI.Position scupperBody[2,3]=[-2.1,.77,.49;-2.1,-.77,.49];
  final parameter SI.Length flotationHalfHeight=reserveFlotationMass/(2*waterDensity*waterplaneArea);
  final parameter SI.Position flotationCenterZ=flotationHalfHeight-draft;
  OpenCockpitBilge bilge(initialVolume=initialBilgeLitres/1000,pumpCommand=bilgePump,
    upright=rotationBodyToENU[3,3],rimDepth=rimDepth,scupperDepth=scupperDepth);
  Real rpm(start=0,fixed=true);
  SI.Angle steering(start=0,fixed=true);
  Real forward[2],port[2],shaftAxis[3],diskPosition,throttle,dragCoefficient,planingFraction,waterFraction;
  Real wetFraction[4],fillFraction,floodedImmersion;
  SI.Velocity slip[2],stationRate[4];
  SI.Length planing,projectedHalfHeight,rimDepth[4],scupperDepth[2];
  SI.Position localSea[4],stationArm[4,3],propellerBody[3],propellerArm[3],floodWaterCenter[3],floodWaterArm[3];
  SI.Force thrust,lift[4],externalForceENU[3],effectiveFloodWeight;
  SI.Torque fuelMoment[3],driveMoment[3],externalMomentENU[3];
  SI.Torque weightMoment[3],buoyancyMoment[3],hullMoment[3];
  SI.Mass massChange(min=-mass),waterMass,payloadChange(min=-mass);
  SI.Inertia floodInertia[3];
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
  angularVelocityENU=rollRate*rotationBodyToENU[:,1]+cat(1,pitchRate*port,{yawRate});
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
  planing=noEvent(min(.22,max(0,slip[1]-4)*.022)*max(0,rotationBodyToENU[3,3])^2);
  massChange=powerplant.fuelMass-powerplant.fuelDensity*fuelCapacity/1000;
  waterMass=waterDensity*bilge.volume;
  payloadChange=massChange+waterMass;
  ladenMass=mass+payloadChange;
  bilgeState=1000*{bilge.volume,bilge.inflow,bilge.pumpFlow,bilge.gravityDrain};
  fillFraction=noEvent(min(1,bilge.standingDepth/bilge.rimHeight));
  // A partially flooded cockpit concentrates water along the downhill side.
  floodWaterCenter={-.55-.50*rotationBodyToENU[3,1]*(1-fillFraction),
    -.50*rotationBodyToENU[3,2]*(1-fillFraction),cockpitSole+.5*bilge.standingDepth};
  floodWaterArm=rotationBodyToENU*floodWaterCenter;
  floodedImmersion=noEvent(min(1,max(0,.5+
    (sum(localSea)/4-z-floodWaterArm[3])/max(.03,bilge.standingDepth))));
  // The submerged portion communicates with the sea. It adds inertia without a net weight load.
  effectiveFloodWeight=gravity*waterMass*(1-floodedImmersion);
  floodInertia=waterMass*{.15+floodWaterCenter[2]^2+floodWaterCenter[3]^2,
    .60+floodWaterCenter[1]^2+floodWaterCenter[3]^2,.75+floodWaterCenter[1]^2+floodWaterCenter[2]^2};
  fuelMoment=cross(rotationBodyToENU*fuelCenter,{0,0,-gravity*massChange});
  weightMoment=cross(rotationBodyToENU*loadedCenter,{0,0,-gravity*mass})+
    cross(floodWaterArm,{0,0,-effectiveFloodWeight});
  waterFraction=min(1,max(0,(time-waterStamp)/waterSamplePeriod));
  when sample(0,waterSamplePeriod) then
    sea=stationSeaElevation;
    previousSea=if initial() then sea else pre(sea);
    waterStamp=time;
  end when;
  // Finite buoyancy cells rotate with the hull. Their vertical depth includes
  // the projected beam and length as either axis approaches vertical.
  projectedHalfHeight=sqrt((flotationHalfHeight*rotationBodyToENU[3,3])^2+
    (.60*rotationBodyToENU[3,2])^2+(.90*rotationBodyToENU[3,1])^2);
  for i in 1:4 loop
    localSea[i]=(1-waterFraction)*previousSea[i]+waterFraction*sea[i];
    stationArm[i,:]=rotationBodyToENU*{stationX[i],stationY[i],flotationCenterZ};
    stationSamplePointsENU[i,:]={east,north}+stationArm[i,1:2];
    stationRate[i]=vertical+angularVelocityENU[1]*stationArm[i,2]-angularVelocityENU[2]*stationArm[i,1];
    wetFraction[i]=noEvent(max(0,min(1,.5+
      (localSea[i]-z-stationArm[i,3]+planing)/(2*projectedHalfHeight))));
    lift[i]=noEvent(max(0,reserveFlotationMass*gravity/4*wetFraction[i]+
      1300*min(1,10*min(wetFraction[i],1-wetFraction[i]))*
      ((sea[i]-previousSea[i])/waterSamplePeriod-stationRate[i])));
    rimDepth[i]=localSea[i]-z-rotationBodyToENU[3,:]*cockpitRimBody[i,:];
  end for;
  scupperDepth={localSea[i+2]-z-rotationBodyToENU[3,:]*scupperBody[i,:] for i in 1:2};
  buoyancyMoment={stationArm[:,2]*lift,-stationArm[:,1]*lift,0};
  hullMoment=buoyancyMoment+externalMomentENU+driveMoment+fuelMoment+weightMoment;

  (1450+payloadChange)*der(surge)=thrust*(forward*shaftAxis[1:2])-(.65+abs(slip[1]))*dragCoefficient*slip[1]
    +(1450+payloadChange)*sway*yawRate+forward*externalForceENU[1:2];
  (1750+payloadChange)*der(sway)=thrust*(port*shaftAxis[1:2])-(420+590*abs(slip[2])+175*abs(slip[1]))*slip[2]
    -(1750+payloadChange)*surge*yawRate+port*externalForceENU[1:2];
  (1750+payloadChange)*der(vertical)=sum(lift)-(mass+massChange)*gravity-effectiveFloodWeight+
    externalForceENU[3]+thrust*shaftAxis[3];
  (670+massChange*fuelCenter[3]^2+floodInertia[1])*der(rollRate)=
    rotationBodyToENU[:,1]*hullMoment-700*rollRate-500*surge*yawRate;
  (3100+massChange*(fuelCenter[1]^2+fuelCenter[3]^2)+floodInertia[2])*der(pitchRate)=port*hullMoment[1:2]-1900*pitchRate;
  (2350+massChange*fuelCenter[1]^2+floodInertia[3])*der(yawRate)=hullMoment[3]
    -(880+300*abs(slip[1])+430*abs(yawRate))*yawRate;

  boatPose={east,north,z,roll,pitch,yaw};
  engine={rpm,sqrt(slip*slip),thrust,steering};
  annotation(Documentation(info="<html><p>Four rotating buoyancy cells preserve the level-water displacement slope and limit static lift to the approximate swamped capacity. Their positions give the two sides distinct lever arms when the boat is on its beam ends or inverted. Cockpit overtopping uses the nearby wave samples for each rim point. Retained water contributes mass and inertia and shifts downhill when the cockpit is partially full.</p><p>The reduced-order rollover and flooding approximations represent vehicle behavior in simulation.</p></html>"));
end BostonWhaler190;
