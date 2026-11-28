within Marine.Powerboats;
model InboardPlaningCruiser "Reduced-order inboard cruiser with a shaft propeller and planar V-hull strips"
  parameter Real referenceMass(min=1,unit="kg")=3250 "Displacement with a full tank";
  parameter Real fuelCapacity(min=0,unit="l")=380;
  parameter Real initialFuelLitres(min=0,max=fuelCapacity,unit="l")=fuelCapacity;
  parameter Real fuelDensity(min=0,unit="kg/m3")=740;
  parameter Real fuelCenter[3](each unit="m")={-.62,0,-.18};
  parameter Real idleFuelRate(min=0,unit="l/h")=4.5;
  parameter Real fullFuelRate(min=idleFuelRate,unit="l/h")=140;
  parameter Real engineOffAfter(unit="s")=10;
  parameter Real engineCrankTime(unit="s")=1;
  parameter Real shaftRpmAhead(min=0)=2900,shaftRpmAstern(min=0)=1100;
  parameter Real shaftTimeConstant(min=.01,unit="s")=.85;
  parameter Real addedHeaveMass(min=0,unit="kg")=2600;
  parameter Real rollInertia(min=1,unit="kg.m2")=3600;
  parameter Real pitchInertia(min=1,unit="kg.m2")=22500;
  parameter Real yawInertia(min=1,unit="kg.m2")=14000;
  parameter Real waterDensity(min=1,unit="kg/m3")=1025;
  parameter Real hullStations[7](each unit="m")={-3.8,-2.7,-1.55,-.2,1.2,2.45,3.82};
  parameter Real stripHalfBeam[7](each min=.001,each unit="m")={.53,.64,.67,.65,.59,.46,.02};
  parameter Real displacementFraction[6](each min=.001)={.15,.20,.23,.21,.15,.06};
  parameter Real stripOffset(min=.001,unit="m")=.55 "Port and starboard buoyancy lever arm";
  parameter Real planingOnset(min=0,unit="m/s")=4;
  parameter Real planingSpeed(min=planingOnset+.01,unit="m/s")=11;
  parameter Real quadraticDragDisplacement(min=0,unit="kg/m")=118;
  parameter Real quadraticDragPlaning(min=0,unit="kg/m")=10.9;
  parameter Real linearDragDisplacement(min=0,unit="kg/s")=180;
  parameter Real linearDragPlaning(min=0,unit="kg/s")=38;
  parameter Real initialEast(unit="m")=0,initialNorth(unit="m")=0,initialHeading(unit="rad")=0;

  input Real throttleLever=0 "Astern -1, neutral 0, ahead +1";
  input Real helmLever=0 "Port-positive helm";
  input Real meanSeaLevel(unit="m")=0;
  input Real waterVelocityENU[3](each unit="m/s")=zeros(3) "Water velocity at the vessel";
  input Real sectionWaterElevation[2,6](each unit="m")=zeros(2,6)
    "Port and starboard wave elevation relative to meanSeaLevel at the section locations";
  input Real driveWaterElevation[2](each unit="m")=zeros(2)
    "Wave elevation relative to meanSeaLevel at the propeller and rudder";
  input Real externalWrench[6]=zeros(6) "ENU force and moment about the hull origin";

  output Real boatPose[6] "East, north, up, roll, pitch, heading; pitch is positive bow-down";
  output Real engine[4] "Shaft RPM, signed through-water surge speed, propeller thrust, rudder angle";
  output Real fuelState[6] "Litres remaining, capacity, litres/hour, and tank centroid";
  output Real mass(unit="kg");
  output Real fuelMass(unit="kg");
  output Real engineIdleTime(unit="s");
  output Real starterRemaining(unit="s");
  output Real engineRunning;
  output Real planingFraction "Surge-resistance blend into the planing regime";
  output Real sectionPointsENU[2,6,3];
  output Real drivePointsENU[2,3] "Propeller disk center and rudder reference point";
  output Real propellerWetFraction;
  output Real velocityENU[3](each unit="m/s");
  output Real rotationBodyToENU[3,3];
  output Real angularVelocityENU[3](each unit="rad/s") "Small-angle roll and pitch approximation for applied loads";

  EngineFuelSystem powerplant(capacity=fuelCapacity,tankCenter=fuelCenter,initialFuelLitres=initialFuelLitres,
    fuelDensity=fuelDensity,idleFuelRate=idleFuelRate,fullFuelRate=fullFuelRate,engineOffAfter=engineOffAfter,
    engineCrankTime=engineCrankTime,throttleLever=throttleLever);
  Marine.Propulsion.CruiserPropulsion propulsion(waterDensity=waterDensity,shaftRevolutions=shaftRPM/60,
    helm=helmLever,upInBody=rotationBodyToENU[3,:],
    propellerInflow=waterRelativeBody+cross(transpose(rotationBodyToENU)*angularVelocityENU,propulsion.propellerCenter),
    rudderInflow=waterRelativeBody+cross(transpose(rotationBodyToENU)*angularVelocityENU,propulsion.rudderCenter),
    propellerDepth=meanSeaLevel+driveWaterElevation[1]-drivePointsENU[1,3],
    rudderDepth=meanSeaLevel+driveWaterElevation[2]-drivePointsENU[2,3]);

protected
  constant Real gravity(unit="m/s2")=9.80665;
  final parameter Real stripX[6]={.5*(hullStations[j]+hullStations[j+1]) for j in 1:6};
  final parameter Real stripArea[6]={(hullStations[j+1]-hullStations[j])*
    (stripHalfBeam[j]+stripHalfBeam[j+1]) for j in 1:6};
  final parameter Real floatX=sum(stripArea.*stripX)/sum(stripArea);
  final parameter Real nominalVolumes[6]=.5*referenceMass/waterDensity*displacementFraction/sum(displacementFraction);
  Real east(start=initialEast,fixed=true),north(start=initialNorth,fixed=true),up;
  Real roll(start=0,fixed=true),pitch(start=0,fixed=true),heading(start=initialHeading,fixed=true);
  Real surge(start=0,fixed=true),sway(start=0,fixed=true),heave(start=0,fixed=true);
  Real rollRate(start=0,fixed=true),pitchRate(start=0,fixed=true),yawRate(start=0,fixed=true);
  Real shaftRPM(start=0,fixed=true);
  Real heelPitch[3,3],yawRotation[3,3],waterRelativeBody[3];
  Real driveForce[3],driveMoment[3],forceHeading[3],momentHeading[3],fuelMoment[3];
  Real sectionLift[2,6],fuelMassChange,effectiveThrottle,quadraticDrag,linearDrag;

  function immersedVolume "V-section displacement below the chine and linear topsides above it"
    input Real referenceVolume,waterplane,immersion;
    output Real volume;
  protected
    Real draft,wet;
  algorithm
    draft:=2*referenceVolume/waterplane;
    wet:=max(0,draft+immersion);
    volume:=min(2.5*referenceVolume,if noEvent(immersion<0) then referenceVolume*(wet/draft)^2
      else referenceVolume+waterplane*immersion);
  end immersedVolume;

initial equation
  up=meanSeaLevel;

equation
  yawRotation=[cos(heading),-sin(heading),0;sin(heading),cos(heading),0;0,0,1];
  heelPitch=[cos(pitch),sin(pitch)*sin(roll),sin(pitch)*cos(roll);
    0,cos(roll),-sin(roll);
    -sin(pitch),cos(pitch)*sin(roll),cos(pitch)*cos(roll)];
  rotationBodyToENU=yawRotation*heelPitch;
  angularVelocityENU=yawRotation*{rollRate*cos(pitch),pitchRate,yawRate-rollRate*sin(pitch)};
  velocityENU=yawRotation*{surge,sway,heave}+waterVelocityENU;
  waterRelativeBody=transpose(heelPitch)*{surge,sway,heave};
  der(east)=velocityENU[1];
  der(north)=velocityENU[2];
  der(up)=velocityENU[3];
  der(roll)=rollRate;
  der(pitch)=pitchRate;
  der(heading)=yawRate;

  for h in 1:2 loop
    for j in 1:6 loop
      sectionPointsENU[h,j,:]={
        east+cos(heading)*stripX[j]*cos(pitch)-sin(heading)*(3-2*h)*stripOffset*cos(roll),
        north+sin(heading)*stripX[j]*cos(pitch)+cos(heading)*(3-2*h)*stripOffset*cos(roll),
        up-(stripX[j]-floatX)*sin(pitch)+(3-2*h)*stripOffset*sin(roll)*cos(pitch)};
      sectionLift[h,j]=waterDensity*gravity*(immersedVolume(nominalVolumes[j],stripArea[j],
        meanSeaLevel+sectionWaterElevation[h,j]-sectionPointsENU[h,j,3])-nominalVolumes[j]);
    end for;
  end for;
  drivePointsENU[1,:]={east,north,up}+rotationBodyToENU*propulsion.propellerCenter;
  drivePointsENU[2,:]={east,north,up}+rotationBodyToENU*propulsion.rudderCenter;

  fuelMass=powerplant.fuelMass;
  fuelMassChange=fuelMass-fuelCapacity*fuelDensity/1000;
  mass=referenceMass+fuelMassChange;
  fuelMoment=cross(heelPitch*fuelCenter,{0,0,-gravity*fuelMassChange});
  effectiveThrottle=powerplant.engineRunning*powerplant.throttle;
  der(shaftRPM)=((if effectiveThrottle>=0 then shaftRpmAhead else shaftRpmAstern)*effectiveThrottle-shaftRPM)/shaftTimeConstant;
  driveForce=heelPitch*(propulsion.propellerForce+propulsion.rudderForce);
  driveMoment=heelPitch*(cross(propulsion.propellerCenter,propulsion.propellerForce)+
    cross(propulsion.rudderCenter,propulsion.rudderForce)+propulsion.shaftReaction);
  forceHeading=driveForce+transpose(yawRotation)*externalWrench[1:3];
  momentHeading=driveMoment+transpose(yawRotation)*externalWrench[4:6]+fuelMoment;
  planingFraction=noEvent(max(0,min(1,(surge-planingOnset)/(planingSpeed-planingOnset))));
  quadraticDrag=quadraticDragDisplacement*(1-planingFraction)+quadraticDragPlaning*planingFraction;
  linearDrag=linearDragDisplacement*(1-planingFraction)+linearDragPlaning*planingFraction;

  der(surge)=sway*yawRate+(forceHeading[1]-quadraticDrag*surge*abs(surge)-linearDrag*surge)/mass;
  der(sway)=-surge*yawRate+forceHeading[2]/mass-(1.5+.45*abs(sway))*sway;
  der(heave)=(sum(sectionLift)+forceHeading[3]-gravity*fuelMassChange-16500*heave)/(mass+addedHeaveMass);
  der(rollRate)=(.68*stripOffset*sum(sectionLift[1,:]-sectionLift[2,:])+momentHeading[1]
    -7400*rollRate-1950*surge*yawRate)/(rollInertia+fuelMassChange*(fuelCenter[2]^2+fuelCenter[3]^2));
  der(pitchRate)=(-sum((stripX[j]-floatX)*(sectionLift[1,j]+sectionLift[2,j]) for j in 1:6)+momentHeading[2]-47000*pitchRate)
    /(pitchInertia+fuelMassChange*(fuelCenter[1]^2+fuelCenter[3]^2));
  der(yawRate)=-yawRate/.9+momentHeading[3]/(yawInertia+fuelMassChange*(fuelCenter[1]^2+fuelCenter[2]^2));
  boatPose={east,north,up,roll,pitch,heading};
  engine={shaftRPM,surge,propulsion.propellerThrust,propulsion.rudderAngle};
  fuelState=powerplant.fuelState;
  engineIdleTime=powerplant.idleTime;
  starterRemaining=powerplant.starterRemaining;
  engineRunning=powerplant.engineRunning;
  propellerWetFraction=propulsion.propellerWetFraction;

  annotation(Documentation(info="<html><p>A reduced-order 7.6 m inboard cruiser. A fixed-angle shaft and immersed rudder use the open-water propulsion model. Two rows of six V-section strips provide heave and restoring moments. An empirical axial resistance changes between the displacement and planing regimes.</p><p>The world supplies water elevations and any contact, mooring or other external wrench. Hull coordinates are forward, port and up. The six-component pose follows the OceanMotorboat convention, with pitch positive bow-down.</p></html>"));
end InboardPlaningCruiser;
