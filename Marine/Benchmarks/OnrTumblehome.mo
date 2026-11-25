within Marine.Benchmarks;
model OnrTumblehome "Model 5613 at Iowa basin scale; body axes forward, port, up"
  parameter Real L=3.147, mass=72.6, rho=998.6, g=9.80665;
  parameter Real diameter=0.1066, rudderArea=0.012, gm=0.0422, rollPeriod=1.665;
  parameter Real designSpeed=1.11, designRpm=538, wakeFraction=0.12, thrustDeduction=0.12;
  parameter Real xRudder=-1.40, zSide=-0.106, zRudder=-0.130;
  parameter Real stations[7]={-1.5235,-0.9934,-0.4633,0.0668,0.5969,1.1270,1.6571};
  parameter Real beamHalf[7]={.1409,.1780,.1920,.1920,.1736,.0851,0};
  parameter Real bareVolume[6]={.00478743,.01490140,.01810605,.01723612,.01127263,.00380672};
  parameter Real additionalYawDamping=.027 "Dimensionless fully appended correction fitted to SIMMAN 5.2.3 steady diameter only";
  parameter Real yawInertia=mass*(.246*L)^2;
  parameter Real rollInertia=mass*g*gm*(rollPeriod/6.283185307)^2;
  parameter Real pitchInertia=mass*(.246*L)^2;
  parameter Real rudderSpan=.1082 "Span of the moving blade from the IGES, m";
  parameter Real aspect=rudderSpan^2/rudderArea;
  parameter Real rudderSlope=6.13*aspect/(aspect+2.25) "Fujii normal-force slope";
  parameter Real flowStraightening=.60 "ONRT rudder inflow (Sadat-Hosseini et al., 2011)";
  parameter Real initialHeading=0,initialSurge=0,initialSway=0,initialYawRate=0;
  parameter Real initialHeave=0,initialPitch=0,initialRoll=0,initialW=0,initialQ=0,initialP=0,initialRpm=0;
  parameter Real initialPhase=0 "Incident wave phase at release [rad]";
  input Real shaftDemand=0 "Common inward shaft command [rpm]";
  input Real helm=0 "Positive turns to port [deg]";
  input Real tow=0 "1 prescribes straight track through still water";
  input Real towSpeed=1.11;
  input Real waveAmplitude=0, waveDirection=3.14159265 "Travel direction from east towards north [rad]";
  input Real wavelength=3.147, depth=3;
  input Real generatorStart=-100000 "Simulation time of the most recent wavemaker command [s]";
  input Real generatorStroke=0.10 "Peak-to-peak vertical plunger travel [m]";
  input Real generatorFrequency=0 "Hz, zero selects the incident-wave frequency";
  input Real generatorCycles=8, generatorRampCycles=2, generatorStop=1e9;
  parameter Real generatorX=34.5, tankLength=40, wedgeWidth=.36 "Effective wedge width at the waterline [m]";
  output Real plungerTravel, generatorProfile[641], trainDuration, driveDuration;
  Real groupSpeed, wavePeriod, trainAge, trainAmplitude, driveOmega, driveK, rampTime, stroke;
  Real generatedLevel[6],generatedSlope[6],generatedForce[6],centerLevel,centerSlope;
  input Real basinContact=0;
  parameter Real initialEast=0,initialNorth=0;
  output Real wallWrench[3] "Surge and sway force [N], yaw moment [N m]";
  output Real east(start=initialEast,fixed=true),north(start=initialNorth,fixed=true),heading(start=initialHeading,fixed=true);
  output Real surge(start=initialSurge,fixed=true),sway(start=initialSway,fixed=true),yawRate(start=initialYawRate,fixed=true);
  output Real heave(start=initialHeave,fixed=true),pitch(start=initialPitch,fixed=true),roll(start=initialRoll,fixed=true)
    "Heave up in m; pitch bow-up and roll port-up in rad";
  Real w(start=initialW,fixed=true),q(start=initialQ,fixed=true),p(start=initialP,fixed=true);
  output Real rpm(start=initialRpm,fixed=true),rudder(start=0,fixed=true) "RPM magnitude; helm angle [deg]";
  output Real thrust[2],torque[2],rudderSide[2],resistance,advanceRatio[2];
  output Real propAngle(start=0,fixed=true) "Inward rotation magnitude [rad]";
  output Real waveNumber,waveOmega,waveElevation;
  Real n,localU[2],rudderInflow[2],rudderAngle[2],rudderDrag[2];
  Real jetSpeed[2],jetRadius[2],jetHeight[2],lateralOffset[2],jetFraction[2];
  parameter Real rudderBottom=-.044781*L,rudderTop=-.010412*L;
  parameter Real propX=1.625-.9267*L;
  parameter Real jetZ=-.03565*L+(xRudder-propX)*tan(5*pi/180);
  output Real immersedVolume[6];
  Real relativeLevel[6],wavePhase[6],waveForce[6];
  Real x[6],area[6],stripForce[6];
  Real speed,speedScale,vPrime,rPrime,hullSide,hullYaw,referenceResistance,rollSlope;
  constant Real pi=3.141592653589793;
  function basinReaction "Compliant contact at the extreme wet hull point on each basin face"
    input Real e,n,psi,u,v,r,xs[7],beam[7],plunger;
    output Real wrench[3];
  protected
    Real penetration[4],arm[4,2],normal[4,2],d[4],offset[2],point[2],velocity[2],force[2],world[2];
    Real c=cos(psi),s=sin(psi),west,load,torque;
  algorithm
    // Inward normals of west, east, south and north faces.
    normal:=[1,0;-1,0;0,1;0,-1];penetration:=zeros(4);arm:=zeros(4,2);
    for i in 1:7 loop
      for side in -1:2:1 loop
        offset:={c*xs[i]-s*side*beam[i],s*xs[i]+c*side*beam[i]};
        point:={e,n}+offset;
        // Waterline intercept of the sloping beach and the working platforms.
        west:=if abs(point[2])>=2.42 and abs(point[2])<=6.08 then -3.32 else -4.95;
        d:={west-point[1],point[1]-(34.48-.60*(.54-plunger)/.86),-9.80-point[2],point[2]-9.80};
        for j in 1:4 loop
          if d[j]>penetration[j] then penetration[j]:=d[j];arm[j,:]:=offset; end if;
        end for;
      end for;
    end for;
    world:=zeros(2);torque:=0;
    for j in 1:4 loop
      if penetration[j]>0 then
        velocity:={c*u-s*v-r*arm[j,2],s*u+c*v+r*arm[j,1]};
        load:=max(0,22000*penetration[j]-1800*min(1,penetration[j]/.012)*(velocity*normal[j,:]));
        force:=load*normal[j,:]-.16*load*tanh((velocity[2]*normal[j,1]-velocity[1]*normal[j,2])/.05)
          *{-normal[j,2],normal[j,1]};
        world:=world+force;torque:=torque+arm[j,1]*force[2]-arm[j,2]*force[1];
      end if;
    end for;
    wrench:={c*world[1]+s*world[2],-s*world[1]+c*world[2],torque};
  end basinReaction;
  function dispersionK
    input Real omega,h;
    output Real k;
  protected
    Real th;
  algorithm
    k:=max(omega^2/9.80665,omega/sqrt(9.80665*h));
    for j in 1:5 loop
      th:=tanh(k*h);
      k:=k-(k*th-omega^2/9.80665)/(th+k*h*(1-th^2));
    end for;
  end dispersionK;
  function waveTrain "Linear regular-wave packet travelling west from the six synchronous plungers"
    input Real distance,age,k,omega,groupSpeed,duration,ramp,amplitude;
    output Real eta,slope "Elevation and derivative with respect to east position";
  protected
    Real arrival,envelope,rate,phase,rise,fall,riseAngle,fallAngle;
  algorithm
    arrival:=age-distance/groupSpeed;
    eta:=0; slope:=0;
    if distance>=0 and arrival>0 and arrival<duration then
      riseAngle:=pi*min(1,arrival/ramp);
      fallAngle:=pi*min(1,(duration-arrival)/ramp);
      rise:=.5*(1-cos(riseAngle)); fall:=.5*(1-cos(fallAngle));
      envelope:=rise*fall;
      rate:=pi/(2*ramp)*(sin(riseAngle)*fall-rise*sin(fallAngle));
      phase:=k*distance-omega*age;
      eta:=amplitude*envelope*cos(phase);
      slope:=amplitude*(rate/groupSpeed*cos(phase)+envelope*k*sin(phase));
    end if;
  end waveTrain;
  function curve "Uniformly spaced samples at J=0.1:0.1:1.7; clamp outside measurement"
    input Real j;
    input Real values[17];
    output Real k;
  protected
    Real a;
    Integer i;
  algorithm
    a:=max(0,min(16,10*j-1));
    i:=min(16,1+integer(floor(a)));
    k:=values[i]+(a+1-i)*(values[i+1]-values[i]);
  end curve;
  // IIHR Open_Water_EFD.dat. Torque keeps the original handed signs.
  constant Real ktP[17]={.601538,.593757,.551041,.536078,.493776,.453442,.396313,.350752,.324562,.263809,.216899,.172472,.123282,.080027,.025836,-.026849,-.100446};
  constant Real ktS[17]={.600193,.577550,.544662,.506766,.478168,.433448,.393055,.348293,.294021,.252986,.210002,.165227,.117383,.067843,.008305,-.043234,-.113359};
  constant Real kqP[17]={.146444,.144970,.134973,.132235,.122645,.113993,.101275,.090915,.085983,.073363,.063856,.054553,.043123,.032855,.021143,.009232,-.006759};
  constant Real kqS[17]={-.146387,-.141271,-.134044,-.125590,-.119666,-.110005,-.101523,-.092207,-.078191,-.071927,-.063319,-.053901,-.043396,-.032017,-.019163,-.008301,.007841};
equation
  der(rpm)=max(-300,min(300,(max(0,min(900,shaftDemand))-rpm)/0.08));
  der(rudder)=max(-35,min(35,(max(-35,min(35,helm))-rudder)/0.06));
  n=rpm/60;
  der(propAngle)=2*pi*n;
  for i in 1:2 loop
    localU[i]=(1-wakeFraction)*(surge-(3-2*i)*.02661*L*yawRate);
    advanceRatio[i]=localU[i]/max(.0001,n*diameter);
    thrust[i]=rho*n^2*diameter^4*(if i==1 then curve(advanceRatio[i],ktP) else curve(advanceRatio[i],ktS));
    torque[i]=rho*n^2*diameter^5*(if i==1 then curve(advanceRatio[i],kqP) else curve(advanceRatio[i],kqS));
    // Actuator-disk mass and momentum give the contracted jet radius.
    // Only the blade area crossing that jet sees the increased dynamic pressure.
    jetSpeed[i]=sqrt(localU[i]^2+2*max(0,thrust[i])/(rho*pi*diameter^2/4));
    jetRadius[i]=.5*diameter*sqrt(.5*(1+abs(localU[i])/max(.0001,jetSpeed[i])));
    lateralOffset[i]=(3-2*i)*(.020116-.02661)*L+.017*sin(rudder*pi/180);
    jetHeight[i]=sqrt(max(0,jetRadius[i]^2-lateralOffset[i]^2));
    jetFraction[i]=max(0,min(rudderTop,jetZ+jetHeight[i])-max(rudderBottom,jetZ-jetHeight[i]))/(rudderTop-rudderBottom);
    rudderInflow[i]=sqrt((1-jetFraction[i])*localU[i]^2+jetFraction[i]*jetSpeed[i]^2);
    rudderAngle[i]=rudder*pi/180+atan2(flowStraightening*(sway+xRudder*yawRate),max(.01,rudderInflow[i]));
    rudderSide[i]=-0.5*rho*rudderArea*(rudderInflow[i]^2+(flowStraightening*(sway+xRudder*yawRate))^2)*rudderSlope*
      sin(rudderAngle[i])*cos(rudder*pi/180);
    rudderDrag[i]=-rudderSide[i]*tan(rudder*pi/180);
  end for;
  // One-point self-propulsion calibration, encompassing viscous and wave resistance.
  referenceResistance=(1-thrustDeduction)*rho*(designRpm/60)^2*diameter^4*cos(5*pi/180)*
    (curve((1-wakeFraction)*designSpeed/(designRpm/60*diameter),ktP)+
     curve((1-wakeFraction)*designSpeed/(designRpm/60*diameter),ktS));
  resistance=referenceResistance*surge*abs(surge)/designSpeed^2;
  // Hashimoto et al. (2011), circular-motion tests, Table 2. Normalize
  // the polynomial inside the tested drift/yaw envelope at low headway.
  speed=sqrt(surge^2+sway^2);
  speedScale=max(.01,max(speed,max(abs(sway)/sin(15*pi/180),abs(yawRate)*L/.6)));
  vPrime=sway/speedScale;
  rPrime=yawRate*L/speedScale;
  // Dimensional linear terms scale with actual headway. Bound only the
  // higher-order extrapolation as a tight turn takes rL/U beyond the tests.
  hullSide=.5*rho*L*.112*(speed*(-.300*sway-.0932*yawRate*L)
    +speedScale^2*(-1.773*vPrime^3+.262*vPrime^2*rPrime-.800*vPrime*rPrime^2+.174*rPrime^3));
  hullYaw=.5*rho*L^2*.112*(speed*(-.0832*sway-(.0549+additionalYawDamping)*yawRate*L)
    +speedScale^2*(-.532*vPrime^3-.629*vPrime^2*rPrime-.139*vPrime*rPrime^2-.00446*rPrime^3));
  waveNumber=2*pi/max(.1,wavelength);
  waveOmega=sqrt(g*waveNumber*tanh(waveNumber*depth));
  driveOmega=if generatorFrequency>0 then 2*pi*min(2,max(.25,generatorFrequency)) else waveOmega;
  driveK=dispersionK(driveOmega,max(.1,depth));
  wavePeriod=2*pi/driveOmega;
  groupSpeed=.5*driveOmega/driveK*(1+2*driveK*depth/sinh(2*driveK*depth));
  trainAge=time-generatorStart;
  rampTime=wavePeriod*max(.25,min(generatorRampCycles,max(1,generatorCycles)/2));
  driveDuration=if generatorStart<0 then 0 else min(max(1,generatorCycles)*wavePeriod,
    max(0,generatorStop-generatorStart)+rampTime);
  trainDuration=driveDuration+tankLength/groupSpeed;
  // Approximate the documented stroke limits using constant peak speed above 0.62 Hz.
  stroke=max(0,min(generatorStroke,min(.25,.155*wavePeriod)));
  trainAmplitude=.5*stroke*wedgeWidth*driveK;
  plungerTravel=if trainAge>0 and trainAge<driveDuration then
    -.5*stroke*sin(driveOmega*trainAge)*.25*(1-cos(pi*min(1,trainAge/rampTime)))*
    (1-cos(pi*min(1,(driveDuration-trainAge)/rampTime))) else 0;
  for j in 1:641 loop
    generatorProfile[j]=waveTrain(tankLength*(641-j)/640,trainAge,driveK,driveOmega,groupSpeed,driveDuration,rampTime,trainAmplitude);
  end for;
  (centerLevel,centerSlope)=waveTrain(generatorX-east,trainAge,driveK,driveOmega,groupSpeed,driveDuration,rampTime,trainAmplitude);
  waveElevation=centerLevel+waveAmplitude*cos(waveNumber*(east*cos(waveDirection)+north*sin(waveDirection))-waveOmega*time+initialPhase);
  for i in 1:6 loop
    x[i]=(stations[i+1]+stations[i])/2;
    area[i]=(stations[i+1]-stations[i])*(beamHalf[i+1]+beamHalf[i]);
    wavePhase[i]=waveNumber*((east+x[i]*cos(heading))*cos(waveDirection)
      +(north+x[i]*sin(heading))*sin(waveDirection))-waveOmega*time+initialPhase;
    (generatedLevel[i],generatedSlope[i])=waveTrain(generatorX-east-x[i]*cos(heading),trainAge,
      driveK,driveOmega,groupSpeed,driveDuration,rampTime,trainAmplitude);
    generatedForce[i]=-rho*g*immersedVolume[i]*generatedSlope[i];
    relativeLevel[i]=generatedLevel[i]+waveAmplitude*cos(wavePhase[i])-heave-x[i]*pitch;
    immersedVolume[i]=max(0,bareVolume[i]*mass/(rho*sum(bareVolume))+area[i]*relativeLevel[i]);
    stripForce[i]=rho*g*area[i]*relativeLevel[i]-110*area[i]*(w+x[i]*q);
    waveForce[i]=rho*g*immersedVolume[i]*waveNumber*waveAmplitude*sin(wavePhase[i]);
  end for;
  wallWrench=if basinContact>.5 then basinReaction(east,north,heading,surge,sway,yawRate,stations,beamHalf,plungerTravel) else zeros(3);
  der(surge)=wallWrench[1]/(1.05*mass)+(if tow>.5 then (towSpeed-surge)/.5 else
    ((1-thrustDeduction)*cos(5*pi/180)*sum(thrust)-resistance-sum(rudderDrag)+sum(waveForce)*cos(waveDirection-heading)+sum(generatedForce)*cos(heading)+mass*sway*yawRate)/(1.05*mass));
  der(sway)=wallWrench[2]/(1.7*mass)+(if tow>.5 then -sway/.2 else
    (hullSide+sum(rudderSide)+sum(waveForce)*sin(waveDirection-heading)-sum(generatedForce)*sin(heading)-mass*surge*yawRate)/(1.7*mass));
  der(yawRate)=wallWrench[3]/(1.65*yawInertia)+(if tow>.5 then -yawRate/.2 else
    (sum(x[i]*waveForce[i] for i in 1:6)*sin(waveDirection-heading)-sum(x[i]*generatedForce[i] for i in 1:6)*sin(heading)+hullYaw+xRudder*sum(rudderSide)
      +.02661*L*(1-thrustDeduction)*(thrust[2]-thrust[1])*cos(5*pi/180))/(1.65*yawInertia));
  der(heading)=yawRate;
  der(east)=surge*cos(heading)-sway*sin(heading);
  der(north)=surge*sin(heading)+sway*cos(heading);
  der(heave)=w;
  der(w)=sum(stripForce)/(1.8*mass);
  der(pitch)=q;
  der(q)=sum(x[i]*stripForce[i] for i in 1:6)/(1.8*pitchInertia);
  rollSlope=-centerSlope*sin(heading)-waveNumber*waveAmplitude*sin(waveDirection-heading)*
    sin(waveNumber*(east*cos(waveDirection)+north*sin(waveDirection))-waveOmega*time+initialPhase);
  der(roll)=p;
  der(p)=(mass*g*gm*(rollSlope-roll)-2*.09*sqrt(rollInertia*mass*g*gm)*p
    -zSide*(hullSide)-zRudder*sum(rudderSide))/rollInertia;
end OnrTumblehome;
