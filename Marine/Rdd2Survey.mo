within Marine;
model Rdd2Survey "CogniPilot RDD2 hovering over the wreck survey area"
  input Real windENU[3] "Wind velocity at the aircraft [m/s]";
  parameter Real bedHeight=-13.71, displacedVolume=.0012 "Flooded vehicle displacement [m3]";
  parameter Real waterDragArea=.075,bodyHeight=.16;
  input Real motorCut,waterLevel,currentENU[3];
  output Real immersed;
  Real waterSlip[3],waterLoad[3];
  Vehicles.Rdd2.Plant aircraft(initialGroundClearance_m=4.9-bedHeight,groundAltitude=bedHeight,
    windWorld=windENU,exposedFraction=1-immersed,fluidForceWorld=waterLoad+deckWrench[1:3],contactMomentWorld=deckWrench[4:6]);

  input Real deckEnabled,deckState[12];
  output Real deckWrench[12] "World force and torque on drone then launch";
  function landingContact
    input Real enabled,deck[12],p[3],quat[4],v[3],bodyRate[3];
    output Real result[12];
  protected
    Real A[3,3],B[3,3],r[3],delta[3],w[3],f[3],m[3],fb[3],mb[3],y,a,b;
    constant Real centers[3]={-1.98,1.54,2.42},lengths[3]={.98,.55,.34},widths[3]={.86,.67,.32};
    MultiBodyLie.Collision.Material rubber(k=2600,d=5,mu=.9,eps=.0003,v_eps=.08);
    MultiBodyLie.Collision.ContactPoint pt;
  algorithm
    result:=zeros(12);delta:=p-deck[1:3];
    if enabled>.5 and delta*delta<16 then
      A:=MultiBodyLie.Math.quaternionToT(quat);
      w:=transpose(A)*bodyRate;
      y:=deck[4];a:=deck[5];b:=-deck[6];
      B[1,:]:={cos(y)*cos(b),sin(y)*cos(b),-sin(b)};
      B[2,:]:={cos(y)*sin(b)*sin(a)-sin(y)*cos(a),sin(y)*sin(b)*sin(a)+cos(y)*cos(a),cos(b)*sin(a)};
      B[3,:]:=cross(B[1,:],B[2,:]);
      for foot in 1:4 loop
        r:=transpose(A)*{if foot<=2 then .17 else -.17,if mod(foot,2)==0 then .17 else -.17,-.085};
        for section in 1:3 loop
          pt:=MultiBodyLie.Collision.sphereBox(p,r,.015,deck[1:3],transpose(B)*{centers[section],0,.66},B,{lengths[section],widths[section],.12});
          if pt.gap<.004 then
            (f,m,fb,mb):=MultiBodyLie.Collision.pairWrenches(pt,v,w,deck[7:9],deck[10:12],rubber);
            result:=result+cat(1,f,m,fb,mb);
          end if;
        end for;
      end for;
    end if;
  end landingContact;
  output Real dragBody[3]=aircraft.aerodynamicDrag;
  parameter Real origin[2]={-5,0} "Survey launch coordinates ENU";
  input Real pilot,forward,left,climb,turn,stationEast,stationNorth;
  output Real position[3],quaternion[4],rotorSpeed[4];
  output Real rotorAngle[4](each start=0,each fixed=true);
  Real target[3](start={-5,0,5},each fixed=true),yawTarget(start=0,fixed=true);
  Real attitude[3](each start=0,each fixed=true);
  discrete Real gps[3](start={-5,0,5},each fixed=true),gpsVelocity[3](each start=0,each fixed=true);
  parameter Real holdIntegralGain=.36 "Position-loop integral gain [1/s3]";
  parameter Real maxIntegralAcceleration=6 "Bound on sustained disturbance rejection [m/s2]";
  parameter Real tiltLimit=.72 "Maximum roll or pitch for station keeping [rad]";
  Real integralAcceleration[3](each start=0,each fixed=true);
  Real forwardAcceleration,leftAcceleration,upAcceleration;
  Real desired[3],acceleration[3],rollTarget,pitchTarget,collective,torque[3],motorThrust[4];

equation
  deckWrench=landingContact(deckEnabled,deckState,position,quaternion,aircraft.truth.velocityWorldEnu_m_s,aircraft.truth.angularVelocityBodyFlu_rad_s);
  position=aircraft.truth.positionWorldEnu_m+{origin[1],origin[2],0};
  quaternion=aircraft.truth.quaternionWorldBody;
  immersed=min(1,max(0,.5+(waterLevel-position[3])/bodyHeight));
  waterSlip=currentENU-aircraft.truth.velocityWorldEnu_m_s;
  waterLoad=immersed*1025*({0,0,9.80665*displacedVolume}+.5*waterDragArea*sqrt(waterSlip*waterSlip+1e-12)*waterSlip);
  rotorSpeed=aircraft.motorOmega_rad_s;
  der(rotorAngle)={1,-1,1,-1}.*rotorSpeed;
  when sample(0,.1) then
    gps=position;
    gpsVelocity=aircraft.truth.velocityWorldEnu_m_s;
  end when;
  // Ideal 10 Hz GNSS plus body-rate gyro integration, initialized on a level pad.
  der(attitude[1])=aircraft.imu.angularVelocityBodyFlu_rad_s[1]+tan(attitude[2])*(sin(attitude[1])*aircraft.imu.angularVelocityBodyFlu_rad_s[2]+cos(attitude[1])*aircraft.imu.angularVelocityBodyFlu_rad_s[3]);
  der(attitude[2])=cos(attitude[1])*aircraft.imu.angularVelocityBodyFlu_rad_s[2]-sin(attitude[1])*aircraft.imu.angularVelocityBodyFlu_rad_s[3];
  der(attitude[3])=(sin(attitude[1])*aircraft.imu.angularVelocityBodyFlu_rad_s[2]+cos(attitude[1])*aircraft.imu.angularVelocityBodyFlu_rad_s[3])/cos(attitude[2]);
  der(yawTarget)=if pilot>.5 then turn else 0;
  desired=if pilot>.5 then {2*(forward*cos(yawTarget)-left*sin(yawTarget)),2*(forward*sin(yawTarget)+left*cos(yawTarget)),climb}
    else {max(-2,min(2,stationEast-target[1])),max(-2,min(2,stationNorth-target[2])),max(-1,min(1,5-target[3]))};
  der(target[1:2])=desired[1:2];
  der(target[3])=if target[3]<(if deckEnabled>.5 and (position[1]-deckState[1])^2+(position[2]-deckState[2])^2<16 then .35 else 1) and desired[3]<0 then 0 else desired[3];
  for i in 1:3 loop
    der(integralAcceleration[i])=if motorCut>.5 or immersed>.5 then -integralAcceleration[i]
      elseif (integralAcceleration[i]>=maxIntegralAcceleration and target[i]>gps[i]) or
             (integralAcceleration[i]<=-maxIntegralAcceleration and target[i]<gps[i]) then 0
      else holdIntegralGain*(target[i]-gps[i]);
  end for;
  acceleration=1.4*(target-gps)+2.4*(desired-gpsVelocity)+integralAcceleration;
  forwardAcceleration=cos(attitude[3])*acceleration[1]+sin(attitude[3])*acceleration[2];
  leftAcceleration=-sin(attitude[3])*acceleration[1]+cos(attitude[3])*acceleration[2];
  upAcceleration=max(1,9.80665+acceleration[3]);
  pitchTarget=max(-tiltLimit,min(tiltLimit,atan2(forwardAcceleration,upAcceleration)));
  rollTarget=max(-tiltLimit,min(tiltLimit,atan2(-leftAcceleration,sqrt(forwardAcceleration^2+upAcceleration^2))));
  collective=aircraft.vehicleMass_kg*(9.80665+acceleration[3])/max(.5,cos(attitude[1])*cos(attitude[2]));
  torque={.86*(rollTarget-attitude[1]),.86*(pitchTarget-attitude[2]),.42*atan2(sin(yawTarget-attitude[3]),cos(yawTarget-attitude[3]))}-
    {.23*aircraft.imu.angularVelocityBodyFlu_rad_s[1],.23*aircraft.imu.angularVelocityBodyFlu_rad_s[2],.18*aircraft.imu.angularVelocityBodyFlu_rad_s[3]};
  for k in 1:4 loop
    // The RDD2 effectiveness matrix has orthogonal rows. Use its actual motor order.
    motorThrust[k]=collective/4+sum(aircraft.rotorEffectiveness[j+1,k]*torque[j]/
      (aircraft.rotorEffectiveness[j+1,:]*aircraft.rotorEffectiveness[j+1,:]) for j in 1:3);
    aircraft.commands.motor[k]=if motorCut>.5 or immersed>.5 then 0 else sqrt(max(0,min(aircraft.omegaMax^2,motorThrust[k]/aircraft.thrustCoefficient_N_s2)))/aircraft.omegaMax;
  end for;
end Rdd2Survey;
