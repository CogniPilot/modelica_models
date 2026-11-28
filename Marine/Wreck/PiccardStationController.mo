within Marine.Wreck;
block PiccardStationController "Sampled current-compensated position hold for two axial and four vertical motors"
  import Modelica.Constants.g_n;
  import Modelica.Constants.pi;
  import Modelica.Blocks.Interfaces.RealInput;
  import Modelica.Blocks.Interfaces.RealOutput;
  parameter Real dt(unit="s",min=Modelica.Constants.small)=.02 "Controller sample period";
  parameter Real dryMass(unit="kg",min=0)=18;
  parameter Real displaced(unit="m3",min=0)=.0184 "Sealed displacement when immersed";
  parameter Real density(unit="kg/m3",min=Modelica.Constants.small)=1025;
  parameter Real addedMass[3](each unit="kg",each min=0)={3,9,11} "Body-axis added masses";
  parameter Real linearDrag(unit="N.s/m",min=0)=1.5;
  parameter Real surgeDrag(unit="N.s2/m2",min=0)=2;
  parameter Real swayDrag(unit="N.s2/m2",min=0)=5.75;
  parameter Real feedforwardGain(min=0,max=1)=1;
  parameter Real minVectorForce(unit="N",min=Modelica.Constants.small)=2
    "Minimum effective vertical load used for the bank target";
  parameter Real ballastReserve(unit="m3",min=0)=.00020 "Additional piston volume per tank in sustained current";
  parameter Real tankCapacity(unit="m3",min=0)=.0008 "Maximum volume per piston tank";
  parameter Real arm(unit="m",min=Modelica.Constants.small)=.16 "Lateral lever arm of each motor pair";
  parameter Real foreX(unit="m",min=Modelica.Constants.small)=.0325 "Forward vertical pair position in body coordinates";
  parameter Real aftX(unit="m",max=-Modelica.Constants.small)=-.215 "Aft vertical pair position in body coordinates";
  parameter Real motorLimit(unit="N",min=Modelica.Constants.small)=14 "Symmetric thrust limit per motor";
  RealInput active "Above 0.5 while position control is engaged";
  RealInput pose[3](each unit="m") "Measured east, north and pressure depth, positive down";
  RealInput attitude[3](each unit="rad") "Estimated roll, pitch and heading in the FLU convention";
  RealInput gyro[3](each unit="rad/s") "High-rate body angular velocity";
  RealInput groundSpeed[3](each unit="m/s") "Forward and port in the level heading frame, up in ENU";
  RealInput bottomLock "Above 0.5 when horizontal velocity is valid";
  RealInput currentENU[2](each unit="m/s") "Estimated east and north water velocities";
  RealInput pilot[4] "Forward speed, port speed, yaw rate, up speed in m/s, m/s, rad/s and m/s";
  RealInput pitchReference(unit="rad") "Requested FLU pitch, positive nose down";
  RealInput netWeight(unit="N") "Estimated weight minus buoyancy, positive down";
  RealInput restoringMoment[3](each unit="N.m") "Estimated hydrostatic moment in body axes";
  RealOutput motors[6](each unit="N");
  RealOutput tankTargets[2](each unit="m3") "Fore and aft piston volume targets";
  RealOutput targetRoll(unit="rad") "Target bank in the FLU convention";
  RealOutput forceTarget[3](each unit="N") "Forward, port and up forces in the level heading frame";
protected
  discrete Real sampleState[26](each start=0,each fixed=true);
  function stationSample "One controller sample including its retained state"
    input Real prev[26];
    input Real active,pose[3],attitude[3],gyro[3],groundSpeed[3],bottomLock,currentENU[2],pilot[4];
    input Real pitchReference,netWeight,restoringMoment[3];
    input Real dryMass,displaced,density,addedMass[3],linearDrag,surgeDrag,swayDrag,arm,foreX,aftX,motorLimit,feedforwardGain,dt,minVectorForce,ballastReserve,tankCapacity;
    output Real nextSample[26];
  protected
    Real station[3],bias[3],yawRef,motors[6],tankTargets[2],targetRoll,forceTarget[3];
    Boolean engaged;
    Real yawLead,cs,sn,holdE,holdN,depthError,relativeSurge,relativeSway;
    Real axialDemand,uprightDemand,surgeSpeed,swaySpeed,upSpeed,surge,sway,vertical,heave,bank,trim,waterSpeed;
    Real rollTorque,pitchTorque,yawTorque,rawMotors[6];
    Real cosBank,sinBank,cosPitch,sinPitch,headingRate,pitchRate,headingMoment,pitchMoment,headingNow;
    Real upBody[3],steepWeight,poleYawRate,steepMoment[3],headingPredicted,compassOffset,poleRecovery;
    Real yawDifferential,axialRoom,axial,rollDifferential,pitchDifferential,attitudeScale;
    Real foreRatio,aftRatio,minHeave,maxHeave,fore,aft;
    Real waterForward,waterPort,groundUp,relBody[3],hydroMoment[3];
    Boolean moving,horizontalValid;
  algorithm
    engaged:=active>.5;
    moving:=abs(pilot[1])>.025 or abs(pilot[2])>.01;
    horizontalValid:=bottomLock>.5;
    for i in 1:3 loop
      // Track the measured pose during travel and capture it on the first
      // neutral sample, independently for depth and horizontal position.
      station[i]:=if not engaged or prev[8]<.5 or
        (if i==3 then abs(pilot[4])>.025 or prev[24]>.5 else moving or prev[23]>.5)
        then pose[i] else prev[i];
    end for;
    holdE:=station[1]-pose[1]; holdN:=station[2]-pose[2]; depthError:=pose[3]-station[3];
    for i in 1:3 loop
      bias[i]:=if not engaged or prev[8]<.5 then 0
        elseif (if i==3 then abs(pilot[4])>.025 else moving) then prev[3+i]*(1-dt/.8)
        elseif i<3 and not horizontalValid then prev[3+i]
        else max(-5,min(5,prev[3+i]+.35*dt*(if i==3 then depthError else station[i]-pose[i])));
    end for;
    // Allow for the motor and attitude response during a deliberate change of heading.
    yawLead:=attitude[3]+.35*pilot[3];
    cs:=cos(yawLead); sn:=sin(yawLead);
    surgeSpeed:=if moving then pilot[1] else max(-.55,min(.55,.8*(holdE*cs+holdN*sn)));
    swaySpeed:=if moving then pilot[2] else max(-.4,min(.4,.8*(-holdE*sn+holdN*cs)));
    upSpeed:=if abs(pilot[4])>.025 then pilot[4] else max(-.4,min(.4,.8*depthError));
    relativeSurge:=surgeSpeed-feedforwardGain*(currentENU[1]*cs+currentENU[2]*sn);
    relativeSway:=swaySpeed+feedforwardGain*(currentENU[1]*sn-currentENU[2]*cs);
    surge:=linearDrag*relativeSurge+surgeDrag*relativeSurge*abs(relativeSurge)+bias[1]*cs+bias[2]*sn
      +(if horizontalValid then 18*(surgeSpeed-groundSpeed[1]) else 0);
    sway:=linearDrag*relativeSway+swayDrag*relativeSway*abs(relativeSway)-bias[1]*sn+bias[2]*cs
      +(if horizontalValid then 12*(swaySpeed-groundSpeed[2]) else 0);
    vertical:=netWeight+bias[3]+24*(upSpeed-groundSpeed[3]);
    forceTarget:={surge,sway,vertical};
    // Resolve commanded force in the pitched frame before banking to produce sway.
    cosBank:=cos(attitude[1]); sinBank:=sin(attitude[1]);
    cosPitch:=cos(attitude[2]); sinPitch:=sin(attitude[2]);
    axialDemand:=cosPitch*surge-sinPitch*vertical;
    uprightDemand:=sinPitch*surge+cosPitch*vertical;
    bank:=-max(-.80,min(.80,atan2(sway,max(minVectorForce,abs(uprightDemand)))))*uprightDemand/sqrt(uprightDemand*uprightDemand+.04);
    targetRoll:=bank;
    heave:=uprightDemand/max(.55,cosBank);
    upBody:={-sinPitch,sinBank*cosPitch,cosBank*cosPitch};
    steepWeight:=max(0,min(1,(abs(attitude[2])-pi/3)/(pi/12)));
    poleYawRate:=(upBody[1]*gyro[1]+upBody[3]*gyro[3])/max(.1,1-upBody[2]^2);
    // Use one observable yaw axis throughout a vertical maneuver. Wait for
    // level attitude and two full navigator samples before changing references.
    poleRecovery:=if not engaged then 0
      elseif abs(attitude[2])>pi/3 or abs(pitchReference)>pi/3 then .6
      elseif abs(attitude[2])<pi/18 and abs(gyro[2])<.1 then max(0,prev[26]-dt)
      else prev[26];
    headingRate:=if poleRecovery>0 then poleYawRate
      else (sinBank*gyro[2]+cosBank*gyro[3])/max(.4,cosPitch);
    pitchRate:=cosBank*gyro[2]-sinBank*gyro[3];
    // At the pole the forward-axis bearing is undefined. Carry the observable
    // port-axis azimuth through the maneuver. Reconcile its reference with
    // the navigator before resuming the ordinary 5 Hz heading corrections.
    headingPredicted:=prev[22]+dt*headingRate;
    compassOffset:=if not engaged or prev[8]<.5 then 0
      elseif poleRecovery>0 or prev[26]>0 then headingPredicted-attitude[3] else prev[25];
    headingNow:=if not engaged or prev[8]<.5 then attitude[3]
      elseif poleRecovery<=0 and abs(attitude[3]-prev[21])>1e-9 then attitude[3]+compassOffset
      else headingPredicted;
    yawRef:=if not engaged or prev[8]<.5 or abs(pilot[3])>.015 then headingNow else prev[7];
    headingMoment:=7*(pilot[3]-headingRate)+11*atan2(sin(yawRef-headingNow),cos(yawRef-headingNow));
    pitchMoment:=10*(pitchReference-attitude[2])-4*pitchRate;
    // Translating through a beam current produces a Munk moment on this
    // elongated hull. Estimate it from onboard velocity/current measurements.
    waterForward:=(if horizontalValid then groundSpeed[1] else surgeSpeed)
      -feedforwardGain*(currentENU[1]*cos(attitude[3])+currentENU[2]*sin(attitude[3]));
    waterPort:=(if horizontalValid then groundSpeed[2] else swaySpeed)
      -feedforwardGain*(-currentENU[1]*sin(attitude[3])+currentENU[2]*cos(attitude[3]));
    groundUp:=groundSpeed[3];
    relBody:={cosPitch*waterForward-sinPitch*groundUp,
      sinBank*sinPitch*waterForward+cosBank*waterPort+sinBank*cosPitch*groundUp,
      cosBank*sinPitch*waterForward-sinBank*waterPort+cosBank*cosPitch*groundUp};
    hydroMoment:=cross(relBody,addedMass.*relBody);
    rollTorque:=9*(targetRoll-attitude[1])-3.6*(gyro[1]+sinPitch*headingRate)
      -sinPitch*headingMoment-restoringMoment[1]+hydroMoment[1];
    pitchTorque:=cosBank*pitchMoment+sinBank*cosPitch*headingMoment-restoringMoment[2]+hydroMoment[2];
    yawTorque:=-sinBank*pitchMoment+cosBank*cosPitch*headingMoment-restoringMoment[3]+hydroMoment[3];
    steepMoment:=steepTiltMoment(upBody,targetRoll,pitchReference,gyro,headingMoment)-restoringMoment+hydroMoment;
    rollTorque:=(1-steepWeight)*rollTorque+steepWeight*steepMoment[1];
    pitchTorque:=(1-steepWeight)*pitchTorque+steepWeight*steepMoment[2];
    yawTorque:=(1-steepWeight)*yawTorque+steepWeight*steepMoment[3];
    // A common axial command spends only the thrust left after differential yaw.
    yawDifferential:=max(-motorLimit,min(motorLimit,yawTorque/(2*arm)));
    axialRoom:=motorLimit-abs(yawDifferential);
    axial:=max(-axialRoom,min(axialRoom,axialDemand/2));
    // Preserve attitude moments as a pair. Collective can use the remaining
    // headroom on each motor pair without inducing an unwanted pitch moment.
    rollDifferential:=rollTorque/(4*arm);
    pitchDifferential:=-pitchTorque/(2*(foreX-aftX));
    attitudeScale:=min(1,motorLimit/max(.0001,abs(rollDifferential)+abs(pitchDifferential)));
    rollDifferential:=rollDifferential*attitudeScale;
    pitchDifferential:=pitchDifferential*attitudeScale;
    foreRatio:=-aftX/(2*(foreX-aftX)); aftRatio:=foreX/(2*(foreX-aftX));
    minHeave:=max((-motorLimit+abs(rollDifferential)-pitchDifferential)/foreRatio,
                  (-motorLimit+abs(rollDifferential)+pitchDifferential)/aftRatio);
    maxHeave:=min((motorLimit-abs(rollDifferential)-pitchDifferential)/foreRatio,
                  (motorLimit-abs(rollDifferential)+pitchDifferential)/aftRatio);
    heave:=max(minHeave,min(maxHeave,heave));
    fore:=foreRatio*heave+pitchDifferential; aft:=aftRatio*heave-pitchDifferential;
    rawMotors:={axial-yawDifferential,axial+yawDifferential,
      fore+rollDifferential,fore-rollDifferential,aft+rollDifferential,aft-rollDifferential};
    for i in 1:6 loop
      motors[i]:=if engaged then max(-motorLimit,min(motorLimit,rawMotors[i])) else 0;
    end for;
    // At zero lateral demand trim for neutral buoyancy. Under sustained beam
    // current, admit enough water to vector upward thrust at a moderate bank.
    waterSpeed:=sqrt(currentENU*currentENU);
    trim:=(displaced-dryMass/density)/2+min(.00030,ballastReserve*waterSpeed/(waterSpeed+.12)
      +abs(sway)/(2*density*g_n*tan(.45)))-.00010*depthError;
    tankTargets:={max(0,min(tankCapacity,trim-.0002*(attitude[2]-pitchReference))),
      max(0,min(tankCapacity,trim+.0002*(attitude[2]-pitchReference)))};
    nextSample:=cat(1,station,bias,{yawRef,if engaged then 1 else 0},motors,tankTargets,{targetRoll},forceTarget,{attitude[3],headingNow,if moving then 1 else 0,if abs(pilot[4])>.025 then 1 else 0,compassOffset,poleRecovery});
  end stationSample;
equation
  motors=sampleState[9:14];
  tankTargets=sampleState[15:16];
  targetRoll=sampleState[17];
  forceTarget=sampleState[18:20];
  when sample(0,dt) then
    sampleState=stationSample(pre(sampleState),active,pose,attitude,gyro,groundSpeed,bottomLock,currentENU,
      pilot,pitchReference,netWeight,restoringMoment,dryMass,displaced,density,addedMass,linearDrag,surgeDrag,swayDrag,
      arm,foreX,aftX,motorLimit,feedforwardGain,dt,minVectorForce,ballastReserve,tankCapacity);
  end when;
  annotation(Documentation(info="<html><p>Position hold for a vehicle with two horizontal axial motors and two fore/aft vertical pairs. Each pair is ordered port then starboard. The axial pair pushes forward and the vertical pairs push up for positive thrust.</p><p>The controller captures each neutral axis independently, uses measured current for drag feedforward, and varies bank to generate lateral force from vertical thrust. The attitude allocator reserves motor headroom before assigning collective thrust. It also carries an observable heading through near-vertical pitch.</p><p>Supply an independently estimated pose, current, and velocity. A zero bottom-lock input bypasses horizontal velocity feedback and holds horizontal integral bias. Disengaging sets motor targets to zero and leaves tank targets available for the caller to accept or override.</p><p>See <a href=\"modelica://Marine.Wreck.PiccardStationHoldExample\">PiccardStationHoldExample</a> for an isolated controller exercise.</p></html>"));
end PiccardStationController;
