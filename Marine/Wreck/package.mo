within Marine;
package Wreck "Submerged survey vehicle, wreck contact and sensor navigation"
  import Marine.Environment.SpectralSea;
  constant Real g=9.80665;
  constant Real rho=1025;
  constant Real pi=Modelica.Constants.pi;

  model Tank "Floodable piston tank, positive stroke admits seawater"
    parameter Real capacity=0.0008, maxFlow=0.000028, stallPressure=300000;
    input Real target, pressure;
    Real volume(stateSelect=StateSelect.always,start=0.00035,fixed=true), pumpFlow(stateSelect=StateSelect.always,start=0,fixed=true), mass;
  equation
    der(pumpFlow)=(max(-1,min(1,(target-volume)/0.00005))*maxFlow*(if target<volume then max(0,1-pressure/stallPressure) else 1)-pumpFlow)/0.18;
    der(volume)=if (volume<=0 and pumpFlow<0) or (volume>=capacity and pumpFlow>0) then 0 else pumpFlow;
    mass=rho*volume;
  end Tank;

  function stationReached
    input Real station[3], east, north, depth;
    output Boolean reached;
  algorithm
    reached:=abs(station[1]-east)<.32 and abs(station[2]-north)<.24 and abs(.29-depth-station[3])<.28;
  end stationReached;

  model Plant "Six-degree-of-freedom survey submarine with ballasting, actuation and contact"
    inner MultiBodyLie.World world(n={0,0,-1});
    parameter Real dryMass=18, displaced=0.0184;
    parameter Real drag[3]={8,23,30}, angularDrag[3]={2,3,3};
    MultiBodyLie.Parts.Body body(m=dryMass,I_11=0.32,I_22=1.6,I_33=1.7,
      angles_fixed=true,w_0_fixed=true,Q(each stateSelect=StateSelect.always),w_a(each stateSelect=StateSelect.always),r_0(each stateSelect=StateSelect.always,start={-5,0,-10},each fixed=true),v_0(each stateSelect=StateSelect.always,each fixed=true));
    MultiBodyLie.Forces.WorldForceAndTorque collision(resolveInFrame=MultiBodyLie.Types.ResolveInFrameB.world);
    MultiBodyLie.Forces.WorldForceAndTorque loads(resolveInFrame=MultiBodyLie.Types.ResolveInFrameB.frame_b);
    input Real windSpeed(start=11), windDirection(start=0.45);
    input Real hullSurfaceOffset=0, propSurfaceOffset[7]=zeros(7)
      "Additional local free-surface elevation at the hull and at each propeller [m]";
    Marine.Environment.WaterColumn ocean(pos=body.r_0,windSpeed=windSpeed,windDirection=windDirection);
    output Real waveHeight[SpectralSea.N,SpectralSea.N]=ocean.tile;
    output Real seaLevel=ocean.tide, windActual=ocean.wind;
    output Real specificForce[3]=loads.force/dryMass, rates[3]=body.w_a;
    output Real attitude[2](each start=0,each fixed=true), verticalSpeed(start=0,fixed=true);
    Real accelAtt[2], vzDvl;
    parameter Real addedMass[3]={3,9,11};
    output Real contactForce[3](each start=0,each fixed=true,each stateSelect=StateSelect.always), contactMoment[3](each start=0,each fixed=true,each stateSelect=StateSelect.always);
    Real contactTarget[3], momentTarget[3];
    Real netForce[3],effectiveMass[3];
    Tank fore, aft;
    input Real sensor[91];
    input Real navigationUp(unit="m",start=0) "Estimated vehicle height in ENU coordinates";
    input Real navigationUpValid(start=0) "Enable the external fused height estimate";
    input Real elevationRanges[100](each start=0) "Four elevation fans of 25 slant ranges, zero means no echo";
    input Real manual(start=0) "Enable pilot velocity setpoints above 0.5";
    input Real pilot[4](each start=0) "Body surge, port sway, yaw rate and upward speed [m/s,m/s,rad/s,m/s]";
    Real lateralThrust(start=0,fixed=true,stateSelect=StateSelect.always) "Reversible transverse thruster [N]";
    Real swayCommand;
    parameter Real propellerDiameter=0.17, thrustCoefficient=0.035, inceptionNumber=3.5;
    output Real propellerRPM[7], cavitation[7], appliedThrust[7];
    parameter Real propMount[7,3]=[0,.32,0;0,-.32,0;.4,.32,0;.4,-.32,0;-.4,.32,0;-.4,-.32,0;0,0,0];
    parameter Real propAxis[7,3]=[1,0,0;1,0,0;0,0,1;0,0,1;0,0,1;0,0,1;0,1,0];
    parameter Real propMomentArm[7,3]={cross(propMount[i,:],propAxis[i,:]) for i in 1:7};
    Real propPosition[7,3],propDiskHeight[7],diskCut[7],localSurface[7];
    output Real propWet[7];
    Real shaftDemand[7], tipDynamicPressure[7];
    output Real position[3]=body.r_0, orientation[4]=body.Q, velocity[3]=body.frame_a.v, yawRate=body.w_a[3], pressure=ocean.gaugePressure, current[3]=ocean.velocity, tankLitres[2]=1000*{fore.volume,aft.volume}, motor[6]=thrust;
    output Real nav[3](start={-5,0,0},each fixed=true) "East, north, yaw from onboard sensors";
    output Real cornerScore, visualWeight;
    discrete Real edge[6](each start=0,each fixed=true), Ji[6,6](each start=0,each fixed=true), Jj[6,6](each start=0,each fixed=true);
    discrete Boolean edgeValid(start=false,fixed=true);
    Real trackWeight;
    parameter Integer waypointCount=12;
    parameter Real waypoints[waypointCount,3]=[
      -1,0,-9.8; 3,0,-9.8; 6.5,0,-9.8; 6.5,-1.83,-9.41;
      11,-1.83,-9.41; 17.5,-1.83,-9.41; 17.5,0,-9.8; 17.5,1.83,-10.19;
      12,1.83,-10.19; 6.5,1.83,-10.19; 6.5,0,-9.8; -1.5,0,-9.8]
      "ENU survey stations at the stern, companionways and three deck passages";
    output Integer waypoint(start=1,fixed=true);
    output Boolean surveyDone(start=false,fixed=true);
    output Real surveyTelemetry[16]=cat(1,{waypoint,waypointCount,if surveyDone then 1 else 0},appliedThrust,buoy,contactForce)
      "Survey station, total, completion, applied motor forces, body buoyancy and world contact";
    Real goal[3],eastError,northError,verticalError,surgeRef,swayRef,depthEst,surge,yawCmd,heave,rollCmd,pitchCmd;
    Real goalDepth;
    Real cmd[6], thrust[6](each stateSelect=StateSelect.always,each start=0,each fixed=true), rel[3], up[3], buoy[3], ballast[3], wet;
    parameter Real airDensity=1.225, windageCdA[3]={0.12,0.32,0.25} "Exposed hull drag areas [m2]";
    parameter Real windageCenter[3]={0.04,0,0.12} "Centre of wind pressure in body axes [m]";
    output Real windForce[3] "Aerodynamic hull load in body axes [N]";
    Real apparentAir[3];
    Real heading,pitch,roll;
  equation
    connect(loads.frame_b,body.frame_a);
    connect(collision.frame_b,body.frame_a);
    collision.force=contactForce;
    collision.torque=contactMoment;

    accelAtt={atan2(sensor[87],sensor[88]),atan2(-sensor[86],sqrt(sensor[87]^2+sensor[88]^2))};
    der(attitude)={sensor[89],sensor[90]}+1.5*(accelAtt-attitude);
    vzDvl=-sum(sensor[1:4])/(4*cos(pi/8));
    der(verticalSpeed)=sensor[88]-g+(if sensor[63]>0.5 then 4*(vzDvl-verticalSpeed) else -0.2*verticalSpeed);
    // Rows 1..4 are signed DVL beam velocities, 5..8 slant ranges, 9..57 sonar.
    // Row 58 gyro, 59 pressure, 60..62 scan-matched planar pose, 63 lock.
    cornerScore=Vision.Features.FastCircleScore(sensor[68:83]);
    visualWeight=if sensor[85]>0.5 and cornerScore>6 and abs(sensor[84]-sensor[58])<0.16 then 0.22 else 0;
    // Hold one coherent pose factor between successive 5 Hz scan acquisitions.
    when sample(0,0.2) then
      (edge,Ji,Jj,edgeValid)=SLAM.PoseGraph.PGEdge({nav[1],nav[2],0},identity(3),
        {sensor[60],sensor[61],0},SLAM.PoseGraph.PGExp({0,0,sensor[62]-nav[3]}),zeros(3),identity(3));
    end when;
    trackWeight=if edgeValid and sensor[67]>=8 and edge[1]^2+edge[2]^2<1 then 1.8 else 0;
    der(nav[1])=sensor[64]*cos(nav[3])-sensor[65]*sin(nav[3])+trackWeight*edge[1];
    der(nav[2])=sensor[64]*sin(nav[3])+sensor[65]*cos(nav[3])+trackWeight*edge[2];
    der(nav[3])=(1-visualWeight)*sensor[58]+visualWeight*sensor[84]+trackWeight*edge[6];
    goal=waypoints[waypoint,:];
    eastError=goal[1]-sensor[60];
    northError=goal[2]-sensor[61];
    heading=sensor[62];
    depthEst=if navigationUpValid>.5 then .29-navigationUp else sensor[59]/(rho*g);
    goalDepth=if manual>.5 then 10.29 else .29-goal[3];
    verticalError=depthEst-goalDepth;
    // Waypoints advance only on measured arrival. Hold the final station.
    when sample(.2,.2) then
      waypoint=if manual<.5 and stationReached(waypoints[pre(waypoint),:],sensor[60],sensor[61],depthEst) then min(waypointCount,pre(waypoint)+1) else pre(waypoint);
      surveyDone=pre(surveyDone) or (pre(waypoint)==waypointCount and manual<.5 and stationReached(waypoints[pre(waypoint),:],sensor[60],sensor[61],depthEst));
    end when;
    // The hovering sub can traverse a hatch sideways and reverse out of the stern.
    // Keep the camera looking along the corridor while resolving guidance into body axes.
    surgeRef=max(-.65,min(.65,eastError*cos(heading)+northError*sin(heading)));
    swayRef=max(-.38,min(.38,-eastError*sin(heading)+northError*cos(heading)));
    surge=if manual>.5 then 18*(pilot[1]-sensor[64]) else 24*(surgeRef-sensor[64]);
    yawCmd=if manual>.5 then 4*(pilot[3]-sensor[58]) else -6*sin(heading)-3*sensor[58];
    swayCommand=if manual>.5 then 22*(pilot[2]-sensor[65]) else 26*(swayRef-sensor[65]);
    der(lateralThrust)=(max(-16,min(16,swayCommand))-lateralThrust)/0.14;
    heave=if manual>0.5 then 24*(pilot[4]-verticalSpeed) else 12*verticalError-15*verticalSpeed;
    roll=attitude[1];
    pitch=attitude[2];
    rollCmd=-3*roll-sensor[89];
    pitchCmd=-4*pitch-2*sensor[90];
    cmd={surge/2-yawCmd/0.64,surge/2+yawCmd/0.64,
         heave/4+rollCmd/1.28-pitchCmd/1.6,heave/4-rollCmd/1.28-pitchCmd/1.6,
         heave/4+rollCmd/1.28+pitchCmd/1.6,heave/4-rollCmd/1.28+pitchCmd/1.6};
    for i in 1:6 loop
      der(thrust[i])=(max(-14,min(14,cmd[i]))-thrust[i])/0.14;
    end for;
    shaftDemand=cat(1,thrust,{lateralThrust});
    for i in 1:7 loop
      propPosition[i,:]=body.r_0+transpose(body.frame_a.R.T)*propMount[i,:];
      localSurface[i]=propSurfaceOffset[i]+seaLevel+Perception.seaHeight(ocean.tile,propPosition[i,1],propPosition[i,2]);
      propDiskHeight[i]=.5*propellerDiameter*sqrt(max(0,1-(up*propAxis[i,:])^2));
      diskCut[i]=max(-1,min(1,(localSurface[i]-propPosition[i,3])/max(.001,propDiskHeight[i])));
      propWet[i]=.5+(asin(diskCut[i])+diskCut[i]*sqrt(max(0,1-diskCut[i]^2)))/pi;
      appliedThrust[i]=(propWet[i]+(1-propWet[i])*airDensity/rho)*shaftDemand[i];
      propellerRPM[i]=60*shaftDemand[i]/sqrt(rho*thrustCoefficient*propellerDiameter^4*sqrt(shaftDemand[i]^2+0.0001));
      tipDynamicPressure[i]=0.5*rho*(pi*propellerDiameter*propellerRPM[i]/60)^2;
      cavitation[i]=propWet[i]*max(0,min(1,1-(101325+ocean.gaugePressure-2300)/(inceptionNumber*max(1,tipDynamicPressure[i]))));
    end for;
    fore.pressure=ocean.gaugePressure;
    aft.pressure=ocean.gaugePressure;
    fore.target=max(0,min(fore.capacity,0.00042-0.00010*verticalError-0.0002*pitch));
    aft.target=max(0,min(aft.capacity,0.00042-0.00010*verticalError+0.0002*pitch));
    up=body.frame_a.R.T*{0,0,1};
    wet=max(0,min(1,(ocean.surface+hullSurfaceOffset-body.r_0[3]+0.16)/0.32));
    buoy=rho*g*displaced*wet*up;
    ballast=-g*(fore.mass+aft.mass)*up;
    rel=body.frame_a.v-body.frame_a.R.T*ocean.velocity;
    apparentAir=body.frame_a.R.T*({ocean.wind*cos(ocean.bearing),ocean.wind*sin(ocean.bearing),0}-body.v_0)
      -cross(body.w_a,windageCenter);
    windForce=0.5*airDensity*(1-wet)*windageCdA.*apparentAir.*abs(apparentAir);
    effectiveMass=fill(dryMass+fore.mass+aft.mass,3)+addedMass;
    loads.force=dryMass*(netForce ./ effectiveMass+g*up);
    (contactTarget,momentTarget)=structureContact(body.r_0,body.frame_a.R.T,body.v_0,body.w_a);
    der(contactForce)=(contactTarget-contactForce)/0.014;
    der(contactMoment)=(momentTarget-contactMoment)/0.014;
    netForce=windForce+buoy+ballast-dryMass*g*up-cross(body.w_a,addedMass.*rel)-wet*(3*rel+drag.*rel.*abs(rel))+transpose(propAxis)*appliedThrust;
    loads.torque=cross(windageCenter,windForce)+cross({0,0,0.055},buoy)+cross({0.34,0,-0.08},-fore.mass*g*up)+cross({-0.34,0,-0.08},-aft.mass*g*up)
      -angularDrag.*body.w_a-cross(rel,addedMass.*rel)+transpose(propMomentArm)*appliedThrust;
    annotation(experiment(StopTime=125,Interval=0.2,Tolerance=1e-5));
  end Plant;
end Wreck;
