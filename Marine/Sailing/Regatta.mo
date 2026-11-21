within Marine.Sailing;
model Regatta "Five-minute fleet starts and an equilateral triangular course"
  input Real utc "Unix seconds from the browser clock";
  input Real east,north;
  parameter Real windBearing=atan2(4,10) "Direction the prevailing wind travels, counterclockwise from east";
  parameter Real gateEast=-10,gateNorth=-6.5,lineLength=40,legLength=200,zone=2.5;
  final parameter Real upE=-cos(windBearing),upN=-sin(windBearing);
  final parameter Real rightE=upN,rightN=-upE "Starboard when facing the wind";
  output Real pinEast,pinNorth,committeeEast,committeeNorth,markEast[2],markNorth[2];
  input Real wakeTile[64,64],backgroundTile[32,32],driftEast,driftNorth,tide,swellHeight;
  parameter Real swellK=.34,swellBearing=1.04159;
  parameter Real launchMass=1150,waterplaneArea=7.2,rollInertia=650,pitchInertia=3100;
  final parameter Real stationX[4]={1.65,1.65,-1.65,-1.65},stationY[4]={.65,-.65,.65,-.65};
  final parameter Real draft=launchMass/(1025*waterplaneArea);
  output Real launchPose[3] "Elevation, roll and bow-up pitch of anchored launch [m,rad,rad]";
  Real heave(start=0,fixed=true),roll(start=0,fixed=true),pitch(start=0,fixed=true);
  Real heaveRate(start=0,fixed=true),rollRate(start=0,fixed=true),pitchRate(start=0,fixed=true);
  Real stationE[4],stationN[4],immersion[4],lift[4],stationRate[4];
  discrete Real water[7](each start=0,each fixed=true),priorWater[7](each start=0,each fixed=true);
  discrete Real stamp(start=0,fixed=true);
  Real fraction;
  model AnchorLine "Three tension-only links carrying two submerged lumped masses"
    input Real fairlead[3],fairleadVelocity[3],current[2];
    parameter Real bed=-13.7,anchorX=0,anchorY=0,initialTop[3]={0,0,0};
    parameter Real nodeMass=4,submergedWeight=3,stiffness=1600,damping=80,slack=1.5;
    final parameter Real anchor[3]={anchorX,anchorY,bed};
    final parameter Real restLength=(sqrt((initialTop-anchor)*(initialTop-anchor))+slack)/3;
    output Real endForce[3],path[4,3],tension[3];
    Real node[2,3],speed[2,3](each start=0,each fixed=true);
    Real velocity[4,3],delta[3,3],axis[3,3],distance[3],traction[3,3];
  initial equation
    for k in 1:2 loop
      node[k,:]=anchor+(initialTop-anchor)*k/3;
    end for;
  equation
    path[1,:]=anchor;path[4,:]=fairlead;
    path[2:3,:]=node;velocity[2:3,:]=speed;
    velocity[1,:]=zeros(3);velocity[4,:]=fairleadVelocity;
    for k in 1:3 loop
      delta[k,:]=path[k+1,:]-path[k,:];
      distance[k]=sqrt(delta[k,:]*delta[k,:]+1e-12);axis[k,:]=delta[k,:]/distance[k];
      tension[k]=if distance[k]>restLength then max(0,stiffness*(distance[k]-restLength)
        +damping*(velocity[k+1,:]-velocity[k,:])*axis[k,:]) else 0;
      traction[k,:]=tension[k]*axis[k,:];
    end for;
    for k in 1:2 loop
      der(node[k,:])=speed[k,:];
      nodeMass*der(speed[k,:])=traction[k+1,:]-traction[k,:]+4*({current[1],current[2],0}-speed[k,:])
        +{0,0,-submergedWeight+max(0,bed-node[k,3])*1200};
    end for;
    endForce=-traction[3,:];
  end AnchorLine;

  input Real surfaceFlow[2],surfaceWind[2];
  parameter Real markHeight=1.75,markArea=3*sqrt(3.0)*.85^2/4,markDraft=.10;
  final parameter Real markMass=1025*markArea*markHeight/3*(1-(1-markDraft/markHeight)^3);
  final parameter Real homeE[4]={gateEast+rightE*lineLength/2,gateEast-rightE*lineLength/2,
    gateEast+legLength*upE,gateEast+legLength*(upE/2-sqrt(3.0)*rightE/2)};
  final parameter Real homeN[4]={gateNorth+rightN*lineLength/2,gateNorth-rightN*lineLength/2,
    gateNorth+legLength*upN,gateNorth+legLength*(upN/2-sqrt(3.0)*rightN/2)};
  Real offset[4,2](each start=0,each fixed=true),motion[4,2](each start=0,each fixed=true);
  Real markHeightOffset[3],markRate[3];
  Real localTop[4,3],topVelocity[4,3],lineLoad[4,3],gateWidth,gateRight[2];
  output Real buoyPosition[3,3],mooringPath[4,4,3];
  AnchorLine rode[4](each anchorX=5*upE,each anchorY=5*upN,
    initialTop=[2.5*upE,2.5*upN,.5;0,0,-markDraft;0,0,-markDraft;0,0,-markDraft]);
  input Real racerState[11] "Enabled, heading, heel, pitch, heave, east/north speed, yaw rate, vertical speed, roll and pitch rates";
  input Real hullStations[7],hullWidths[7],hullOffset "Racer hull geometry in metres";
  Real bodyContact[4,2];
  output Real contact[12] "World force and moment about each vessel origin, racer then launch";
  output Real launchHeading(start=windBearing+3.141592653589793,fixed=true);
  Real launchYawRate(start=0,fixed=true),forwardE,forwardN;
  function hullContact "Compound hull proxies in the MultiBodyLie collision engine"
    input Real delta[2],state[11],stations[7],widths[7],offset;
    input Real launchAngle,launchHeight,launchRoll,launchPitch,launchVelocity[3],launchRates[3];
    output Real load[12];
  protected
    // Three successive hull boxes approximate the launch's narrowing forebody.
    constant Real boxX[3]={-1,1.65,2.75},halfLength[3]={2,.65,.45},halfBeam[3]={1.04,.83,.40};
    MultiBodyLie.Collision.Material rubber(k=12000,d=.65,mu=.24,eps=.0003,v_eps=.06);
    MultiBodyLie.Collision.ContactPoint point;
    Real racerAxes[3,3],launchAxes[3,3],arm[3],box[3],v[3],w[3],omega[3],f[3],m[3],fb[3],mb[3],lateral;
  algorithm
    load:=zeros(12);
    if state[1]>.5 and delta*delta<49 then
      racerAxes:=hullAxes(state[2],state[3],state[4]);
      launchAxes:=hullAxes(launchAngle,launchRoll,-launchPitch);
      v:={state[6],state[7],state[9]};
      w:=state[10]*racerAxes[1,:]+state[11]*{-sin(state[2]),cos(state[2]),0}+{0,0,state[8]};
      omega:=launchRates[1]*launchAxes[1,:]-launchRates[2]*{-sin(launchAngle),cos(launchAngle),0}+{0,0,launchRates[3]};
      for h in 1:(if offset>0 then 2 else 1) loop
        lateral:=(3-2*h)*offset;
        for k in 1:7 loop
          arm:=stations[k]*racerAxes[1,:]+lateral*racerAxes[2,:];
          for j in 1:3 loop
            box:=boxX[j]*launchAxes[1,:]+.24*launchAxes[3,:];
            point:=MultiBodyLie.Collision.sphereBox({delta[1],delta[2],state[5]},arm,widths[k],
              {0,0,launchHeight},box,launchAxes,{halfLength[j],halfBeam[j],.54});
            if point.gap<.015 then
              (f,m,fb,mb):=MultiBodyLie.Collision.pairWrenches(point,v,w,launchVelocity,omega,rubber);
              load:=load+cat(1,f,m,fb,mb);
            end if;
          end for;
        end for;
      end for;
    end if;
  end hullContact;
  function hullAxes "World directions of forward, port and up"
    input Real yaw,roll,pitch;
    output Real axes[3,3];
  algorithm
    axes[1,:]:={cos(yaw)*cos(pitch),sin(yaw)*cos(pitch),-sin(pitch)};
    axes[2,:]:={cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll),
      sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll),cos(pitch)*sin(roll)};
    axes[3,:]:=cross(axes[1,:],axes[2,:]);
  end hullAxes;
  output Real markPose[3,3] "Heave, east-axis roll and bow-up pitch for each pyramid";
  Real buoySpeed[3,3](each start=0,each fixed=true),buoyState[3,3](each start=0,each fixed=true);
  final parameter Real bx[3]={0,.34641,-.34641},by[3]={.4,-.2,-.2};
  parameter Real pyramidHeight=1.75,pyramidArea=3*sqrt(3.0)*.85^2/4,pyramidDraft=.10;
  final parameter Real pyramidMass=1025*pyramidArea*pyramidHeight/3*(1-(1-pyramidDraft/pyramidHeight)^3);
  discrete Real buoyWater[3,3](each start=0,each fixed=true),lastBuoyWater[3,3](each start=0,each fixed=true);
  Real buoyEast[3],buoyNorth[3],depthAt[3,3],buoyLift[3,3],buoyV[3,3];
  input Real landingReaction[6] "Drone contact wrench on launch, world ENU";
  output Real deckState[12] "Origin ENU, yaw/roll/bow-up pitch, linear and angular velocity ENU";
  function elevationAt "Periodic bilinear elevation on an east-north grid"
    input Real grid[:,:],east,north,period;
    output Real elevation;
  protected
    Integer rows,cols,i,j,ii,jj;
    Real x,y,a,b;
  algorithm
    rows:=size(grid,1); cols:=size(grid,2);
    x:=east*cols/period; y:=north*rows/period;
    i:=1+mod(integer(floor(x)),cols); j:=1+mod(integer(floor(y)),rows);
    ii:=1+mod(i,cols); jj:=1+mod(j,rows);
    a:=x-floor(x); b:=y-floor(y);
    elevation:=(1-b)*((1-a)*grid[j,i]+a*grid[j,ii])+b*((1-a)*grid[jj,i]+a*grid[jj,ii]);
  end elevationAt;
  output Real countdown;
  discrete output Integer stage(start=0,fixed=true) "0 prestart, 1 start open, 2/3 marks, 4 finish, 5 finished";
  discrete output Real elapsed(start=0,fixed=true);
  discrete output Real lapTime(start=0,fixed=true) "Time from the boat crossing the start until it crosses the finish";
  discrete Real crossed(start=0,fixed=true);
  Real along,across;
  discrete Real lastAlong(start=0,fixed=true),lastAcross(start=0,fixed=true),heat(start=-1,fixed=true),started(start=0,fixed=true);
  discrete Real boundary,crossingAt;
  discrete Boolean onLine,startCross,finishCross;
equation
  bodyContact[1,:]=contact[7:8]+landingReaction[1:2];bodyContact[2:4,:]=zeros(3,2);
  forwardE=cos(launchHeading);forwardN=sin(launchHeading);
  der(launchHeading)=launchYawRate;
  contact=hullContact({east-committeeEast,north-committeeNorth},racerState,hullStations,hullWidths,hullOffset,
    launchHeading,heave,roll,pitch,{motion[1,1],motion[1,2],heaveRate},{rollRate,pitchRate,launchYawRate});
  3500*der(launchYawRate)=contact[12]+landingReaction[6]+2.5*cos(pitch)*(forwardE*lineLoad[1,2]-forwardN*lineLoad[1,1])-1900*launchYawRate;
  committeeEast=homeE[1]+offset[1,1];committeeNorth=homeN[1]+offset[1,2];
  pinEast=buoyPosition[1,1];pinNorth=buoyPosition[1,2];
  markEast=buoyPosition[2:3,1];markNorth=buoyPosition[2:3,2];
  localTop[1,:]={offset[1,1]+2.5*forwardE*cos(pitch),offset[1,2]+2.5*forwardN*cos(pitch),heave+.5+2.5*sin(pitch)};
  topVelocity[1,:]={motion[1,1]-2.5*forwardN*cos(pitch)*launchYawRate-2.5*forwardE*sin(pitch)*pitchRate,motion[1,2]+2.5*forwardE*cos(pitch)*launchYawRate-2.5*forwardN*sin(pitch)*pitchRate,heaveRate+2.5*cos(pitch)*pitchRate};
  for m in 1:3 loop
    buoyPosition[m,:]={homeE[m+1]+offset[m+1,1],homeN[m+1]+offset[m+1,2],markHeightOffset[m]};
    localTop[m+1,:]={offset[m+1,1],offset[m+1,2],markHeightOffset[m]-markDraft};
    topVelocity[m+1,:]={motion[m+1,1],motion[m+1,2],markRate[m]};
    markHeightOffset[m]=buoyState[m,1];markRate[m]=buoySpeed[m,1];
  end for;
  for body in 1:4 loop
    der(offset[body,:])=motion[body,:];
    rode[body].fairlead=localTop[body,:];rode[body].fairleadVelocity=topVelocity[body,:];
    rode[body].current=surfaceFlow;lineLoad[body,:]=rode[body].endForce;
    for j in 1:4 loop
      mooringPath[body,j,:]=rode[body].path[j,:]+{homeE[body],homeN[body],0};
    end for;
    (if body==1 then launchMass else markMass)*der(motion[body,:])=lineLoad[body,1:2]+bodyContact[body,:]
      +(if body==1 then 450 else 65)*(surfaceFlow-motion[body,:])
      +(if body==1 then 2.8 else .48)*sqrt((surfaceWind-motion[body,:])*(surfaceWind-motion[body,:]))*(surfaceWind-motion[body,:]);
  end for;
  launchPose={heave,roll,pitch};
  deckState={committeeEast,committeeNorth,heave,launchHeading,roll,pitch,motion[1,1],motion[1,2],heaveRate,cos(launchHeading)*cos(pitch)*rollRate+sin(launchHeading)*pitchRate,sin(launchHeading)*cos(pitch)*rollRate-cos(launchHeading)*pitchRate,launchYawRate+sin(pitch)*rollRate};
  buoyEast={pinEast,markEast[1],markEast[2]};buoyNorth={pinNorth,markNorth[1],markNorth[2]};
  markPose=buoyState;
  for b in 1:3 loop
    der(buoyState[b,:])=buoySpeed[b,:];
    for j in 1:3 loop
      depthAt[b,j]=max(0,min(pyramidHeight,pyramidDraft+(1-fraction)*lastBuoyWater[b,j]+fraction*buoyWater[b,j]
        -buoyState[b,1]-by[j]*sin(buoyState[b,2])-bx[j]*sin(buoyState[b,3])));
      buoyV[b,j]=buoySpeed[b,1]+by[j]*cos(buoyState[b,2])*buoySpeed[b,2]+bx[j]*cos(buoyState[b,3])*buoySpeed[b,3];
      buoyLift[b,j]=if depthAt[b,j]>0 then max(0,1025*9.80665*pyramidArea*pyramidHeight/9*
        (1-(1-depthAt[b,j]/pyramidHeight)^3)+120*((buoyWater[b,j]-lastBuoyWater[b,j])/.1-buoyV[b,j])) else 0;
    end for;
    pyramidMass*der(buoySpeed[b,1])=sum(buoyLift[b,:])-pyramidMass*9.80665+lineLoad[b+1,3];
    (.16*pyramidMass)*der(buoySpeed[b,2])=cos(buoyState[b,2])*(by*buoyLift[b,:])-6*buoySpeed[b,2];
    (.16*pyramidMass)*der(buoySpeed[b,3])=cos(buoyState[b,3])*(bx*buoyLift[b,:])-6*buoySpeed[b,3];
  end for;
  der(heave)=heaveRate; der(roll)=rollRate; der(pitch)=pitchRate;
  fraction=min(1,max(0,(time-stamp)/.1));
  for k in 1:4 loop
    stationE[k]=committeeEast+forwardE*stationX[k]-forwardN*stationY[k];
    stationN[k]=committeeNorth+forwardN*stationX[k]+forwardE*stationY[k];
    immersion[k]=draft+priorWater[k]*(1-fraction)+water[k]*fraction-heave
      -stationX[k]*sin(pitch)-stationY[k]*sin(roll)*cos(pitch);
    stationRate[k]=heaveRate+stationX[k]*cos(pitch)*pitchRate+stationY[k]*
      (cos(roll)*cos(pitch)*rollRate-sin(roll)*sin(pitch)*pitchRate);
    lift[k]=if immersion[k]>0 then max(0,1025*9.80665*waterplaneArea/4*min(.78,immersion[k])
      +1400*((water[k]-priorWater[k])/.1-stationRate[k])) else 0;
  end for;
  launchMass*der(heaveRate)=landingReaction[3]+contact[9]+sum(lift)-launchMass*9.80665+lineLoad[1,3];
  rollInertia*der(rollRate)=cos(launchHeading)*landingReaction[4]+sin(launchHeading)*landingReaction[5]+cos(pitch)*(forwardE*contact[10]+forwardN*contact[11])+sin(pitch)*contact[12]+cos(roll)*cos(pitch)*(stationY*lift);
  pitchInertia*der(pitchRate)=sin(launchHeading)*landingReaction[4]-cos(launchHeading)*landingReaction[5]+forwardN*contact[10]-forwardE*contact[11]+2.5*(cos(pitch)*lineLoad[1,3]-sin(pitch)*(forwardE*lineLoad[1,1]+forwardN*lineLoad[1,2]))+cos(pitch)*(stationX*lift)-sin(roll)*sin(pitch)*(stationY*lift);
  when sample(0,.1) then
    for k in 1:4 loop
      water[k]=tide+elevationAt(backgroundTile,stationE[k],stationN[k],64)
        +elevationAt(wakeTile,stationE[k]-driftEast,stationN[k]-driftNorth,64)
        +swellHeight*sin(swellK*(stationE[k]*cos(swellBearing)+stationN[k]*sin(swellBearing))-sqrt(9.80665*swellK)*time);
    end for;
    for m in 1:3 loop
      water[m+4]=tide+elevationAt(backgroundTile,buoyPosition[m,1],buoyPosition[m,2],64)
        +elevationAt(wakeTile,buoyPosition[m,1]-driftEast,buoyPosition[m,2]-driftNorth,64)
        +swellHeight*sin(swellK*(buoyPosition[m,1]*cos(swellBearing)+buoyPosition[m,2]*sin(swellBearing))-sqrt(9.80665*swellK)*time);
    end for;
    for b in 1:3 loop
      for j in 1:3 loop
        buoyWater[b,j]=tide+elevationAt(backgroundTile,buoyEast[b]+bx[j],buoyNorth[b]+by[j],64)
          +elevationAt(wakeTile,buoyEast[b]+bx[j]-driftEast,buoyNorth[b]+by[j]-driftNorth,64)
          +swellHeight*sin(swellK*((buoyEast[b]+bx[j])*cos(swellBearing)+(buoyNorth[b]+by[j])*sin(swellBearing))-sqrt(9.80665*swellK)*time);
      end for;
    end for;
    lastBuoyWater=if initial() then buoyWater else pre(buoyWater);
    priorWater=if initial() then water else pre(water);
    stamp=time;
  end when;
  gateWidth=sqrt((committeeEast-pinEast)^2+(committeeNorth-pinNorth)^2+1e-12);
  gateRight={(committeeEast-pinEast)/gateWidth,(committeeNorth-pinNorth)/gateWidth};
  along=(north-(committeeNorth+pinNorth)/2)*gateRight[1]-(east-(committeeEast+pinEast)/2)*gateRight[2];
  across=(east-(committeeEast+pinEast)/2)*gateRight[1]+(north-(committeeNorth+pinNorth)/2)*gateRight[2];
  countdown=300-mod(utc,300);
  when sample(0,.05) then
    boundary=floor(utc/300)*300;
    crossingAt=pre(lastAcross)+(across-pre(lastAcross))*(-pre(lastAlong))/(if abs(along-pre(lastAlong))>1e-8 then along-pre(lastAlong) else 1);
    onLine=abs(crossingAt)<gateWidth/2;
    startCross=onLine and pre(lastAlong)<=0 and along>0;
    finishCross=onLine and pre(lastAlong)>=0 and along<0;
    heat=boundary;
    stage=if boundary>pre(heat) and pre(heat)>=0 and (pre(stage)<=1 or pre(stage)==5) then (if startCross then 2 else 1)
          elseif pre(stage)==1 and startCross then 2
          elseif pre(stage)==2 and (east-markEast[1])^2+(north-markNorth[1])^2<zone^2 then 3
          elseif pre(stage)==3 and (east-markEast[2])^2+(north-markNorth[2])^2<zone^2 then 4
          elseif pre(stage)==4 and finishCross then 5 else pre(stage);
    started=if boundary>pre(heat) and (pre(stage)<=1 or pre(stage)==5) then boundary else pre(started);
    elapsed=if stage>=2 and pre(stage)<>5 then utc-started elseif stage==5 then pre(elapsed) else 0;
    crossed=if stage==2 and pre(stage)<>2 then utc else pre(crossed);
    lapTime=if stage>=2 and pre(stage)<>5 then utc-crossed elseif stage==5 then pre(lapTime) else 0;
    lastAlong=along;
    lastAcross=across;
  end when;
end Regatta;
