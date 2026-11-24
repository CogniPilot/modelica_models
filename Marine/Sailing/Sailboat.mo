within Marine.Sailing;
model Sailboat "Autonomous fin-keel sloop on a three-leg closed course"
  function wetTriangle "Fraction below a planar water surface, linear depth across a triangle"
    input Real depths[3] "Positive below water [m]";
    output Real fraction;
  protected
    Real low,mid,high;
  algorithm
    low:=min(depths); high:=max(depths); mid:=sum(depths)-low-high;
    if high<=0 then fraction:=0;
    elseif low>=0 then fraction:=1;
    elseif mid<=0 then fraction:=high/(high-low)*high/(high-mid);
    else fraction:=1-low/(low-high)*low/(low-mid);
    end if;
  end wetTriangle;

  function waterStrip "Water pressure normal to a flexible sail panel in heading-aligned axes"
    input Real corners[4,3], normal[3], waterVelocity[3], panelVelocity[3];
    input Real height,slopeX,slopeY,area;
    output Real wet;
    output Real force[3];
  protected
    Real depth[4],speed;
  algorithm
    for j in 1:4 loop
      depth[j]:=height+slopeX*corners[j,1]+slopeY*corners[j,2]-corners[j,3];
    end for;
    wet:=(wetTriangle(depth[{1,2,3}])+wetTriangle(depth[{1,3,4}]))/2;
    speed:=(waterVelocity-panelVelocity)*normal;
    force:=0.5*1025*1.2*wet*area*speed*abs(speed)*normal;
  end waterStrip;
  function contactWrench "Horizontal compliant hull contact at sampled waterline sections"
    input Real pose[7], traffic[4,9];
    input Boolean twin;
    output Real wrench[3];
  protected
    Real a,b,x,y,u,v,dx,dy,d,r,rate,f,armX,armY,otherX,otherY;
  algorithm
    wrench:=zeros(3);
    for p in 1:4 loop
      if traffic[p,1]>.5 and abs(pose[7]-traffic[p,8])<.65 then
        for i in 1:(if twin then 2 else 1) loop
          for j in 1:(if traffic[p,9]>.5 then 2 else 1) loop
            for k in -2:2 loop
              for l in -2:2 loop
                a:=k*(if twin then .95 else .42); b:=if twin then (3-2*i)*1.02 else 0;
                armX:=a*cos(pose[3])-b*sin(pose[3]);armY:=a*sin(pose[3])+b*cos(pose[3]);
                a:=l*(if traffic[p,9]>.5 then .95 else .42);b:=if traffic[p,9]>.5 then (3-2*j)*1.02 else 0;
                otherX:=a*cos(traffic[p,4])-b*sin(traffic[p,4]);otherY:=a*sin(traffic[p,4])+b*cos(traffic[p,4]);
                dx:=pose[1]+armX-traffic[p,2]-otherX;dy:=pose[2]+armY-traffic[p,3]-otherY;
                d:=sqrt(dx*dx+dy*dy);r:=(if twin then .49 else .31)+(if traffic[p,9]>.5 then .49 else .31);
                if d<r and d>.0001 then
                  x:=dx/d;y:=dy/d;
                  u:=pose[4]-pose[6]*armY-traffic[p,5]+traffic[p,7]*otherY;
                  v:=pose[5]+pose[6]*armX-traffic[p,6]-traffic[p,7]*otherX;
                  rate:=u*x+v*y;f:=max(0,850*(r-d)-55*rate);
                  wrench:=wrench+f*{x,y,armX*y-armY*x};
                end if;
              end for;
            end for;
          end for;
        end for;
      end if;
    end for;
  end contactWrench;
  function standingRig "One loaded midpoint per elastic cable, mast step fixed"
    input Real anchors[3,3],tang[3],previous[2],rate[2],sailMomentBody[3],jibLoad[3],clothReaction[2,3],gravityBody[3];
    input Real dt,EA,preload,entrainedMass,previousSag[3,3];
    output Real lean[2],velocity[2],path[3,3,3],tension[3],lateralK[3],midSag[3,3];
  protected
    Real direction[3],head[3],force[3],q[3],weight[3],drive[2],dragK,rest,L,L0,P,lo,hi,sag,halfSpan,Tline;
  algorithm
    // Boom/mast share the overturning moment. The headsail load below is
    // already carried by the forestay and is omitted from this applied moment.
    drive:={sailMomentBody[2],-sailMomentBody[1]}/(tang[3]-.41);
    head:=tang+{previous[1],previous[2],0};
    for i in 1:3 loop
      direction:=head-anchors[i,:];L:=sqrt(direction*direction);direction:=direction/L;
      q:=tang-anchors[i,:];L0:=sqrt(q*q);
      P:=if i<=2 then preload else preload*(2*.96/1.82)*L0/sqrt(.96^2+1.02^2+5.51^2);
      rest:=L0/(1+P/EA);
      weight:=.045*L0*.5*gravityBody;
      force:=weight+(if i<=2 then clothReaction[min(i,2),:] else jibLoad);
      // Implicit transverse damping of the moving wire and attached wet cloth.
      dragK:=(8+entrainedMass/dt)/dt;
      force:=force+dragK*previousSag[i,:];
      force:=force-direction*(force*direction);
      P:=sqrt(force*force);
      // Midpoint transverse force balance. Either half may carry tension
      // only. Cable length and geometric stiffness grow as it deflects.
      lo:=0;hi:=.5*sqrt(max(0,rest^2-L^2))+.02+L*(P/EA)^(1/3);
      for iteration in 1:16 loop
        sag:=(lo+hi)/2;halfSpan:=sqrt(.25*L^2+sag^2);
        Tline:=EA*max(0,2*halfSpan/rest-1);
        if (2*Tline+dragK*halfSpan)*sag<P*halfSpan then lo:=sag;else hi:=sag;end if;
      end for;
      sag:=(lo+hi)/2;halfSpan:=sqrt(.25*L^2+sag^2);
      path[i,1,:]:=anchors[i,:];path[i,3,:]:=head;
      midSag[i,:]:=sag*force/max(1e-10,P);
      path[i,2,:]:=(head+anchors[i,:])/2+midSag[i,:];
      tension[i]:=EA*max(0,2*halfSpan/rest-1);
      q:=(path[i,2,:]-head)*tension[i]/halfSpan;
      drive:=drive+q[1:2];
      lateralK[i]:=2*tension[i]/halfSpan+2*EA/rest*sag^2/halfSpan^2;
    end for;
    // First fore-aft and athwartship hounds sway mode. Backward damping
    // accounts for rig losses without a high-frequency cable time step.
    velocity:=((14+entrainedMass)*rate+dt*(drive-850*previous))/(14+entrainedMass+dt*350);
    lean:=previous+dt*velocity;
  end standingRig;
  input Real traffic[4,9] "Active, east, north, yaw, east/north velocity, yaw rate, height, twin hull";
  output Real peerForce[3] "World east/north force and yaw moment";
  input Real committeeLoad[6] "World force and moment of the committee launch about the sailboat origin";
  input Real wakeEast,wakeNorth,wakeYaw "Free-surface pressure reaction from the local wave FMU";
  input Real useSectionWater,sectionWater[2,6],previousWater[2,6],waterStamp;
  Real waterFraction;
  input Real windEast, windNorth, currentEast, currentNorth, waveHeight, waveRoll, wavePitch;
  input Real orbitalEast,orbitalNorth,orbitalUp "Wave particle velocity at the boat [m/s]";
  input Real manualHelm,rudderInput,trimInput;
  input Real vangInput "Tackle adjustment, 0 eased to 1 tight";
  Real boomElevation(start=.04,fixed=true),boomLiftRate(start=0,fixed=true);
  Real vangLength,vangTension,vangArm,clewTorque;
  parameter Real vangAttachment=.42,vangDrop=.115;
  input Real downhaulInput "Catamaran luff adjustment, 0 eased to 1 hauled";
  parameter Real downhaulTravel(unit="m")=.10 "Available shortening at the tack";
  parameter Real luffRigidity(unit="N")=45000, downhaulStiffness(unit="N/m")=40000;
  Real downhaulShortening(start=0,fixed=true),tackDrop,downhaulTension;
  Real payout(start=.12,fixed=true),drumAngle,lineSpeed,boomLimit;
  parameter Real drumRadius=.043, sheetArm=.44;
  parameter Boolean catamaran=false;
  parameter Real launchSpeed=.45,launchHeight=.2,launchPitch=0,launchVertical=0,launchPitchRate=0;
  input Real swellAmplitude,reachTrial,deckManual,mainCarInput,jibCarInput,jibTrimInput;
  Real mainCar(start=0,fixed=true),jibCar(start=0,fixed=true),jibSheet(start=.35,fixed=true);
  parameter Integer sailOf[3]={1,2,2};
  parameter Real attachRadius[2]={if catamaran then 1.84 else .44,feet[2]};
  Real fairlead[3,3],attachment[2,3],tipLocal[2,3],tipVel[2,3],leadVel[3,3];
  Real sheetForce[3,3],lineDirection[3,3],ropeTorque[3,3],lineSpan[3],lineRate[3],lineTension[3],lineTarget[3];
  parameter Real travelerRiser(unit="m")=.05 "Traveler pin height above the rail datum";
  final parameter Real mainInitialSpan=sqrt((attachRadius[1]*(1-cos(.04)*cos(.17)))^2+
    (attachRadius[1]*cos(.04)*sin(.17))^2+(.50-travelerRiser+attachRadius[1]*sin(.04))^2);
  Real linePaid[3](start={if catamaran then mainInitialSpan else .32,.32,.32},each fixed=true),trimRate[2](each start=0,each fixed=true),hingeLoad[2],sheetYaw[3],sheetLift;
  parameter Real mastEI(unit="N.m2")=27000 "Estimated fore-aft spar bending rigidity";
  parameter Real mastModalMass(unit="kg")=6, mastDampingRatio=.22;
  final parameter Real stayFraction=if catamaran then (5.85-footZ[1])/spans[1] else .68;
  final parameter Real mastStiffness=3*mastEI/(spans[1]^3*(1-stayFraction)^2);
  Real mastTip(start=0,fixed=true),mastTipRate(start=0,fixed=true);
  parameter Real mainsheetEA(unit="N")=30000 "Assumed working axial rigidity of one sheet part";
  parameter Real sheetTail(unit="m")=2 "Loaded line between tackle and winch";
  final parameter Integer purchase=6;
  parameter Real sheetHaulForce(unit="N")=220 "Assumed available force at the working tail";
  parameter Real sheetHoldingForce(unit="N")=500 "Assumed slip load at the working tail";
  Real haulFraction,brakeSlip;
  Real mainSheetStiffness(unit="N/m") "Effective stiffness across both blocks";
  parameter Real blockStack(unit="m")=.12 "Combined closed height between tackle attachments";
  output Real mainBlockClearance "Free distance between the mainsheet block cheeks [m]";
  // Reference sail cut with the two blocks together and the traveler centered.
  final parameter Real footCut=if catamaran then asin((blockStack-.50+travelerRiser)/attachRadius[1]) else 0;
  parameter Real leechEA(unit="N")=90000 "Estimated axial rigidity of the reinforced mainsail leech";
  final parameter Real leechRest=sqrt((spans[1]-feet[1]*sin(footCut))^2+(feet[1]*cos(footCut)-(if catamaran then .14 else 0))^2)+.012;
  Real leechDelta[3],leechDirection[3],leechLength,leechLoad,leechLift;
  Real blockContact,mainTension,jibTension;
  Real jibTrimTarget;
  output Real exposedSheetSag[3] "Free-span deflection with excess line retained on deck";
  parameter Real stayEA(unit="N")=520000,shroudPreload(unit="N")=220;
  final parameter Real stayAnchors[3,3]=[-.52,1.02,.34;-.52,-1.02,.34;2.26,0,.75];
  final parameter Real tang[3]={.44,0,5.85};
  final parameter Real jibStation[6]={(row-1)/5*spans[2]/(tang[3]-stayAnchors[3,3]) for row in 1:6};
  discrete output Real stayPath[3,3,3],stayTension[3],stayStiffness[3];
  discrete Real midSag[3,3](each start=0,each fixed=true);
  discrete Real tangLean[2](each start=0,each fixed=true),tangRate[2](each start=0,each fixed=true);
  Real contactBody[2,3],mainMomentBody[3],jibWireLoad[3];
  output Real sailTail[2,4,3] "Paired face telltales on main and jib";
  output Real shroudTail[2,3] "Direction of each sidestay streamer in hull coordinates, port first";
  function streamerDirection
    input Real relativeVelocity[3],gravityDown[3],density;
    output Real direction[3];
  protected
    Real resultant[3];
  algorithm
    // Drag and weight per unit projected area for a light polyester ribbon.
    resultant:=.5*density*sqrt(relativeVelocity*relativeVelocity)*relativeVelocity+.05*9.80665*(1-density/1380)*gravityDown;
    direction:=resultant/max(1e-8,sqrt(resultant*resultant));
  end streamerDirection;
  Real tailArm[2,3],tailFluid[2,3];
  Real tailImmersion[2] "Wet fraction across the ribbon near the waterline";
  Real mainCarTarget,jibCarTarget,windSide;
  parameter Real swellK=.34,swellBearing=1.04159;
  Real swellPhase,swellLocal[2],waterDamping,waterVertical;
  final parameter Real waterplaneArea=sum((stationX[2:7]-stationX[1:6]).*(beamHalf[2:7]+beamHalf[1:6]))*(if catamaran then 2 else 1);
  parameter Real heaveDampingRatio=.38 "Estimated fraction of critical damping at design waterline";
  parameter Real podRadius=if catamaran then .17 else .075, podMass=if catamaran then 1.3 else .35;
  parameter Real podZ=if catamaran then 8.44 else 3.02;
  Real podCenter[3],podSurface[3],podNormal[3],podCap,podWetVolume,podBuoyancy,podRoll,podPitch,totalMass;
  output Real sectionVolume[2,6];
  Real volumes[2,6], wettedSkin[2],skinArea,hullRe,frictionCoefficient,skinDrag;
  parameter Real waterViscosity=1.05e-6 "Kinematic viscosity [m2/s]";
  parameter Real formFactor=.12 "Assumed viscous form factor, requires tow calibration";
  parameter Real foilTop[4]=if catamaran then {.10,.15,.10,.15} else {-.11,-.035,0,0};
  parameter Real foilBottom[4]=if catamaran then {-.34,-.57,-.34,-.57} else {-.75,-.405,-.1,-.1};
  Real foilEnd[4,2,3],foilDepth[4,2],foilArm[4,3],foilVelocity[4,3],foilBody[4,3];
  Real bodyLoad[4,3],foilLoad[4,3],foilTorque[4,3],foilCL[4],foilOrbital[4,3],foilPhase[4];
  Real impact[2,3],deckWet[2],impactTorque[2];
  output Real deckImmersion,contactLoad;
  parameter Real spans[2]={2.6,2.12},feet[2]={1.28,.60},footZ[2]={.19,.19},mainRoach=0;
  parameter Real yawInertia=19,rollInertia=28,pitchInertia=12, halfSpacing=0;
  parameter Real ballastMass=0;
  input Real ballastManual,ballastCmdX,ballastCmdY;
  Real ballastX(start=0,fixed=true),ballastY(start=0,fixed=true),targetX,targetY;
  Real hullLift[2],hullPitch[2],hullRoll[2],massRoll,massPitch;
  parameter Real mass=64, noGo=0.83;
  parameter Real launchEast=-8,launchNorth=-7,launchHeading=-2.1;
  Real east(start=launchEast,fixed=true), north(start=launchNorth,fixed=true), heading(start=launchHeading,fixed=true);
  output Real shoreWrench[6];
  Real surge(start=launchSpeed,fixed=true), sway(start=0,fixed=true), yawRate(start=0,fixed=true);
  // Effective inertia of the water entrained by the submerged part of each sail.
  // Chord and span mean-square arms use a triangular planform approximation.
  Real rigWaterMass[2],hingeWaterInertia[2],rigWaterInertia,levelYawMoment;
  Real heel(start=0,fixed=true), pitch(start=launchPitch,fixed=true), heave(start=launchHeight,fixed=true), vertical(start=launchVertical,fixed=true);
  Real sheet,rudder(start=0,fixed=true);
  discrete Integer leg(start=1,fixed=true), tack(start=1,fixed=true);
  Real goalE,goalN,dx,dy,desired,upwind,relative,crossTrack,course,error,sheetTarget;
  parameter Real forestaySlope=(1.16-.12)/(2.78-.09) "Aft rake per metre of rise";
  parameter Real footX[2]={.12,1.16-(.19-.09)*forestaySlope};
  parameter Real axisX[2]={0,-forestaySlope/sqrt(1+forestaySlope^2)},axisZ[2]={1,1/sqrt(1+forestaySlope^2)};
  output Real sheetRotation[2,3,3] "Sail frame to hull frame";
  SailMembrane cloth[2](cutAngle={footCut,0},footInTrack={catamaran,false},each mastSupport=stayFraction,hasShrouds={true,false},battenCount={if catamaran then 8 else 0,if catamaran then 4 else 0},span=spans,foot=feet,rake={0,spans[2]*forestaySlope},roach={mainRoach,0});
  Real rotation[3,3],rig[2,3,3],trim[2](each start=.17,each fixed=true), waterRelative[3],spin[3],origin[2,3];
  Real pitchRate(start=launchPitchRate,fixed=true),rigMoment[2,3],rigForce[2,3],surfaceNormal[3];
  output Real sailVertex[2,24,3],wettedArea[2],leechSpeed[2],sailMoment[3];
  Real ve,vn,awx,awy,beta,fx,fy;
  Real foilAlpha[4],foilLift[4],foilDrag[4],foilSide[4],foilAxial[4],inflow[4],forward[4],angle[4],wetFoil[4];
  parameter Real foilX[4]={0.06,-0.93,0,0},foilY[4]={0,0,0,0},foilArea[4]={0.115,0.044,0,0},aspect[4]={3.5,2.6,1,1};
  output Real sailForward[2], sailPort[2], buoyancy;
  parameter Real fixInterval=0.2, gpsVariance=0.16;
  Real vane(start=0,fixed=true), vaneRate(start=0,fixed=true), cupSpeed(start=8,fixed=true);
  Real gyro,magnetic,accelX,accelY,measuredWindE,measuredWindN;
  discrete Real fixE(start=-8,fixed=true),fixN(start=-7,fixed=true);
  discrete Real navE(start=-8,fixed=true),navN(start=-7,fixed=true),speedE(start=0,fixed=true),speedN(start=0,fixed=true);
  discrete Real courseEstimate(start=-2.1,fixed=true),headingVar(start=0.04,fixed=true);
  discrete Real p11(start=.25,fixed=true),p12(start=0,fixed=true),p22(start=.25,fixed=true);
  discrete Real a11,a12,a22,kp,kv,predE,predN,headingPred,kh;
  Real rollRate(start=0,fixed=true);
  // Six longitudinal panels of the rendered hull loft, split port/starboard.
  parameter Real stationX[7]={-1.1,-.82,-.3,.18,.63,.98,1.22};
  parameter Real beamHalf[7]={.22,.29,.335,.32,.235,.12,.01};
  parameter Real beamFraction[4]={0,.46,.86,1},deckZ=.077;
  parameter Real centerMassX=-.14846 "Longitudinal mass centre for level design trim [m]";
  Real pitchBuoyancy,weightArm;
  function hullHydrostatics
    input Real xs[7],widths[7],fractions[4],deck,rot[3,3],surface[3],waterLevel,yOffset,depthScale;
    input Real sections[6],useSections;
    input Real amp,kDir[2],phase,translation[3],spinRate[3];
    output Real lift,pitchTorque,rollTorque,dynamicForce[3],dynamicPitch,maxDeckWet,volume[6],skin;
  protected
    Real profile[3],stationDeck,stationLevel,center[3],x,y,w,scale,bottom,top,area,wetDepth,verticalProjection,point[3],lever[3];
    Real sectionZ[4],localLift,wave,crestPhase,draft,deckWet,wetFraction,normalV,waterV[3],velocity[3],panelForce[3],deckPoint[3];
    Real deckPressure,entryPressure,edgeA[3],edgeB[3],depthA,depthB,wetLength;
  algorithm
    skin:=0;lift:=0; pitchTorque:=0;rollTorque:=0;dynamicForce:=zeros(3);dynamicPitch:=0;maxDeckWet:=0;volume:=zeros(6);
    verticalProjection:=surface*rot[:,3];
    for station in 1:6 loop
      x:=(xs[station]+xs[station+1])/2;
      w:=(widths[station]+widths[station+1])/2;
      scale:=(.48+1.55*w)*depthScale;
      profile:=Shore.pontoonSection(x);
      stationDeck:=if depthScale>1 then profile[2] else deck;
      if depthScale>1 then w:=profile[1];end if;
      sectionZ:=if depthScale>1 then stationDeck*ones(4)+(profile[3]-stationDeck)*{1,.90,.56,0} else {-.19*scale,-.16*scale,-.05*scale,deck};
      center:=rot*{x,yOffset,0};
      stationLevel:=waterLevel+(if useSections>.5 then sections[station]-surface[1:2]*center[1:2] else 0);
      for strip in 1:3 loop
        bottom:=(sectionZ[strip]+sectionZ[strip+1])/2;
        top:=stationDeck;
        area:=(xs[station+1]-xs[station])*w*(fractions[strip+1]-fractions[strip]);
        for side in 1:2 loop
          y:=(if side==1 then 1 else -1)*w*(fractions[strip+1]+fractions[strip])/2;
          edgeA:=rot*{x,yOffset+(if side==1 then 1 else -1)*w*fractions[strip],sectionZ[strip]};
          edgeB:=rot*{x,yOffset+(if side==1 then 1 else -1)*w*fractions[strip+1],sectionZ[strip+1]};
          depthA:=stationLevel+surface*edgeA+amp*sin(phase+kDir*edgeA[1:2]);
          depthB:=stationLevel+surface*edgeB+amp*sin(phase+kDir*edgeB[1:2]);
          wetLength:=if min(depthA,depthB)>=0 then 1 elseif max(depthA,depthB)<=0 then 0 else max(depthA,depthB)/abs(depthA-depthB);
          skin:=skin+wetLength*(xs[station+1]-xs[station])*sqrt((edgeA-edgeB)*(edgeA-edgeB));
          point:=rot*{x,y+yOffset,if verticalProjection<=0 then bottom else top};
          crestPhase:=phase+kDir*point[1:2];
          wave:=amp*sin(crestPhase);
          wetDepth:=min(top-bottom,max(0,(stationLevel+surface*point+wave)/max(1e-9,abs(verticalProjection))));
          lever:=point+rot[:,3]*(if verticalProjection<=0 then .5*wetDepth else -.5*wetDepth);
          volume[station]:=volume[station]+area*wetDepth;
          localLift:=1025*9.80665*area*wetDepth;
          if depthScale>1 then
            deckPoint:=rot*{x,y+yOffset,stationDeck};
            deckWet:=stationLevel+surface*deckPoint+amp*sin(phase+kDir*deckPoint[1:2]);
            if x>1 then maxDeckWet:=max(maxDeckWet,deckWet);end if;
            wetFraction:=min(1,wetDepth/.065);
            // One-sided pressure on wetted underside and overtopped deck.
            // Relative velocity includes angular motion at this section and deep-water orbital flow.
            waterV:=amp*sqrt(9.80665*.34)*{kDir[1]/.34*sin(crestPhase),kDir[2]/.34*sin(crestPhase),-cos(crestPhase)};
            velocity:=translation+cross(spinRate,lever)-waterV;
            normalV:=velocity*rot[:,3];
            deckPressure:=-.5*1025*1.1*max(0,normalV)^2*min(1,max(0,deckWet)/.09);
            entryPressure:=.5*1025*.65*max(0,-normalV)^2*wetFraction;
            panelForce:=(entryPressure+deckPressure)*area*rot[:,3];
            // Form drag when the bow shoulders and foredeck bury.
            if x>1 then
              panelForce[1]:=panelForce[1]-.5*1025*.28*area*min(1,max(0,deckWet)/.15)*velocity[1]*abs(velocity[1]);
            end if;
            dynamicForce:=dynamicForce+panelForce;
            dynamicPitch:=dynamicPitch+(lever[3]-.52)*panelForce[1]-lever[1]*panelForce[3];
          end if;
          lift:=lift+localLift;rollTorque:=rollTorque+lever[2]*localLift; pitchTorque:=pitchTorque-lever[1]*localLift;
        end for;
      end for;
    end for;
  end hullHydrostatics;

equation
  // Trampoline rail limits in hull coordinates, centred at the carriage mid-travel.
  waterFraction=min(1,max(0,(time-waterStamp)/.1));
  totalMass=mass+podMass;
  podCenter=rotation*{footX[1]+mastTip,0,podZ};
  podSurface=surfaceNormal+cat(1,swellAmplitude*cos(swellPhase+swellLocal*podCenter[1:2])*swellLocal,{0});
  podNormal=podSurface/sqrt(podSurface*podSurface);
  podCap=max(0,min(2*podRadius,podRadius+(waveHeight-heave+surfaceNormal*podCenter+
    swellAmplitude*sin(swellPhase+swellLocal*podCenter[1:2]))/sqrt(podSurface*podSurface)));
  podWetVolume=3.14159265358979*podCap^2*(podRadius-podCap/3);
  podBuoyancy=1025*9.80665*podWetVolume;
  podRoll=(podCenter[2]+podNormal[2]*.75*(2*podRadius-podCap)^2/(3*podRadius-podCap))*podBuoyancy-podCenter[2]*podMass*9.80665;
  podPitch=-(podCenter[1]+podNormal[1]*.75*(2*podRadius-podCap)^2/(3*podRadius-podCap))*podBuoyancy+podCenter[1]*podMass*9.80665;
  swellPhase=swellK*(east*cos(swellBearing)+north*sin(swellBearing))-sqrt(9.80665*swellK)*time;
  swellLocal=swellK*{cos(swellBearing-heading),sin(swellBearing-heading)};
  deckImmersion=max(deckWet);contactLoad=sum(hullLift);
  targetX=if not catamaran then 0 else max(-.72,min(.72,if ballastManual>.5 then ballastCmdX else -3.8*pitch-1.4*pitchRate));
  targetY=if not catamaran then 0 else max(-.84,min(.84,if ballastManual>.5 then ballastCmdY else 3.8*heel+1.2*rollRate));
  der(ballastX)=max(-.35,min(.35,(targetX-ballastX)/.65));
  der(ballastY)=max(-.48,min(.48,(targetY-ballastY)/.55));
  massRoll=-9.80665*ballastMass*(rotation[2,1]*ballastX+rotation[2,2]*ballastY+rotation[2,3]*.52);
  massPitch=9.80665*ballastMass*(rotation[1,1]*ballastX+rotation[1,2]*ballastY+rotation[1,3]*.52);
  goalE=if leg==1 then -32 else if leg==2 then -15 else -8;
  goalN=if leg==1 then -18 else if leg==2 then 9 else -7;
  dx=goalE-navE; dy=goalN-navN;
  // Simulated instruments. Only this measurement layer reads true vehicle motion.
  gyro=yawRate+0.002+0.0025*sin(23.1*time);
  magnetic=heading+.015*sin(5.7*time);
  accelX=(fx+sum(foilAxial)+skinDrag)/mass+.025*sin(18*time);
  accelY=(fy+sum(foilSide)-80*sway*abs(sway)-40*sway)/mass+.028*cos(17*time);
  der(vane)=vaneRate;
  der(vaneRate)=64*sin(beta-vane)-12*vaneRate;
  der(cupSpeed)=(sqrt(awx^2+awy^2)-cupSpeed)/0.32;
  measuredWindE=speedE-cupSpeed*cos(courseEstimate+vane);
  measuredWindN=speedN-cupSpeed*sin(courseEstimate+vane);
  when sample(0,fixInterval) then
    fixE=east+.32*sin(time*4.9)+.18*sin(time*8.21);
    fixN=north+.34*cos(time*4.3)+.15*sin(time*9.41);
    headingPred=pre(courseEstimate)+fixInterval*gyro;
    kh=(pre(headingVar)+.00016)/(pre(headingVar)+.00016+.0008);
    courseEstimate=headingPred+kh*atan2(sin(magnetic-headingPred),cos(magnetic-headingPred));
    headingVar=(1-kh)*(pre(headingVar)+.00016);
    a11=pre(p11)+2*fixInterval*pre(p12)+fixInterval^2*pre(p22)+0.0001;
    a12=pre(p12)+fixInterval*pre(p22);
    a22=pre(p22)+0.012;
    kp=a11/(a11+gpsVariance); kv=a12/(a11+gpsVariance);
    predE=pre(navE)+fixInterval*pre(speedE)+0.5*fixInterval^2*(accelX*cos(courseEstimate)-accelY*sin(courseEstimate));
    predN=pre(navN)+fixInterval*pre(speedN)+0.5*fixInterval^2*(accelX*sin(courseEstimate)+accelY*cos(courseEstimate));
    navE=predE+kp*(fixE-predE); navN=predN+kp*(fixN-predN);
    speedE=pre(speedE)+fixInterval*(accelX*cos(courseEstimate)-accelY*sin(courseEstimate))+kv*(fixE-predE);
    speedN=pre(speedN)+fixInterval*(accelX*sin(courseEstimate)+accelY*cos(courseEstimate))+kv*(fixN-predN);
    p11=(1-kp)*a11; p12=(1-kp)*a12; p22=a22-kv*a12;
  end when;
  desired=atan2(dy,dx);
  upwind=atan2(-measuredWindN,-measuredWindE);
  relative=atan2(sin(desired-upwind),cos(desired-upwind));
  crossTrack=-sin(upwind)*(navE-goalE)+cos(upwind)*(navN-goalN);
  when dx*dx+dy*dy<6.25 then
    leg=if pre(leg)==3 then 1 else pre(leg)+1;
  end when;
  when crossTrack>4 then
    tack=-1;
  elsewhen crossTrack < -4 then
    tack=1;
  end when;
  course=if reachTrial>.5 then -2.1 elseif abs(relative)<noGo then upwind+tack*noGo else desired;
  error=atan2(sin(course-courseEstimate),cos(course-courseEstimate));
  ve=cos(heading)*surge-sin(heading)*sway+currentEast;
  vn=sin(heading)*surge+cos(heading)*sway+currentNorth;
  der(east)=ve; der(north)=vn;
  awx=(windEast-ve)*cos(heading)+(windNorth-vn)*sin(heading);
  awy=-(windEast-ve)*sin(heading)+(windNorth-vn)*cos(heading);
  beta=atan2(-awy,-awx);
  sheetTarget=(if sin(vane)>=0 then 1 else -1)*min(1.40,max(0,abs(atan2(sin(vane),cos(vane)))-0.32));
  boomLimit=2*asin(min(0.99,max(0,payout/(2*sheetArm))));
  lineSpeed=if manualHelm>.5 then .24*trimInput else max(-.40,min(.40,(2*sheetArm*sin(abs(sheetTarget)/2)-payout)/.22));
  der(payout)=if (payout<=.012 and lineSpeed<0) or (payout>=.59 and lineSpeed>0) then 0 else lineSpeed;
  drumAngle=payout/drumRadius;
  windSide=if sin(vane)>=0 then -1 else 1;
  mainCarTarget=if deckManual>.5 then mainCarInput else windSide*.78*max(0,min(1,(abs(vane)-.8)/1.5));
  jibCarTarget=if deckManual>.5 then jibCarInput else .30+.45*min(1,abs(vane)/2.4);
  der(mainCar)=if catamaran then max(-.40,min(.40,(min(.9,max(-.9,mainCarTarget))-mainCar)/.38)) else 0;
  der(jibCar)=if catamaran then max(-.40,min(.40,(min(.88,max(.18,jibCarTarget))-jibCar)/.38)) else 0;
  jibTrimTarget=if manualHelm>.5 then min(1,max(0,(payout-.012)/.578+
    (if deckManual>.5 then jibTrimInput-.35 else 0))) elseif deckManual>.5 then jibTrimInput else min(1,abs(sheetTarget)/1.5);
  der(jibSheet)=if catamaran then max(-.7,min(.7,(jibTrimTarget-jibSheet)/.5)) else 0;
  sheet=trim[1];
  fairlead[1,:]={-attachRadius[1],if catamaran then mainCar else 0,if catamaran then travelerRiser-.50 else -.105};
  fairlead[2,:]={if catamaran then -1.71 else -feet[2],if catamaran then jibCar else 0,if catamaran then -.32 else -.105};
  fairlead[3,:]={fairlead[2,1],-fairlead[2,2],fairlead[2,3]};
  leadVel[1,:]={0,der(mainCar),0};
  leadVel[2,:]={0,der(jibCar),0};
  leadVel[3,:]=-leadVel[2,:];
  lineTarget[1]=if catamaran then blockStack-.010+attachRadius[1]*max(0,payout-.012)/sheetArm else sqrt(fairlead[1,3]^2+(attachRadius[1]*payout/sheetArm)^2);
  for j in 2:3 loop
    lineTarget[j]=if catamaran then (if (j==2 and windSide<0) or (j==3 and windSide>0) then lineSpan[j]+.035
      else .47+2.1*jibSheet) else sqrt(fairlead[2,3]^2+(.88*feet[2]*payout/sheetArm)^2);
  end for;
  mainSheetStiffness=purchase^2*mainsheetEA/(sheetTail+purchase*linePaid[1]);
  // Limit negative drum work under load. A backdriven brake pays out
  // excess extension while the elastic line continues to report its load.
  haulFraction=max(0,1-mainSheetStiffness*max(0,lineSpan[1]-linePaid[1])/(purchase*sheetHaulForce));
  brakeSlip=max(0,lineSpan[1]-linePaid[1]-purchase*sheetHoldingForce/mainSheetStiffness)/.035;
  for j in 1:3 loop
    der(linePaid[j])=if catamaran and j==1 then max(brakeSlip,min(1.8,max(0,(lineTarget[j]-linePaid[j])/.22)))+
      haulFraction*max(-1.8,min(0,(lineTarget[j]-linePaid[j])/.22)) else max(-1.8,min(1.8,(lineTarget[j]-linePaid[j])/.22));
    lineSpan[j]=sqrt((fairlead[j,:]-attachment[sailOf[j],:])*(fairlead[j,:]-attachment[sailOf[j],:]));
    // The sheets lead through deck blocks to the winch. A slack bight
    // rests on deck once its free span reaches the trampoline or sloop deck.
    exposedSheetSag[j]=min(max(0,footZ[sailOf[j]]+min(fairlead[j,3],attachment[sailOf[j],3])
      -(if catamaran then .414 else .083)),.5*sqrt(max(0,linePaid[j]^2-lineSpan[j]^2)));
    lineDirection[j,:]=(fairlead[j,:]-attachment[sailOf[j],:])/max(.001,lineSpan[j]);
    lineRate[j]=lineDirection[j,:]*(leadVel[j,:]-tipVel[sailOf[j],:]);
    lineTension[j]=if (j==3 and not catamaran) or lineSpan[j]<=linePaid[j] then 0 else
      max(0,(if catamaran and j==1 then mainSheetStiffness elseif catamaran then 1750 else 720)*(lineSpan[j]-linePaid[j])+
        (if catamaran and j==1 then 240 elseif catamaran then 40 else 12)*(lineRate[j]-der(linePaid[j])));
    sheetForce[j,:]=(lineTension[j]-(if j==1 then blockContact else 0))*lineDirection[j,:];
    ropeTorque[j,:]=cross(attachment[sailOf[j],:],sheetForce[j,:]);
    sheetYaw[j]=ropeTorque[j,:]*{axisX[sailOf[j]],0,axisZ[sailOf[j]]};
  end for;
  mainBlockClearance=max(0,lineSpan[1]-blockStack);
  blockContact=if catamaran and lineSpan[1]<blockStack then max(0,65000*(blockStack-lineSpan[1])-180*lineRate[1]) else 0;
  // Head-to-clew reinforcement transfers boom downhaul into an aft spar load.
  // Its stretch is measured between the moving masthead and the rotating boom.
  leechDelta={mastTip,0,spans[1]+tackDrop}-sheetRotation[1,:,:]*
    {-feet[1]*cos(boomElevation),0,feet[1]*sin(boomElevation)};
  leechLength=sqrt(leechDelta*leechDelta);
  leechDirection=leechDelta/leechLength;
  leechLoad=if catamaran then leechEA*max(0,leechLength/leechRest-1) else 0;
  leechLift=leechLoad*leechDirection*(sheetRotation[1,:,:]*
    {feet[1]*sin(boomElevation),0,feet[1]*cos(boomElevation)});
  der(mastTip)=mastTipRate;
  (mastModalMass+.25*rigWaterMass[1])*der(mastTipRate)=(if catamaran then sheetRotation[1,1,:]*cloth[1].mastLoad else 0)
    -leechLoad*leechDirection[1]-mastStiffness*mastTip-2*mastDampingRatio*sqrt(mastModalMass*mastStiffness)*mastTipRate;
  mainMomentBody=sheetRotation[1,:,:]*cloth[1].fluidMoment;
  jibWireLoad=.5*sheetRotation[2,:,:]*cloth[2].fluidForce;
  for wire in 1:2 loop
    contactBody[wire,:]=sheetRotation[1,:,:]*{0,cloth[1].wireReaction[wire],0};
  end for;
  when sample(0,.01) then
    (tangLean,tangRate,stayPath,stayTension,stayStiffness,midSag)=standingRig(stayAnchors,tang,
      pre(tangLean),pre(tangRate),if catamaran then pre(mainMomentBody) else zeros(3),
      if catamaran then pre(jibWireLoad) else zeros(3),pre(contactBody),transpose(rotation)*{0,0,-9.80665},.01,stayEA,shroudPreload,.5*sum(pre(rigWaterMass)),pre(midSag));
  end when;
  for row in 1:6 loop
    cloth[1].luffShift[row,:]=if catamaran then transpose(sheetRotation[1,:,:])*
      {stayPath[1,3,1]-tang[1],stayPath[1,3,2],0}*((row-1)/5/stayFraction)-{0,0,tackDrop*(6-row)/5} else zeros(3);
    cloth[2].luffShift[row,:]=if catamaran then transpose(sheetRotation[2,:,:])*(
      (if jibStation[row]<=.5 then (1-2*jibStation[row])*stayPath[3,1,:]+2*jibStation[row]*stayPath[3,2,:]
       else (2-2*jibStation[row])*stayPath[3,2,:]+(2*jibStation[row]-1)*stayPath[3,3,:])
        -((1-jibStation[row])*stayAnchors[3,:]+jibStation[row]*tang)) else zeros(3);
  end for;
  cloth[1].mastOffset=transpose(sheetRotation[1,:,:])*{mastTip,0,0};
  cloth[2].mastOffset=zeros(3);
  mainTension=lineTension[1];jibTension=sum(lineTension[2:3]);
  sheetLift=cross(attachment[1,:]+{0,0,tackDrop},sheetForce[1,:])*sheetRotation[1,:,2];
  vangLength=sqrt(vangAttachment^2+vangDrop^2+2*vangAttachment*vangDrop*sin(boomElevation));
  vangArm=vangAttachment*vangDrop*cos(boomElevation)/vangLength;
  vangTension=if catamaran then 0 else max(0,3200*(vangLength-(.49-.065*max(0,min(1,vangInput)))));
  // The tackle and reinforced luff are elastic elements in series. The head
  // is fixed while the tack slide travels down the mast under the downhaul.
  der(downhaulShortening)=if catamaran then (downhaulTravel*max(0,min(1,downhaulInput))-downhaulShortening)/.25 else 0;
  tackDrop=downhaulShortening/(1+luffRigidity/(spans[1]*downhaulStiffness));
  downhaulTension=luffRigidity*tackDrop/spans[1];
  clewTorque=feet[1]*(sin(boomElevation)*cloth[1].clewForce[1]+cos(boomElevation)*cloth[1].clewForce[3]);
  der(boomElevation)=boomLiftRate;
  der(boomLiftRate)=(clewTorque+leechLift-vangTension*vangArm+sheetLift-(if catamaran then 26 else .95)*cos(boomElevation)-(if catamaran then 90 else 3)*boomLiftRate
    +900*max(0,(if catamaran then -.25 else -.035)-boomElevation)-900*max(0,boomElevation-.65))/((if catamaran then 11 else .28)+hingeWaterInertia[1]);
  cloth[1].leechTension=leechLoad;
  cloth[2].leechTension=0;
  cloth[1].boomElevation=boomElevation;
  cloth[2].boomElevation=0;
  der(rudder)=((if manualHelm>.5 then .54*rudderInput else max(-0.56,min(0.56,1.35*error-0.45*gyro)))-rudder)/0.2;

  // Body to level, heading-aligned frame (forward, port, up). Wave slopes
  // and orbital velocity are supplied by the same sea as the hull buoyancy.
  rotation=[cos(pitch), sin(pitch)*sin(heel), sin(pitch)*cos(heel);
            0, cos(heel), -sin(heel);
            -sin(pitch), cos(pitch)*sin(heel), cos(pitch)*cos(heel)];
  spin={rollRate*cos(pitch),pitchRate,yawRate-rollRate*sin(pitch)};
  waterRelative={orbitalEast*cos(heading)+orbitalNorth*sin(heading)-surge,
       -orbitalEast*sin(heading)+orbitalNorth*cos(heading)-sway,orbitalUp-vertical};
  surfaceNormal={-tan(wavePitch),tan(waveRoll),-1};
  for side in 1:2 loop
    tailArm[side,:]=rotation*(.44*stayPath[side,1,:]+.56*stayPath[side,2,:]);
    tailImmersion[side]=noEvent(max(0,min(1,.5+(waveHeight-heave+surfaceNormal*tailArm[side,:]+
      swellAmplitude*sin(swellPhase+swellLocal*tailArm[side,1:2]))/.03)));
    tailFluid[side,:]=transpose(rotation)*((1-tailImmersion[side])*{awx,awy,-vertical}+tailImmersion[side]*waterRelative-cross(spin,tailArm[side,:]));
    shroudTail[side,:]=if catamaran then streamerDirection(tailFluid[side,:],-rotation[3,:],
      1.225+(1025-1.225)*tailImmersion[side]) else zeros(3);
  end for;
  for m in 1:2 loop
    tipLocal[m,:]=if m==1 then {-attachRadius[m]*cos(boomElevation),0,attachRadius[m]*sin(boomElevation)}
      else {-attachRadius[m],0,0};
    attachment[m,:]=sheetRotation[m,:,:]*tipLocal[m,:]-(if m==1 then {0,0,tackDrop} else zeros(3));
    tipVel[m,:]=cross(trimRate[m]*{axisX[m],0,axisZ[m]},attachment[m,:])+
      (if m==1 then sheetRotation[m,:,:]*cross({0,boomLiftRate,0},tipLocal[m,:])-{0,0,der(tackDrop)} else zeros(3));
    rigWaterMass[m]=1025*.04*wettedArea[m];
    hingeWaterInertia[m]=rigWaterMass[m]*feet[m]^2/6;
    hingeLoad[m]=cross(cloth[m].vertex[4,:],cloth[m].clewForce)*{axisX[m],0,axisZ[m]};
    der(trim[m])=trimRate[m];
    der(trimRate[m])=(hingeLoad[m]+(if m==1 then sheetYaw[1] else sheetYaw[2]+sheetYaw[3])
      -(if catamaran then 22 else 1.1)*trimRate[m]
      -(if catamaran then 1500 else 130)*(max(0,trim[m]-1.68)+min(0,trim[m]+1.68)))/
      ((if catamaran then (if m==1 then 12 else 4) else (if m==1 then .42 else .22))+hingeWaterInertia[m]);
    // Rodrigues rotation about the luff through the tack.
    sheetRotation[m,:,:]=[
      cos(trim[m])+axisX[m]^2*(1-cos(trim[m])), -axisZ[m]*sin(trim[m]), axisX[m]*axisZ[m]*(1-cos(trim[m]));
      axisZ[m]*sin(trim[m]), cos(trim[m]), -axisX[m]*sin(trim[m]);
      axisX[m]*axisZ[m]*(1-cos(trim[m])), axisX[m]*sin(trim[m]), cos(trim[m])+axisZ[m]^2*(1-cos(trim[m]))];
    rig[m,:,:]=rotation*sheetRotation[m,:,:];
    origin[m,:]=rotation*{footX[m],0,footZ[m]};
    for side in 1:2 loop
      cloth[m].shroudFoot[side,:]=transpose(sheetRotation[m,:,:])*{
        (if catamaran then -.52 else -.05)-footX[m],
        (3-2*side)*(if catamaran then 1.02 else .31),
        (if catamaran then .34 else .09)-footZ[m]};
    end for;
    cloth[m].wireK=if catamaran then stayStiffness[1:2] else {1e8,1e8};
    for side in 1:2 loop
      cloth[m].shroudMid[side,:]=if catamaran then transpose(sheetRotation[m,:,:])*
        (stayPath[side,2,:]-{footX[m],0,footZ[m]}) else (cloth[m].shroudHead+cloth[m].shroudFoot[side,:])/2;
    end for;
    cloth[m].shroudHead=if catamaran then transpose(sheetRotation[m,:,:])*(stayPath[1,3,:]-{footX[m],0,footZ[m]}) else transpose(sheetRotation[m,:,:])*{
      (if catamaran then .44 else .12)-footX[m],0,
      (if catamaran then 5.85 else 2.78)-footZ[m]};
    cloth[m].air=transpose(rig[m,:,:])*({awx,awy,-vertical}-cross(spin,origin[m,:]));
    cloth[m].water=transpose(rig[m,:,:])*(waterRelative-cross(spin,origin[m,:]));
    cloth[m].omega=transpose(rig[m,:,:])*spin+der(trim[m])*{axisX[m],0,axisZ[m]};
    cloth[m].gravity=transpose(rig[m,:,:])*{0,0,-9.80665};
    cloth[m].waterPlane=cat(1,transpose(rig[m,:,:])*surfaceNormal,{waveHeight-heave+surfaceNormal*origin[m,:]+swellAmplitude*sin(swellPhase+swellLocal*origin[m,1:2])});
    sailVertex[m,:,:]=cloth[m].vertex;
    wettedArea[m]=cloth[m].wetArea;
    leechSpeed[m]=cloth[m].leechSpeed;
    sailTail[m,:,:]=cloth[m].faceTail;
    rigForce[m,:]=rig[m,:,:]*cloth[m].fluidForce;
    rigMoment[m,:]=rig[m,:,:]*cloth[m].fluidMoment+cross(origin[m,:],rigForce[m,:]);
    sailForward[m]=rigForce[m,1];sailPort[m]=rigForce[m,2];
  end for;
  rigWaterInertia=sum(rigWaterMass[m]*(footZ[m]^2+2*footZ[m]*spans[m]/3+spans[m]^2/6) for m in 1:2);
  sailMoment=rigMoment[1,:]+rigMoment[2,:];
  fx=sum(sailForward)+sum(impact[:,1]);
  fy=sum(sailPort)+sum(impact[:,2]);
  for k in 1:4 loop
    for tip in 1:2 loop
      foilEnd[k,tip,:]=rotation*{foilX[k],foilY[k],if tip==1 then foilTop[k] else foilBottom[k]};
      foilDepth[k,tip]=waveHeight-heave+surfaceNormal*foilEnd[k,tip,:]+swellAmplitude*sin(swellPhase+swellLocal*foilEnd[k,tip,1:2]);
    end for;
    wetFoil[k]=if min(foilDepth[k,:])>=0 then 1 elseif max(foilDepth[k,:])<=0 then 0 else max(foilDepth[k,:])/max(1e-9,abs(foilDepth[k,1]-foilDepth[k,2]));
    foilArm[k,:]=if foilDepth[k,1]>foilDepth[k,2] then foilEnd[k,1,:]+.5*wetFoil[k]*(foilEnd[k,2,:]-foilEnd[k,1,:]) else foilEnd[k,2,:]+.5*wetFoil[k]*(foilEnd[k,1,:]-foilEnd[k,2,:]);
    foilPhase[k]=swellPhase+swellLocal*foilArm[k,1:2];
    foilOrbital[k,:]=swellAmplitude*sqrt(9.80665*swellK)*exp(-swellK*max(0,.5*sum(foilDepth[k,:])))*{swellLocal[1]/swellK*sin(foilPhase[k]),swellLocal[2]/swellK*sin(foilPhase[k]),-cos(foilPhase[k])};
    foilVelocity[k,:]=-waterRelative+cross(spin,foilArm[k,:])-foilOrbital[k,:];
    foilBody[k,:]=transpose(rotation)*foilVelocity[k,:];
    forward[k]=foilBody[k,1];inflow[k]=foilBody[k,2];
    angle[k]=if forward[k]^2+inflow[k]^2>1e-12 then atan2(inflow[k],forward[k]) else 0;
    foilAlpha[k]=angle[k]+(if k==2 or k==4 then rudder else 0);
    foilCL[k]=(2*3.14159/(1+2/(.8*aspect[k])))*.5*sin(2*foilAlpha[k]);
    foilLift[k]=.5*1025*foilArea[k]*wetFoil[k]*(forward[k]^2+inflow[k]^2)*foilCL[k];
    foilDrag[k]=.5*1025*foilArea[k]*wetFoil[k]*(forward[k]^2+inflow[k]^2)*
      ((if catamaran then .007 else .018)+foilCL[k]^2/(3.14159*.8*aspect[k])+1.3*sin(foilAlpha[k])^4);
    bodyLoad[k,:]={foilLift[k]*sin(angle[k])-foilDrag[k]*cos(angle[k]),-foilLift[k]*cos(angle[k])-foilDrag[k]*sin(angle[k]),0};
    foilLoad[k,:]=rotation*bodyLoad[k,:];
    foilSide[k]=foilLoad[k,2];foilAxial[k]=foilLoad[k,1];
    foilTorque[k,:]=cross(foilArm[k,:],foilLoad[k,:]);
  end for;
  skinArea=wettedSkin[1]+(if catamaran then wettedSkin[2] else 0);
  hullRe=abs(waterRelative[1])*(stationX[7]-stationX[1])/waterViscosity;
  frictionCoefficient=if hullRe<5e5 then 1.328/sqrt(max(1,hullRe)) else .075/(log10(hullRe)-2)^2;
  skinDrag=.5*1025*skinArea*(1+formFactor)*frictionCoefficient*waterRelative[1]*abs(waterRelative[1]);
  shoreWrench=Shore.contact({east,north,heave},heading,rotation,{surge,sway,vertical},spin,stationX,beamHalf,halfSpacing);
  peerForce=contactWrench({east,north,heading,ve,vn,yawRate,heave},traffic,catamaran);
  der(surge)=((wakeEast+committeeLoad[1]+peerForce[1])*cos(heading)+(wakeNorth+committeeLoad[2]+peerForce[2])*sin(heading)+fx+shoreWrench[1]+sum(foilAxial)+skinDrag)/totalMass+yawRate*sway;
  der(sway)=(-(wakeEast+committeeLoad[1]+peerForce[1])*sin(heading)+(wakeNorth+committeeLoad[2]+peerForce[2])*cos(heading)+fy+shoreWrench[2]+sum(foilSide)-80*sway*abs(sway)-40*sway)/totalMass-yawRate*surge;
  levelYawMoment=shoreWrench[6]+wakeYaw+committeeLoad[6]+peerForce[3]+sailMoment[3]+sum(foilTorque[:,3]);
  der(yawRate)=(levelYawMoment-(if catamaran then 190 else 30)*yawRate)/(yawInertia+rigWaterInertia*(sin(pitch)^2+cos(pitch)^2*sin(heel)^2)+sum(hingeWaterInertia));
  der(heading)=yawRate;
  der(heel)=rollRate;
  // The roll axis pitches with the hull. Its moment is conjugate to the
  // roll-rate term in spin, including when the bows pass vertical.
  der(rollRate)=(cos(pitch)*(committeeLoad[4]*cos(heading)+committeeLoad[5]*sin(heading)+shoreWrench[4]+sum(foilTorque[:,1])+podRoll+sailMoment[1]+(if catamaran then sum(hullRoll)+massRoll else -400*sin(heel-waveRoll)))-sin(pitch)*levelYawMoment-(if catamaran then 280 else 65)*rollRate)/(rollInertia+rigWaterInertia+podMass*podZ^2);
  der(pitch)=pitchRate;
  weightArm=rotation[1,1]*centerMassX;
  der(pitchRate)=(-committeeLoad[4]*sin(heading)+committeeLoad[5]*cos(heading)+shoreWrench[5]+sum(foilTorque[:,2])+podPitch+sailMoment[2]+sum(impactTorque)+pitchBuoyancy+mass*9.80665*weightArm+massPitch-(if catamaran then 420 else 72)*pitchRate)/(pitchInertia+rigWaterInertia+podMass*(podZ^2+footX[1]^2));
  der(heave)=vertical;
  for hull in 1:2 loop
    sectionVolume[hull,:]=if hull==1 or catamaran then volumes[hull,:] else zeros(6);
    (hullLift[hull],hullPitch[hull],hullRoll[hull],impact[hull,:],impactTorque[hull],deckWet[hull],volumes[hull,:],wettedSkin[hull])=hullHydrostatics(stationX,beamHalf,beamFraction,deckZ,rotation,surfaceNormal,(if useSectionWater>.5 then 0 else waveHeight)-heave,(if hull==1 then 1 else -1)*halfSpacing,if catamaran then 2.65 else 1,previousWater[hull,:]*(1-waterFraction)+sectionWater[hull,:]*waterFraction,useSectionWater,swellAmplitude,swellLocal,swellPhase,{surge,sway,vertical},spin);
  end for;
  buoyancy=hullLift[1]+(if catamaran then hullLift[2] else 0);
  pitchBuoyancy=hullPitch[1]+(if catamaran then hullPitch[2] else 0);
  waterDamping=if catamaran then 2*heaveDampingRatio*sqrt(1025*9.80665*waterplaneArea/totalMass)*min(1,buoyancy/(mass*9.80665)) else 5.8;
  waterVertical=orbitalUp-swellAmplitude*sqrt(9.80665*swellK)*cos(swellPhase)
     +(if useSectionWater>.5 then sum(sectionWater-previousWater)/1.2 else 0);
  der(vertical)=(committeeLoad[3]+shoreWrench[3]+podBuoyancy+buoyancy+sum(foilLoad[:,3])+sum(impact[:,3])+sum(rigForce[:,3]))/totalMass-9.80665-waterDamping*(vertical-waterVertical);
  annotation(experiment(StopTime=300,Interval=0.05));
end Sailboat;
