within Marine;
model OceanWorld "Single-clock marine world, with onboard perception and guidance"
  parameter Boolean hobie=true "Choose the hull configuration at translation";
  parameter Real prevailingBearing=atan2(4,10);
  input Real windSpeed(start=11),windBearing(start=prevailingBearing),seaState,epoch;
  input Real sailingManual(start=1),rudder,trim,vang(start=.5),deckManual,mainCar,jibCar,jibSheet;
  input Real ballastManual,ballastX,ballastY,traffic[4,9];
  input Real subManual(start=1),subPilot[4],flightManual(start=1),flightPilot[4],motorCut;
  Sailing.Sailboat boat(catamaran=hobie,mass=if hobie then 205 else 64,
    launchHeight=if hobie then .2 else .069,ballastMass=if hobie then 60 else 0,
    halfSpacing=if hobie then 1.02 else 0,deckZ=if hobie then .23 else .077,
    centerMassX=if hobie then -.28 else -.14846,
    stationX=if hobie then {-2.555,-1.9,-.9,.15,1.12,1.95,2.555} else {-1.1,-.82,-.3,.18,.63,.98,1.22},
    beamHalf=if hobie then {.09,.156,.185,.184,.155,.093,.009} else {.22,.29,.335,.32,.235,.12,.01},
    spans=if hobie then {7.15,4.56} else {2.6,2.12},feet=if hobie then {2.78,1.98} else {1.28,.60},
    mainRoach=if hobie then .82 else 0,forestaySlope=if hobie then 1.82/5.1 else 1.04/2.69,
    footX=if hobie then {.44,2.26} else {.12,1.16-.10*1.04/2.69},
    footZ=if hobie then {.97,.75} else {.19,.19},yawInertia=if hobie then 315 else 19,
    rollInertia=if hobie then 187 else 28,pitchInertia=if hobie then 292 else 12,
    foilX=if hobie then {.15,-2.42,.15,-2.42} else {.06,-.93,0,0},
    foilY=if hobie then {1.02,1.02,-1.02,-1.02} else zeros(4),
    foilArea=if hobie then {.62,.11,.62,.11} else {.115,.044,0,0},
    aspect=if hobie then {.65,3.4,.65,3.4} else {3.5,2.6,1,1},
    manualHelm=sailingManual,rudderInput=rudder,trimInput=trim,vangInput=vang,
    deckManual=deckManual,mainCarInput=mainCar,jibCarInput=jibCar,jibTrimInput=jibSheet,reachTrial=0,
    ballastManual=ballastManual,ballastCmdX=ballastX,ballastCmdY=ballastY,traffic=traffic,committeeLoad=race.contact[1:6],
    windEast=wind.localEast,windNorth=wind.localNorth,currentEast=boatWater.meanFlow[1],currentNorth=boatWater.meanFlow[2],
    orbitalEast=boatWater.velocity[1]-boatWater.meanFlow[1],orbitalNorth=boatWater.velocity[2]-boatWater.meanFlow[2],orbitalUp=boatWater.velocity[3],
    useSectionWater=1,sectionWater=surfaceSections,previousWater=priorSections,waterStamp=sectionStamp,
    waveHeight=waves.incident[1]+boatWater.surface,wavePitch=waves.incident[2],waveRoll=waves.incident[3],
    swellAmplitude=.3*seaState,wakeEast=waves.waveLoad[1],wakeNorth=waves.waveLoad[2],wakeYaw=waves.waveLoad[3]);
  Environment.MarineWind wind(meanEast=windSpeed*cos(windBearing),meanNorth=windSpeed*sin(windBearing),
    east=boat.east,north=boat.north,droneEast=drone.position[1],droneNorth=drone.position[2]);
  Environment.WakeSpectrum waves(east=boat.east,north=boat.north,heading=boat.heading,heel=boat.heel,pitch=boat.pitch,
    stations=boat.stationX,separation=boat.halfSpacing,volume=boat.sectionVolume,beamHalf=boat.beamHalf,
    currentEast=boatWater.meanFlow[1],currentNorth=boatWater.meanFlow[2],windEast=wind.meanEast,windNorth=wind.meanNorth,seaAmplitude=seaState);
  Environment.WaterColumn boatWater(pos={boat.east,boat.north,boat.heave},windSpeed=windSpeed,windDirection=windBearing);
  Sailing.Regatta race(landingReaction=drone.deckWrench[7:12],racerState={1,boat.heading,boat.heel,boat.pitch,boat.heave,boat.ve,boat.vn,boat.yawRate,boat.vertical,boat.rollRate,boat.pitchRate},
    hullStations=boat.stationX,hullWidths=boat.beamHalf,hullOffset=boat.halfSpacing,surfaceFlow=boatWater.meanFlow,surfaceWind={wind.meanEast,wind.meanNorth},wakeTile=waves.height,backgroundTile=boatWater.tile,driftEast=waves.driftE,driftNorth=waves.driftN,
    tide=boatWater.tide,swellHeight=boat.swellAmplitude,windBearing=prevailingBearing,utc=epoch+time,east=boat.east,north=boat.north);
  Wreck.Plant sub(windSpeed=windSpeed,windDirection=windBearing,sensor=sensor,elevationRanges=elev,
    manual=subManual,pilot=subPilot,hullSurfaceOffset=receiverWater[1],propSurfaceOffset=receiverWater[2:8]);
  Environment.WaterColumn droneWater(pos=drone.position,windSpeed=windSpeed,windDirection=windBearing);
  Rdd2Survey drone(deckEnabled=1,deckState=race.deckState,windENU=wind.droneWind,currentENU=droneWater.velocity,waterLevel=droneWater.surface+receiverWater[9],
    pilot=flightManual,forward=flightPilot[1],left=flightPilot[2],climb=flightPilot[3],turn=flightPilot[4],
    stationEast=estimate[1],stationNorth=estimate[2],motorCut=motorCut);
  final parameter Real beams[1017,3]=Wreck.Perception.directions();
  discrete output Real sensor[91](each start=0,each fixed=true),elev[100](each start=0,each fixed=true),
    image[864](each start=0,each fixed=true),estimate[21](each start=0,each fixed=true),
    keyframes[48,180](each start=0,each fixed=true),features[24,5](each start=0,each fixed=true);
  discrete output Real surfaceSections[2,6](each start=0,each fixed=true)
    "Tide, background sea, local wind waves and hull wake at each buoyancy strip [m]";
  discrete Real priorSections[2,6](each start=0,each fixed=true),sectionStamp(start=0,fixed=true);
  Real stripAlong[2,6],stripAcross[2];
  Real receiverPosition[9,2],receiverWater[9];
  discrete Real sampledWake[9](each start=0,each fixed=true),priorWake[9](each start=0,each fixed=true);
equation
  receiverPosition[1,:]=sub.position[1:2];
  receiverPosition[2:8,:]=sub.propPosition[:,1:2];
  receiverPosition[9,:]=drone.position[1:2];
  for i in 1:9 loop
    receiverWater[i]=(1-boat.waterFraction)*priorWake[i]+boat.waterFraction*sampledWake[i]
      +boat.swellAmplitude*sin(boat.swellPhase+boat.swellK*((receiverPosition[i,1]-boat.east)*cos(boat.swellBearing)
      +(receiverPosition[i,2]-boat.north)*sin(boat.swellBearing)));
  end for;
  for hull in 1:2 loop
    stripAcross[hull]=(3-2*hull)*boat.halfSpacing*cos(boat.heel);
    for panel in 1:6 loop
      stripAlong[hull,panel]=.5*(boat.stationX[panel]+boat.stationX[panel+1])*cos(boat.pitch)
        +(3-2*hull)*boat.halfSpacing*sin(boat.heel)*sin(boat.pitch);
    end for;
  end for;
  when sample(0,.1) then
    for i in 1:9 loop
      sampledWake[i]=Sailing.Regatta.elevationAt(waves.height,receiverPosition[i,1]-waves.driftE,
        receiverPosition[i,2]-waves.driftN,waves.period);
    end for;
    priorWake=if initial() then sampledWake else pre(sampledWake);
    for hull in 1:2 loop
      for panel in 1:6 loop
        surfaceSections[hull,panel]=waves.sectionHeight[hull,panel]+boatWater.tide+
          Wreck.Perception.seaHeight(boatWater.tile,
            boat.east+cos(boat.heading)*stripAlong[hull,panel]-sin(boat.heading)*stripAcross[hull],
            boat.north+sin(boat.heading)*stripAlong[hull,panel]+cos(boat.heading)*stripAcross[hull]);
      end for;
    end for;
    priorSections=if initial() then surfaceSections else pre(surfaceSections);
    sectionStamp=time;
  end when;
  when sample(0,.2) then
    (sensor,elev,image,estimate,keyframes,features)=Wreck.Perception.sampleFrame(time,
      cat(1,sub.position,sub.orientation,sub.velocity,{sub.yawRate,sub.pressure},sub.specificForce,sub.rates),
      beams,zeros(1017),0,pre(image),pre(estimate),pre(keyframes));
  end when;
end OceanWorld;
