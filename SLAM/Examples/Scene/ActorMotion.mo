within SLAM.Examples.Scene;
// Six authored actors, ordered as three pedestrians then three traffic cars.
// The rendering scene maps ENU (east,north) to (east,0,-north).
model ActorMotion
  parameter Integer actors = 6;
  parameter Real pi = 3.141592653589793;
  parameter Real speed[actors] = {0.78,0.78,0.78,2.2,2.2,2.2};
  parameter Real offset[actors] = {24.0,36.0+pi*4.05+7.0,10.0,8.0,40.0+pi*1.4+6.0,30.0};
  parameter Real halfLength[actors] = {18.0,18.0,18.0,20.0,20.0,20.0};
  parameter Real radius[actors] = {4.05,4.05,4.05,1.4,1.4,1.4};
  parameter Real phase[actors] = {0.0,0.43,0.86,0.0,0.0,0.0};
  parameter Real animationRate = 0.85;
  input Real sampleTime = 0.0;
  input Real useSceneRoutes = 0.0;
  input Real pedestrianRouteRadius = 4.05;
  input Real carRouteRadius = 1.4;
  input Real pedestrianRouteHalfLength = 18.0;
  // Main Street traffic passes straight through town; recycling happens
  // beyond the downtown buildings instead of turning inside the street.
  input Real streetCarRoutes = 0.0;
  input Real carStreetHalfLength = 55.0;
  input Real pedestrianCrossingRoutes = 0.0;
  // The city sidewalk circuit crosses only at the two authored crossings.
  input Real pedestrianWestCrossing = -29.8;
  input Real pedestrianEastCrossing = 32.0;
  output Real east[actors];
  output Real north[actors];
  output Real sceneYaw[actors];
  output Real distance[actors];
  output Real walkTime[actors];
protected
  Real effectiveRadius[actors];
  Real effectiveOffset[actors];
  Real routeEast[actors];
  Real routeNorth[actors];
  Real routeYaw[actors];
  Real streetEast[actors];
  Real crossingDistance[actors];
  Real crossingLength;
  Real crossingEast[actors];
  Real crossingNorth[actors];
  Real crossingYaw[actors];
  Real straight[actors];
  Real arc[actors];
  Real circumference[actors];
  Real rawDistance[actors];
  Real signedRemainder[actors];
  Real wrappedDistance[actors];
  Real routeDistance[actors];
  Real angle[actors];
  Real eastTangent[actors];
  Real northTangent[actors];
equation
  crossingLength = pedestrianEastCrossing-pedestrianWestCrossing;
  for i in 1:actors loop
    effectiveRadius[i] = radius[i]+useSceneRoutes*
      ((if i <= 3 then pedestrianRouteRadius else carRouteRadius)-radius[i]);
    effectiveOffset[i] = offset[i]+(if i == 2 or i == 5 then
      pi*(effectiveRadius[i]-radius[i]) else 0.0);
    straight[i] = 2.0*(halfLength[i]+(if i <= 3 then
      useSceneRoutes*(pedestrianRouteHalfLength-halfLength[i]) else 0.0));
    arc[i] = pi*effectiveRadius[i];
    circumference[i] = 2.0*(straight[i]+arc[i]);

    distance[i] = speed[i]*sampleTime;
    rawDistance[i] = distance[i]+effectiveOffset[i];
    signedRemainder[i] = rawDistance[i]-(if noEvent(rawDistance[i] >= 0.0) then
      floor(rawDistance[i]/circumference[i]) else -floor(-rawDistance[i]/circumference[i]))*circumference[i];
    wrappedDistance[i] = signedRemainder[i]+circumference[i];
    routeDistance[i] = wrappedDistance[i]-floor(wrappedDistance[i]/circumference[i])*circumference[i];

    angle[i] = if noEvent(routeDistance[i] < straight[i]+arc[i]) then
      -pi/2.0+(routeDistance[i]-straight[i])/effectiveRadius[i] else
      pi/2.0+(routeDistance[i]-2.0*straight[i]-arc[i])/effectiveRadius[i];
    routeEast[i] = if noEvent(routeDistance[i] < straight[i]) then
      -straight[i]/2.0+routeDistance[i] else
      if noEvent(routeDistance[i] < straight[i]+arc[i]) then
        straight[i]/2.0+effectiveRadius[i]*cos(angle[i]) else
        if noEvent(routeDistance[i] < 2.0*straight[i]+arc[i]) then
          straight[i]/2.0-(routeDistance[i]-straight[i]-arc[i]) else
          -straight[i]/2.0+effectiveRadius[i]*cos(angle[i]);
    routeNorth[i] = if noEvent(routeDistance[i] < straight[i]) then -effectiveRadius[i] else
      if noEvent(routeDistance[i] < straight[i]+arc[i]) then effectiveRadius[i]*sin(angle[i]) else
        if noEvent(routeDistance[i] < 2.0*straight[i]+arc[i]) then effectiveRadius[i] else
          effectiveRadius[i]*sin(angle[i]);
    eastTangent[i] = if noEvent(routeDistance[i] < straight[i]) then 1.0 else
      if noEvent(routeDistance[i] < straight[i]+arc[i]) then -sin(angle[i]) else
        if noEvent(routeDistance[i] < 2.0*straight[i]+arc[i]) then -1.0 else -sin(angle[i]);
    northTangent[i] = if noEvent(routeDistance[i] < straight[i] or
      (routeDistance[i] >= straight[i]+arc[i] and routeDistance[i] < 2.0*straight[i]+arc[i])) then
      0.0 else cos(angle[i]);
    routeYaw[i] = atan2(eastTangent[i],-northTangent[i]);

    streetEast[i] = (if i == 5 then -1.0 else 1.0)*
      (rawDistance[i]-floor(rawDistance[i]/(2.0*carStreetHalfLength))*2.0*carStreetHalfLength-carStreetHalfLength);

    crossingDistance[i] = rawDistance[i]
      -floor(rawDistance[i]/(2.0*crossingLength+4.0*effectiveRadius[i]))
        *(2.0*crossingLength+4.0*effectiveRadius[i]);
    crossingEast[i] = if noEvent(crossingDistance[i] < crossingLength) then
      pedestrianWestCrossing+crossingDistance[i] else
      if noEvent(crossingDistance[i] < crossingLength+2.0*effectiveRadius[i]) then
        pedestrianEastCrossing else
        if noEvent(crossingDistance[i] < 2.0*crossingLength+2.0*effectiveRadius[i]) then
          pedestrianEastCrossing-(crossingDistance[i]-crossingLength-2.0*effectiveRadius[i]) else
          pedestrianWestCrossing;
    crossingNorth[i] = if noEvent(crossingDistance[i] < crossingLength) then
      -effectiveRadius[i] else
      if noEvent(crossingDistance[i] < crossingLength+2.0*effectiveRadius[i]) then
        -effectiveRadius[i]+crossingDistance[i]-crossingLength else
        if noEvent(crossingDistance[i] < 2.0*crossingLength+2.0*effectiveRadius[i]) then
          effectiveRadius[i] else
          effectiveRadius[i]-(crossingDistance[i]-2.0*crossingLength-2.0*effectiveRadius[i]);
    crossingYaw[i] = if noEvent(crossingDistance[i] < crossingLength) then pi/2.0 else
      if noEvent(crossingDistance[i] < crossingLength+2.0*effectiveRadius[i]) then pi else
        if noEvent(crossingDistance[i] < 2.0*crossingLength+2.0*effectiveRadius[i]) then -pi/2.0 else 0.0;

    east[i] = routeEast[i]+(if i > 3 then
      streetCarRoutes*(streetEast[i]-routeEast[i]) else
      pedestrianCrossingRoutes*(crossingEast[i]-routeEast[i]));
    north[i] = routeNorth[i]+(if i > 3 then
      streetCarRoutes*((if i == 5 then effectiveRadius[i] else -effectiveRadius[i])-routeNorth[i]) else
      pedestrianCrossingRoutes*(crossingNorth[i]-routeNorth[i]));
    sceneYaw[i] = routeYaw[i]+(if i > 3 then
      streetCarRoutes*((if i == 5 then -pi/2.0 else pi/2.0)-routeYaw[i]) else
      pedestrianCrossingRoutes*(crossingYaw[i]-routeYaw[i]));
    walkTime[i] = sampleTime*animationRate+phase[i];
  end for;
end ActorMotion;
