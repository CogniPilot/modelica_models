within Planning.Examples;
model DubinsAircraftFlightPlan
  "General figure eight whose two traversal headings share one center position"
  parameter Integer waypointCount(min=2) = 7;
  parameter Real waypointPosition[waypointCount, 2] = [
    0.0, 0.0;
    12.0, 7.0;
    12.0, -7.0;
    0.0, 0.0;
    -12.0, 7.0;
    -12.0, -7.0;
    0.0, 0.0]
    "Two outer poses per lobe and three center occurrences";
  parameter Real waypointHeading[waypointCount] = {
    0.7853981633974483,
    0.0,
    3.141592653589793,
    2.356194490192345,
    3.141592653589793,
    0.0,
    0.7853981633974483};
  parameter Real planningTurnRadius(min=0.0) = 2.5;
  parameter Real aircraftMinimumTurnRadius(min=0.0) = 1.0;
  parameter Real flightSpeed(min=0.0) = 1.5;
  parameter Real maximumBankAngle(min=0.0) = 0.5;
  parameter Real maximumRollRate(min=0.0) = 3.0;

  Planning.Examples.DubinsAircraftTrajectory leg[waypointCount - 1](
    startPosition=waypointPosition[1:waypointCount - 1, :],
    startHeading=waypointHeading[1:waypointCount - 1],
    goalPosition=waypointPosition[2:waypointCount, :],
    goalHeading=waypointHeading[2:waypointCount],
    each turnRadius=planningTurnRadius,
    each aircraftMinimumTurnRadius=aircraftMinimumTurnRadius,
    each flightSpeed=flightSpeed,
    each maximumBankAngle=maximumBankAngle,
    each maximumRollRate=maximumRollRate);
  annotation(experiment(StartTime=0.0, StopTime=1.0, Interval=0.0025));
end DubinsAircraftFlightPlan;
