within Planning.Examples;
model DubinsFamilySample "Sample one requested Dubins family over normalized time"
  parameter Planning.Dubins.PathType pathType = Planning.Dubins.PathType.LSL;
  parameter Real startPosition[2] = {0.0, 0.0};
  parameter Real startHeading = 0.0;
  parameter Real goalPosition[2] = {2.0, 1.0};
  parameter Real goalHeading = 1.2;
  parameter Real turnRadius(min=0.0) = 1.0;
  parameter Planning.Dubins.Path path = Planning.Dubins.planType(
    startPosition, startHeading, goalPosition, goalHeading,
    turnRadius, pathType);
  output Real x;
  output Real y;
  output Real heading;
protected
  Planning.Dubins.Pose pose;
equation
  assert(path.feasible, "Requested Dubins family is infeasible");
  pose = Planning.Dubins.evaluate(path, time);
  x = pose.position[1];
  y = pose.position[2];
  heading = pose.heading;
  when terminal() then
    assert(abs(x - goalPosition[1]) < 2.0e-8 and
        abs(y - goalPosition[2]) < 2.0e-8 and
        abs(Planning.Dubins.wrapAngle(heading - goalHeading)) < 2.0e-8,
      "Dubins gallery path failed its terminal boundary condition");
  end when;
  annotation(experiment(StartTime=0.0, StopTime=1.0, Interval=0.0025));
end DubinsFamilySample;
