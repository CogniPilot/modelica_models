within Planning.Dubins;
function planType "Construct a path for a requested Dubins family"
  input Real startPosition[2];
  input Real startHeading;
  input Real goalPosition[2];
  input Real goalHeading;
  input Real turnRadius;
  input Planning.Dubins.PathType pathType;
  output Planning.Dubins.Path path;
protected
  Planning.Dubins.Candidate selected;
algorithm
  selected := Planning.Dubins.candidate(
    startPosition, startHeading, goalPosition, goalHeading,
    turnRadius, pathType);
  path.startPosition := startPosition;
  path.startHeading := startHeading;
  path.goalPosition := goalPosition;
  path.goalHeading := goalHeading;
  path.turnRadius := turnRadius;
  path.pathType := selected.pathType;
  path.normalizedSegmentLength := selected.normalizedSegmentLength;
  path.length := selected.length;
  path.feasible := selected.feasible;
end planType;
