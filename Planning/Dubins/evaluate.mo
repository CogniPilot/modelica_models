within Planning.Dubins;
function evaluate "Evaluate a Dubins path at a fraction in [0, 1]"
  input Planning.Dubins.Path path;
  input Real fraction;
  output Planning.Dubins.Pose pose;
protected
  Planning.Dubins.Pose current;
  Planning.Dubins.Pose nextPose;
  Real remaining;
  Real used;
algorithm
  assert(path.feasible, "Cannot evaluate an infeasible Dubins path");
  assert(path.turnRadius > 0.0, "Dubins turn radius must be positive");
  assert(fraction >= 0.0 and fraction <= 1.0,
    "Dubins evaluation fraction must be in [0, 1]");
  current.position := path.startPosition;
  current.heading := path.startHeading;
  remaining := fraction * path.length / path.turnRadius;
  for segmentIndex in 1:3 loop
    used := min(max(remaining, 0.0),
      path.normalizedSegmentLength[segmentIndex]);
    nextPose := Planning.Dubins.advance(
      current.position,
      current.heading,
      Planning.Dubins.segmentType(path.pathType, segmentIndex),
      used,
      path.turnRadius);
    current := nextPose;
    remaining := remaining - used;
  end for;
  pose := current;
end evaluate;
