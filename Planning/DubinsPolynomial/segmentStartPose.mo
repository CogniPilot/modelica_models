within Planning.DubinsPolynomial;
function segmentStartPose "Nominal pose at the start of a Dubins segment"
  input Real startPosition[2];
  input Real startHeading;
  input Real turnRadius;
  input Planning.Dubins.PathType pathType;
  input Real normalizedSegmentLength[3];
  input Integer segmentIndex(min=1, max=3);
  output Planning.Dubins.Pose pose;
protected
  Planning.Dubins.Pose nextPose;
algorithm
  pose.position := startPosition;
  pose.heading := startHeading;
  for precedingSegment in 1:segmentIndex - 1 loop
    nextPose := Planning.Dubins.advance(
      pose.position,
      pose.heading,
      Planning.Dubins.segmentType(pathType, precedingSegment),
      normalizedSegmentLength[precedingSegment],
      turnRadius);
    pose := nextPose;
  end for;
end segmentStartPose;
