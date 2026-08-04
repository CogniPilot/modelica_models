within Planning.Bezier;
function activeWaypointSegment
  "Locate the active segment in the populated prefix of a waypoint plan"
  input Real segmentDuration[:](each unit = "s");
  input Integer waypointCount(min = 2, max = size(segmentDuration, 1) + 1);
  input Real trajectoryTime(unit = "s");
  output Integer activeSegment(min = 1, max = size(segmentDuration, 1));
protected
  Real elapsed(unit = "s");
algorithm
  activeSegment := 1;
  elapsed := 0.0;
  for segmentIndex in 1:size(segmentDuration, 1) loop
    if segmentIndex < waypointCount and trajectoryTime >= elapsed then
      activeSegment := segmentIndex;
    end if;
    if segmentIndex < waypointCount then
      elapsed := elapsed + segmentDuration[segmentIndex];
    end if;
  end for;
end activeWaypointSegment;
