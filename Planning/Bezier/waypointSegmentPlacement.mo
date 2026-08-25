within Planning.Bezier;

function waypointSegmentPlacement
  "Place a trajectory time on the timeline: which segment, and how far into it"
  input Real segmentDuration[:](each unit = "s");
  input Real segmentStart[size(segmentDuration, 1)](each unit = "s");
  input Integer waypointCount(min = 2);
  input Real trajectoryTime(unit = "s");
  input Real totalDuration(unit = "s");
  output Integer segment;
  output Real localTime(unit = "s");
algorithm
  segment := Planning.Bezier.activeWaypointSegment(
    segmentDuration,
    waypointCount,
    min(max(trajectoryTime, 0.0), totalDuration));
  localTime := max(0.0, min(
    trajectoryTime - segmentStart[segment],
    segmentDuration[segment]));
  annotation(Documentation(info = "<html>
    <p>The timeline lookup, kept in a function because it indexes the plan at
    a computed segment. Its caller holds the plan and the clock; this decides
    only where on the plan a given time lands, and it reads the prefix sums
    the plan already carries rather than rescanning the durations.</p>
  </html>"));
end waypointSegmentPlacement;
