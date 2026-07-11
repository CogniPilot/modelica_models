within Planning.Dubins;
function advance "Advance a pose along one normalized Dubins segment"
  input Real position[2];
  input Real heading;
  input Planning.Dubins.SegmentType kind;
  input Real normalizedLength(min=0.0);
  input Real turnRadius;
  output Planning.Dubins.Pose result;
algorithm
  assert(turnRadius > 0.0, "Dubins turn radius must be positive");
  if kind == SegmentType.left then
    result.position[1] := position[1] + turnRadius
      * (sin(heading + normalizedLength) - sin(heading));
    result.position[2] := position[2] + turnRadius
      * (-cos(heading + normalizedLength) + cos(heading));
    result.heading := heading + normalizedLength;
  elseif kind == SegmentType.right then
    result.position[1] := position[1] + turnRadius
      * (sin(heading) - sin(heading - normalizedLength));
    result.position[2] := position[2] + turnRadius
      * (cos(heading - normalizedLength) - cos(heading));
    result.heading := heading - normalizedLength;
  else
    result.position := position + turnRadius * normalizedLength
      * {cos(heading), sin(heading)};
    result.heading := heading;
  end if;
end advance;
