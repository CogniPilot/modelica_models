within Planning.Dubins;
function segmentType "Return the type of one of a path family's three segments"
  input Planning.Dubins.PathType pathType;
  input Integer segmentIndex(min=1, max=3);
  output Planning.Dubins.SegmentType kind;
algorithm
  assert(segmentIndex >= 1 and segmentIndex <= 3,
    "Dubins segment index must be in 1:3");
  if pathType == PathType.LSL then
    kind := if segmentIndex == 2 then SegmentType.straight else SegmentType.left;
  elseif pathType == PathType.RSR then
    kind := if segmentIndex == 2 then SegmentType.straight else SegmentType.right;
  elseif pathType == PathType.LSR then
    kind := if segmentIndex == 1 then SegmentType.left else
      if segmentIndex == 2 then SegmentType.straight else SegmentType.right;
  elseif pathType == PathType.RSL then
    kind := if segmentIndex == 1 then SegmentType.right else
      if segmentIndex == 2 then SegmentType.straight else SegmentType.left;
  elseif pathType == PathType.RLR then
    kind := if segmentIndex == 2 then SegmentType.left else SegmentType.right;
  else
    kind := if segmentIndex == 2 then SegmentType.right else SegmentType.left;
  end if;
end segmentType;
