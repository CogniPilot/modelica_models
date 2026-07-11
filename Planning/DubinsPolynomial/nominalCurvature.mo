within Planning.DubinsPolynomial;
function nominalCurvature "Signed curvature of a nominal Dubins segment"
  input Planning.Dubins.PathType pathType;
  input Integer segmentIndex(min=1, max=3);
  input Real turnRadius;
  output Real curvature;
protected
  Planning.Dubins.SegmentType kind;
algorithm
  assert(turnRadius > 0.0, "Dubins turn radius must be positive");
  kind := Planning.Dubins.segmentType(pathType, segmentIndex);
  curvature := if kind == Planning.Dubins.SegmentType.left then 1.0 / turnRadius
    else if kind == Planning.Dubins.SegmentType.right then -1.0 / turnRadius
    else 0.0;
end nominalCurvature;
