within Planning.Dubins;
record Path "Selected Dubins path"
  Real startPosition[2];
  Real startHeading;
  Real goalPosition[2];
  Real goalHeading;
  Real turnRadius;
  Planning.Dubins.PathType pathType;
  Real normalizedSegmentLength[3]
    "Normalized segment lengths: turn angles or straight distance/radius";
  Real length "Physical path length";
  Boolean feasible;
end Path;
