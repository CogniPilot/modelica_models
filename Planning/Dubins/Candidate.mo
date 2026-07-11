within Planning.Dubins;
record Candidate "One Dubins family evaluated for a boundary-value problem"
  Planning.Dubins.PathType pathType;
  Real normalizedSegmentLength[3]
    "Normalized segment lengths: turn angles or straight distance/radius";
  Real length "Physical path length";
  Boolean feasible;
end Candidate;
