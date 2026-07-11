within LieGroups.Analysis;
function rotationAngle "Principal SO(3) angle represented by a rotation matrix"
  input Real R[3, 3];
  output Real angle "Angle in [0, pi]";
protected
  Real cosine;
algorithm
  cosine := max(-1.0, min(1.0, 0.5 * (R[1, 1] + R[2, 2] + R[3, 3] - 1.0)));
  angle := acos(cosine);
end rotationAngle;
