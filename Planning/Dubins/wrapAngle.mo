within Planning.Dubins;
function wrapAngle "Wrap an angle to [-pi, pi]"
  input Real angle;
  output Real wrapped;
algorithm
  wrapped := atan2(sin(angle), cos(angle));
end wrapAngle;
