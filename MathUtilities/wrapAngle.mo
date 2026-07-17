within MathUtilities;
function wrapAngle "Wrap an angle to the principal interval"
  input Real angle(unit = "rad");
  output Real result(unit = "rad");
algorithm
  result := atan2(sin(angle), cos(angle));
  annotation(Inline = true);
end wrapAngle;
