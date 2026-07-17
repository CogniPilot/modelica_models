within MathUtilities;
function rateLimit "Limit the change from a current value to a target"
  input Real target;
  input Real current;
  input Real maxStep;
  output Real result;
algorithm
  result := current + MathUtilities.clip(target - current, -maxStep, maxStep);
  annotation(Inline = true);
end rateLimit;
