within MathUtilities;
function clip "Clamp a scalar to a closed interval"
  input Real value "Value to clamp";
  input Real lower "Lower interval bound";
  input Real upper "Upper interval bound";
  output Real result "Clamped value";
algorithm
  result := min(max(value, lower), upper);
  annotation(Inline = true);
end clip;
