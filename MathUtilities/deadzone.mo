within MathUtilities;
function deadzone
  "Remove a centered dead band and rescale the remaining travel to full range"
  input Real value "Normalized input, nominally in [-1, 1]";
  input Real width "Half-width of the dead band, in [0, 0.99]";
  output Real result "Zero inside the dead band, +/-1 at full input";
protected
  Real clipped;
  Real band;
algorithm
  clipped := MathUtilities.clip(value, -1.0, 1.0);
  band := MathUtilities.clip(width, 0.0, 0.99);
  result := if abs(clipped) <= band then
      0.0
    else
      (clipped - sign(clipped) * band) / (1.0 - band);
  annotation(
    Inline = true,
    Documentation(info = "<html>
      <p>The PX4 stick dead band, <code>math::deadzone</code> in
      <code>src/lib/mathlib/math/Functions.hpp</code>. The output is continuous
      at the band edge and reaches unit magnitude at unit input, so the dead
      band costs travel rather than authority.</p>
    </html>"));
end deadzone;
