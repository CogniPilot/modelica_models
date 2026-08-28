within MathUtilities;
function expo "Blend a linear and a cubic response curve"
  input Real value "Normalized input, nominally in [-1, 1]";
  input Real fraction "Cubic fraction, in [0, 1]; 0 is linear";
  output Real result "Shaped output, +/-1 at +/-1 input";
protected
  Real clipped;
  Real blend;
algorithm
  clipped := MathUtilities.clip(value, -1.0, 1.0);
  blend := MathUtilities.clip(fraction, 0.0, 1.0);
  result := (1.0 - blend) * clipped + blend * clipped ^ 3;
  annotation(
    Inline = true,
    Documentation(info = "<html>
      <p>The PX4 stick expo curve, <code>math::expo</code> in
      <code>src/lib/mathlib/math/Functions.hpp</code>. It is odd, monotone for
      every admissible <code>fraction</code>, fixes the endpoints at
      <code>+/-1</code>, and softens the slope around center to
      <code>1 - fraction</code>.</p>
    </html>"));
end expo;
