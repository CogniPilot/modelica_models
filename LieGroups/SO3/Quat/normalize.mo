within LieGroups.SO3.Quat;
function normalize "Normalize quaternion to unit length"
  input Real q[4];
  output Real q_n[4];
protected
  Real n;
  constant Real tolerance = 1.0e-12;
algorithm
  n := sqrt(q[1]^2 + q[2]^2 + q[3]^2 + q[4]^2);
  // Keep the zero-input fallback total so generated-code evaluators do not
  // divide by zero while constructing inactive branches.
  q_n := if n > tolerance then q / n else {1.0, 0.0, 0.0, 0.0};
end normalize;
