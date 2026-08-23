within LieGroups.SO3.Quat;
function rotate_vector_jacobian
  "Derivative of rotate with respect to the vector being rotated"
  input Real q[4] "Unit quaternion {w,x,y,z}";
  input Real v[3] "Vector being rotated";
  output Real J[3, 3] "d/d(dv) of (rotate(q, v + dv) - rotate(q, v))";
algorithm
  J := LieGroups.SO3.Quat.to_DCM(q);
  annotation(Documentation(info="<html>
    <p>Rule for <code>rotate</code>, vector slot. Both slot and result are plain
    vectors, so both are additive and the rule is the rotation matrix itself:</p>
    <pre>rotate(q, v + dv) - rotate(q, v) = R(q) dv</pre>
    <p>The identity is exact and independent of v.</p>
  </html>"));
end rotate_vector_jacobian;
