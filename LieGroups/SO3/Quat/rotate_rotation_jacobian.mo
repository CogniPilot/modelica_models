within LieGroups.SO3.Quat;
function rotate_rotation_jacobian
  "Right-trivialized derivative of rotate with respect to its quaternion"
  input Real q[4] "Unit quaternion {w,x,y,z}";
  input Real v[3] "Vector being rotated";
  output Real J[3, 3] "d/d(dq) of (rotate(q*exp_map(dq), v) - rotate(q, v))";
algorithm
  J := -LieGroups.SO3.Quat.to_DCM(q) * LieGroups.SO3.Quat.wedge(v);
  annotation(Documentation(info="<html>
    <p>Rule for <code>rotate</code>, rotation slot. The quaternion is perturbed
    on the right; the result is a plain vector and is read back additively:</p>
    <pre>rotate(q*exp_map(dq), v) - rotate(q, v) = -R(q) [v]x dq + O(|dq|^2)</pre>
  </html>"));
end rotate_rotation_jacobian;
