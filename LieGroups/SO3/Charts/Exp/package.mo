within LieGroups.SO3.Charts;
package Exp "Exponential coordinates (rotation vector); from_Quat is the principal log, so valid for |delta| < pi"
  extends LieGroups.SO3.Charts.PartialChart;

  redeclare function to_Quat
    input Real delta[3];
    output Real q[4];
  algorithm
    q := LieGroups.SO3.Quat.exp_map(delta);
  end to_Quat;

  redeclare function from_Quat
    input Real q[4];
    output Real delta[3];
  algorithm
    delta := LieGroups.SO3.Quat.log_map(q);
  end from_Quat;

  redeclare function to_Quat_jacobian
    input Real delta[3];
    output Real J[3, 3];
  algorithm
    J := LieGroups.SO3.Quat.right_jacobian_exact(delta);
  end to_Quat_jacobian;

  redeclare function to_Quat_jacobian_inv
    input Real delta[3];
    output Real J_inv[3, 3];
  algorithm
    J_inv := LieGroups.SO3.Quat.right_jacobian_inv_exact(delta);
  end to_Quat_jacobian_inv;

  redeclare function inDomain
    input Real delta[3];
    output Boolean valid;
  algorithm
    // from_Quat is the principal log (|delta| <= pi), so local(retract(delta)) = delta needs |delta| < pi
    valid := delta * delta < (2.0 * asin(1.0))^2;
  end inDomain;
end Exp;
