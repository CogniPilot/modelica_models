within LieGroups.SO3.Charts;
package Mrp "Modified Rodrigues chart, delta = 4*tan(theta/4)*axis; domain |delta| < 4 (principal set), singular at theta = 2*pi"
  extends LieGroups.SO3.Charts.PartialChart;

  redeclare function to_Quat
    input Real delta[3];
    output Real q[4];
  algorithm
    q := LieGroups.SO3.Mrp.to_Quat(delta / 4.0);
  end to_Quat;

  redeclare function from_Quat
    input Real q[4];
    output Real delta[3];
  algorithm
    delta := 4.0 * LieGroups.SO3.Mrp.from_Quat(LieGroups.SO3.Quat.normalize(q));
  end from_Quat;

  redeclare function to_Quat_jacobian
    input Real delta[3];
    output Real J[3, 3] "((1 - |p|^2) I - 2[p]x + 2 p p') / (1 + |p|^2)^2, p = delta/4";
  protected
    Real p[3] = delta / 4.0;
    Real s = p * p;
  algorithm
    J := ((1.0 - s) * identity(3) - 2.0 * LieGroups.SO3.Quat.wedge(p) + 2.0 * {{p[i] * p[j] for j in 1:3} for i in 1:3}) / (1.0 + s)^2;
  end to_Quat_jacobian;

  redeclare function to_Quat_jacobian_inv
    input Real delta[3];
    output Real J_inv[3, 3] "(1 - |p|^2) I + 2[p]x + 2 p p', p = delta/4";
  protected
    Real p[3] = delta / 4.0;
  algorithm
    J_inv := (1.0 - p * p) * identity(3) + 2.0 * LieGroups.SO3.Quat.wedge(p) + 2.0 * {{p[i] * p[j] for j in 1:3} for i in 1:3};
  end to_Quat_jacobian_inv;

  redeclare function inDomain
    input Real delta[3];
    output Boolean valid;
  algorithm
    // from_Quat returns the principal (non-shadow) set |p| <= 1; at |p| = 1 (theta = pi) p and its shadow coincide in
    // magnitude, so the round-trip domain is the open ball |delta| < 4.
    valid := delta * delta < 16.0;
  end inDomain;
end Mrp;
