within LieGroups.SO3.Charts;
package Cayley "Cayley (Gibbs/Rodrigues) chart, delta = 2*tan(theta/2)*axis; reaches every rotation except theta = pi"
  extends LieGroups.SO3.Charts.PartialChart;

  redeclare function to_Quat
    input Real delta[3];
    output Real q[4];
  protected
    Real g[3] = delta / 2.0 "Gibbs vector tan(theta/2)*axis";
  algorithm
    q := cat(1, {1.0}, g) / sqrt(1.0 + g * g);
  end to_Quat;

  redeclare function from_Quat
    input Real q[4];
    output Real delta[3];
  protected
    Real q_n[4];
  algorithm
    q_n := LieGroups.SO3.Quat.normalize(q);
    q_n := if q_n[1] < 0 then -q_n else q_n;
    delta := 2.0 * q_n[2:4] / q_n[1];
  end from_Quat;

  redeclare function to_Quat_jacobian
    input Real delta[3];
    output Real J[3, 3] "(I - [g]x) / (1 + |g|^2), g = delta/2";
  protected
    Real g[3] = delta / 2.0;
  algorithm
    J := (identity(3) - LieGroups.SO3.Quat.wedge(g)) / (1.0 + g * g);
  end to_Quat_jacobian;

  redeclare function to_Quat_jacobian_inv
    input Real delta[3];
    output Real J_inv[3, 3] "I + [g]x + g*g', g = delta/2";
  protected
    Real g[3] = delta / 2.0;
  algorithm
    J_inv := identity(3) + LieGroups.SO3.Quat.wedge(g) + {{g[i] * g[j] for j in 1:3} for i in 1:3};
  end to_Quat_jacobian_inv;
end Cayley;
