within LieGroups.SO3.Charts;
partial package PartialChart
  "Contract implemented by replaceable local coordinate charts on SO(3)"
  constant Boolean symmetric = true
    "true when to_Quat(-delta) = to_Quat(delta)^(-1); then the transition to Exp coordinates is identity + O(|delta|^3)";

  replaceable partial function to_Quat "Chart coordinates -> unit quaternion {w,x,y,z} about the identity"
    input Real delta[3] "Chart coordinates";
    output Real q[4] "Unit quaternion {w,x,y,z}";
  end to_Quat;

  replaceable partial function from_Quat "Unit quaternion {w,x,y,z} -> chart coordinates about the identity"
    input Real q[4] "Unit quaternion {w,x,y,z}";
    output Real delta[3] "Chart coordinates";
  end from_Quat;

  replaceable partial function to_Quat_jacobian
    "Rule for to_Quat: log_map(to_Quat(delta)^(-1) * to_Quat(delta + d)) = J*d; chart rate -> body angular velocity"
    input Real delta[3] "Chart coordinates";
    output Real J[3, 3];
  end to_Quat_jacobian;

  replaceable partial function to_Quat_jacobian_inv "Inverse of to_Quat_jacobian; body angular velocity -> chart rate"
    input Real delta[3] "Chart coordinates";
    output Real J_inv[3, 3];
  end to_Quat_jacobian_inv;

  replaceable function inDomain "true where the chart is valid and to_Quat_jacobian is invertible"
    input Real delta[3] "Chart coordinates";
    output Boolean valid;
  algorithm
    valid := true;
  end inDomain;

  function retract "q0 * to_Quat(delta)"
    input Real q0[4] "Base orientation {w,x,y,z}";
    input Real delta[3] "Chart coordinates";
    output Real q[4] "Orientation {w,x,y,z}";
  algorithm
    q := LieGroups.SO3.Quat.product(q0, to_Quat(delta));
  end retract;

  function local "from_Quat(q0^(-1) * q)"
    input Real q0[4] "Base orientation {w,x,y,z}";
    input Real q[4] "Orientation {w,x,y,z}";
    output Real delta[3] "Chart coordinates";
  algorithm
    delta := from_Quat(LieGroups.SO3.Quat.product(LieGroups.SO3.Quat.inverse(q0), q));
  end local;

  function retract_delta_jacobian
    "Rule for retract in its delta slot: log_map(retract(q0, delta)^(-1) * retract(q0, delta + d)) = J*d"
    input Real q0[4] "Base orientation {w,x,y,z}";
    input Real delta[3] "Chart coordinates";
    output Real J[3, 3];
  algorithm
    J := to_Quat_jacobian(delta);
  end retract_delta_jacobian;

  function rate "Chart rate for a body-frame angular velocity: der(delta) = J(delta)^(-1) * omega"
    input Real delta[3] "Chart coordinates";
    input Real omega[3](each unit = "rad/s") "Angular velocity, body frame";
    output Real deltaDot[3] "Chart coordinate rate";
  algorithm
    deltaDot := to_Quat_jacobian_inv(delta) * omega;
  end rate;
end PartialChart;
