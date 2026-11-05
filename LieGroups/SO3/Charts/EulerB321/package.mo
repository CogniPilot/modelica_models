within LieGroups.SO3.Charts;
package EulerB321
  "Euler B321 chart with delta = {roll, pitch, yaw}, ordered so the Jacobian at zero is the identity; singular at pitch = +-pi/2"
  extends LieGroups.SO3.Charts.PartialChart(final symmetric = false);

  redeclare function to_Quat
    input Real delta[3] "{roll (phi), pitch (theta), yaw (psi)} [rad]";
    output Real q[4];
  algorithm
    q := LieGroups.SO3.EulerB321.to_Quat({delta[3], delta[2], delta[1]});
  end to_Quat;

  redeclare function from_Quat
    input Real q[4];
    output Real delta[3] "{roll (phi), pitch (theta), yaw (psi)} [rad]";
  protected
    Real euler[3] "{yaw, pitch, roll}";
  algorithm
    euler := LieGroups.SO3.EulerB321.from_Quat(LieGroups.SO3.Quat.normalize(q));
    delta := {euler[3], euler[2], euler[1]};
  end from_Quat;

  redeclare function to_Quat_jacobian
    input Real delta[3] "{roll, pitch, yaw}";
    output Real J[3, 3] "Body angular velocity from {roll, pitch, yaw} rates";
  algorithm
    J := [1, 0, -sin(delta[2]);
          0, cos(delta[1]), sin(delta[1]) * cos(delta[2]);
          0, -sin(delta[1]), cos(delta[1]) * cos(delta[2])];
  end to_Quat_jacobian;

  redeclare function to_Quat_jacobian_inv
    input Real delta[3] "{roll, pitch, yaw}";
    output Real J_inv[3, 3];
  protected
    Real sr = sin(delta[1]);
    Real cr = cos(delta[1]);
    Real tp = tan(delta[2]);
    Real cp = cos(delta[2]);
  algorithm
    J_inv := [1, sr * tp, cr * tp;
              0, cr, -sr;
              0, sr / cp, cr / cp];
  end to_Quat_jacobian_inv;

  redeclare function inDomain
    input Real delta[3];
    output Boolean valid;
  algorithm
    // Principal angles, so local(retract(delta)) = delta. cos(pitch) is periodic, so bound the pitch interval
    // explicitly: |pitch| < acos(1e-3) < pi/2 (cond(J) below about 1e3), and |roll|, |yaw| < pi.
    valid := abs(delta[2]) < acos(1e-3) and abs(delta[1]) < 2.0 * asin(1.0) and abs(delta[3]) < 2.0 * asin(1.0);
  end inDomain;
end EulerB321;
