within LieGroups.SO3.EulerB321;
function from_Quat "Convert quaternion to Euler B321 angles {yaw, pitch, roll}"
  input Real q[4] "Unit quaternion {w,x,y,z}";
  output Real euler[3] "{yaw (psi), pitch (theta), roll (phi)} [rad]";
protected
  Real a, b, c, d;
  Real sinp;
algorithm
  a := q[1]; b := q[2]; c := q[3]; d := q[4];

  // Pitch: theta = asin(2*(ac - bd))
  sinp := 2.0*(a*c - d*b);
  sinp := min(max(sinp, -1.0), 1.0);

  // Each branch assembles the whole {yaw, pitch, roll} triple with one vector
  // assignment. The galec production lowering keeps only the last per-element
  // write inside a conditional branch, so writing euler[1..3] separately would
  // drop yaw and pitch; the single vector form emits all three components.
  if sinp * sinp > 0.9999 * 0.9999 then
    // Gimbal lock: pitch near +/- 90 deg
    euler := {atan2(2.0*(b*c + a*d), 1.0 - 2.0*(c*c + d*d)),
              asin(sinp),
              0.0};
  else
    euler := {atan2(2.0*(a*d + b*c), 1.0 - 2.0*(c*c + d*d)),
              asin(sinp),
              atan2(2.0*(a*b + c*d), 1.0 - 2.0*(b*b + c*c))};
  end if;
end from_Quat;
