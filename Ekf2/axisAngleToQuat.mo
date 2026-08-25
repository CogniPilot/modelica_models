within Ekf2;
function axisAngleToQuat
  "Hamilton scalar-first quaternion from an axis-angle (rotation) vector"
  // Transcribes matrix::Quaternion<Type>::Quaternion(const AxisAngle<Type>&),
  // src/lib/matrix/matrix/Quaternion.hpp:170, PX4-Autopilot commit
  // bd62df5e3ac3f3f4a07da4518062a922492adb6c. angle = |v|, axis = v/|v|,
  // q = {cos(angle/2), axis*sin(angle/2)}; identity for angle < 1e-10.
  input Real v[3] "rotation vector (axis times angle), rad";
  output Real q[4] "unit quaternion {w,x,y,z}";
protected
  Real angle;
  Real mag;
  Real c;
algorithm
  angle := sqrt(v[1] * v[1] + v[2] * v[2] + v[3] * v[3]);
  if angle < 1e-10 then
    q[1] := 1.0;
    q[2] := 0.0;
    q[3] := 0.0;
    q[4] := 0.0;
  else
    mag := sin(angle / 2.0);
    c := cos(angle / 2.0);
    q[1] := c;
    q[2] := (v[1] / angle) * mag;
    q[3] := (v[2] / angle) * mag;
    q[4] := (v[3] / angle) * mag;
  end if;
end axisAngleToQuat;
