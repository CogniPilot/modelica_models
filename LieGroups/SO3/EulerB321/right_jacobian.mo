within LieGroups.SO3.EulerB321;
function right_jacobian "Map body angular velocity to B321 Euler rates"
  input Real element[3] "{yaw, pitch, roll}";
  output Real J[3, 3];
protected
  Real pitch;
  Real roll;
  Real cosinePitch;
algorithm
  pitch := element[2];
  roll := element[3];
  cosinePitch := cos(pitch);
  J := [
    0.0, sin(roll) / cosinePitch, cos(roll) / cosinePitch;
    0.0, cos(roll), -sin(roll);
    1.0, sin(roll) * tan(pitch), cos(roll) * tan(pitch)];
end right_jacobian;
