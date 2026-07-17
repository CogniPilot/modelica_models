within Vehicles.Interfaces;
function eulerFromQuaternion "Return roll, pitch, yaw from a scalar-first quaternion"
  input Real quaternion[4];
  output Real euler_rad[3] "Roll, pitch, yaw [rad]";
protected
  Real yawPitchRoll_rad[3];
algorithm
  yawPitchRoll_rad := LieGroups.SO3.EulerB321.from_Quat(quaternion);
  euler_rad := {yawPitchRoll_rad[3], yawPitchRoll_rad[2], yawPitchRoll_rad[1]};
  annotation(Inline = true);
end eulerFromQuaternion;
