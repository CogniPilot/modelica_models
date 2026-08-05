within Estimation.PositionVelocityKF;
function bodySpecificForceToWorld
  "Rotate IMU specific force to world ENU and add gravity"
  input Real attitude_wb[4]
    "Scalar-first quaternion rotating body FLU vectors to world ENU";
  input Real specificForce_b_m_s2[3]
    "Accelerometer specific force in body FLU coordinates [m/s2]";
  input Real gravity_m_s2 = 9.81 "Positive gravitational acceleration";
  output Real acceleration_w_m_s2[3]
    "Gravity-compensated inertial acceleration in world ENU [m/s2]";
protected
  Real normalizedAttitude[4];
algorithm
  normalizedAttitude := LieGroups.SO3.Quat.normalize(attitude_wb);
  acceleration_w_m_s2 := LieGroups.SO3.Quat.rotate(
    normalizedAttitude,
    specificForce_b_m_s2) + {0.0, 0.0, -gravity_m_s2};
end bodySpecificForceToWorld;
