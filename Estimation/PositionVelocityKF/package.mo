within Estimation;
package PositionVelocityKF
  "Fixed-gain position/velocity Kalman filter with external attitude"
  constant Integer StateLength = 6
    "State dimension for {world position, world velocity}";
  constant Integer MeasurementLength = 3
    "Cartesian position measurement dimension";

  type Vector3 = Real[3] "Three-dimensional coordinate vector";
  type Quaternion = Real[4]
    "Scalar-first Hamilton quaternion {w,x,y,z}";
  type Gain = Real[6, 3]
    "Steady-state gain mapping position residual to state correction";
end PositionVelocityKF;
