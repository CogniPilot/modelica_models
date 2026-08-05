within Estimation.PositionVelocityKF;
function predict "Constant-acceleration translational prediction"
  input Estimation.PositionVelocityKF.State previous;
  input Real acceleration_w_m_s2[3]
    "Inertial acceleration in world ENU coordinates [m/s2]";
  input Real dt "Prediction interval [s]";
  output Estimation.PositionVelocityKF.State predicted;
algorithm
  predicted.position := previous.position + previous.velocity * dt
    + 0.5 * acceleration_w_m_s2 * dt * dt;
  predicted.velocity := previous.velocity + acceleration_w_m_s2 * dt;
end predict;
