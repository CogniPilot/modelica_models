within Estimation.PositionVelocityKF;
function initialize "Initialize translational state"
  input Real position[3] "Initial position in world ENU coordinates [m]";
  input Real velocity[3] = zeros(3)
    "Initial velocity in world ENU coordinates [m/s]";
  output Estimation.PositionVelocityKF.State state;
algorithm
  state.position := position;
  state.velocity := velocity;
end initialize;
