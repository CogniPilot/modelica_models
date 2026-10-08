within SLAM.Inertial;
// An acquisition interval is a held measurement, not an integration substep.
// This bound covers the supported sensor rates without fabricating samples.
function ES15HeldIntervalValid
  input Real h;
  output Boolean valid;
algorithm
  valid := h > 0.0 and h <= 0.2;
end ES15HeldIntervalValid;
