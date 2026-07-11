within Estimation.MocapExternalOdometryIEKF;
function tangentTransition "Discrete tangent-state transition used by the IEKF"
  input Real dt "Prediction interval [s]";
  output Covariance transition;
algorithm
  transition := identity(TangentLength);
  transition[1:3, 10:12] := identity(3) * dt;
  transition[7:9, 4:6] := identity(3) * dt;
end tangentTransition;
