within Planning.DubinsPolynomial;
function coordinatedRollAngle
  "Coordinated-flight roll angle associated with signed planar curvature"
  input Real curvature;
  input Real speed(min=0.0);
  input Real gravity(min=0.0) = 9.80665;
  output Real rollAngle;
algorithm
  assert(gravity > 0.0, "Gravity must be positive for coordinated roll mapping");
  rollAngle := atan(speed^2 * curvature / gravity);
end coordinatedRollAngle;
