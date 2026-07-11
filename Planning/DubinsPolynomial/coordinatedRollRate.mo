within Planning.DubinsPolynomial;
function coordinatedRollRate
  "Coordinated-flight roll rate at constant speed"
  input Real curvature;
  input Real curvatureDerivative "Derivative with metric path distance";
  input Real speed(min=0.0);
  input Real gravity(min=0.0) = 9.80665;
  output Real rollRate;
protected
  Real scaledCurvature;
algorithm
  assert(gravity > 0.0, "Gravity must be positive for coordinated roll mapping");
  scaledCurvature := speed^2 * curvature / gravity;
  rollRate := speed^3 * curvatureDerivative
    / (gravity * (1.0 + scaledCurvature^2));
end coordinatedRollRate;
