within Planning.Examples;
model DubinsAircraftTrajectory
  "Dubins initial guess and a smoothed trajectory certified against a flight envelope"
  extends Planning.Examples.DubinsPolynomialSample(
    allowThreeTurnPaths=false);
  parameter Real aircraftMinimumTurnRadius(min=0.0) = 1.0
    "Hard curvature limit of the aircraft, distinct from the conservative planning radius";
  parameter Real flightSpeed(min=0.0) = 2.0
    "Constant speed used for the coordinated-flight check";
  parameter Real maximumBankAngle(min=0.0) = 0.5;
  parameter Real maximumRollRate(min=0.0) = 3.0;
  output Real coordinatedBankAngle;
  output Real coordinatedBankRate;
equation
  coordinatedBankAngle =
    Planning.DubinsPolynomial.coordinatedRollAngle(curvature, flightSpeed);
  coordinatedBankRate =
    Planning.DubinsPolynomial.coordinatedRollRate(
      curvature, curvatureDerivative, flightSpeed);
  assert(aircraftMinimumTurnRadius > 0.0,
    "The aircraft minimum turn radius must be positive");
  assert(abs(curvature) <= 1.0 / aircraftMinimumTurnRadius + 1.0e-8,
    "Smoothed trajectory exceeds the aircraft curvature limit");
  assert(abs(coordinatedBankAngle) <= maximumBankAngle + 1.0e-8,
    "Smoothed trajectory exceeds the coordinated-flight bank-angle limit");
  assert(abs(coordinatedBankRate) <= maximumRollRate + 1.0e-8,
    "Smoothed trajectory exceeds the coordinated-flight bank-rate limit");
  annotation(experiment(StartTime=0.0, StopTime=1.0, Interval=0.0025));
end DubinsAircraftTrajectory;
