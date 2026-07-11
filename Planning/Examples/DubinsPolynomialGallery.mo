within Planning.Examples;
model DubinsPolynomialGallery "Four reproducible smooth trajectory examples"
  Planning.Examples.DubinsPolynomialSample case1;
  Planning.Examples.DubinsPolynomialSample case2(
    startPosition={3.0, -2.0},
    startHeading=0.6,
    goalPosition={2.5127, 10.1974},
    goalHeading=2.3,
    turnRadius=1.7);
  Planning.Examples.DubinsPolynomialSample case3(
    startPosition={-6.0, -4.0},
    startHeading=-0.4,
    goalPosition={12.0, 8.6},
    goalHeading=1.3,
    turnRadius=3.06);
  Planning.Examples.DubinsPolynomialSample case4(
    startPosition={-3.0, 1.0},
    startHeading=0.8,
    goalPosition={-11.0, 7.0},
    goalHeading=2.4,
    turnRadius=1.4);
  annotation(experiment(StartTime=0.0, StopTime=1.0, Interval=0.0025));
end DubinsPolynomialGallery;
