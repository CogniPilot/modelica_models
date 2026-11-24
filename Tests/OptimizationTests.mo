within Tests;
model OptimizationTests "Box and friction-cone QPs against known optima"
  constant Real tolerance = 1.0e-9;
  constant Real big = 1.0e30 "Stands in for an absent bound";
  // Box, diagonal H: x* = clamp(-g ./ diag(H), lower, upper); one free, one at lower, one at upper.
  constant Real Hdiag[3, 3] = [2.0, 0.0, 0.0; 0.0, 4.0, 0.0; 0.0, 0.0, 1.0];
  constant Real gdiag[3] = {-2.0, 8.0, -5.0};
  // Box, coupled 2x2: unconstrained (7/3, -2/3) violates x2 >= 0, so x2 = 0 and x1 = 2.
  constant Real Hpair[2, 2] = [2.0, 1.0; 1.0, 2.0];
  constant Real gpair[2] = {-4.0, -1.0};
  // Box, wide bounds: x* = -H \ g = (-4/18, -2/18, -26/18).
  constant Real Hfree[3, 3] = [4.0, 1.0, 0.0; 1.0, 3.0, 1.0; 0.0, 1.0, 2.0];
  constant Real gfree[3] = {1.0, 2.0, 3.0};
  // Box, coupled 3x3 with every kind of active entry: x* = (1, 0, 0.25).
  constant Real Hmix[3, 3] = [4.0, 1.0, 0.5; 1.0, 3.0, 0.8; 0.5, 0.8, 2.0];
  constant Real gmix[3] = {-6.0, 2.0, -1.0};
  // Cone, A = I: the optimum is the cone projection of -b.
  constant Real I3[3, 3] = [1.0, 0.0, 0.0; 0.0, 1.0, 0.0; 0.0, 0.0, 1.0];
  // Two contacts, A = diag(2, 2, 2, 1, 1, 1): per block the optimum is proj(-b_c / k_c).
  constant Real Atwo[6, 6] = [
    2.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    0.0, 2.0, 0.0, 0.0, 0.0, 0.0;
    0.0, 0.0, 2.0, 0.0, 0.0, 0.0;
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0;
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0;
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0];
  // Two coupled contacts: no closed form; the reference was confirmed by an independent
  // full projected-gradient solve to 1.1e-12 with the same objective -4.411652092472315.
  constant Real Acoupled[6, 6] = [
    2.0, 0.3, 0.1, 0.5, 0.0, 0.0;
    0.3, 1.5, 0.0, 0.0, 0.2, 0.0;
    0.1, 0.0, 1.5, 0.0, 0.0, 0.2;
    0.5, 0.0, 0.0, 2.0, 0.3, 0.1;
    0.0, 0.2, 0.0, 0.3, 1.5, 0.0;
    0.0, 0.0, 0.2, 0.1, 0.0, 1.5];
  constant Real bcoupled[6] = {-3.0, -2.0, 1.0, -2.0, 1.5, 0.5};
  constant Real fcoupled[6] = {
    1.4150093689882137, 0.4767197506880429, -0.30512050340125946,
    1.0689052726075128, -0.7195998231458656, -0.20501158653702525};
  Real boxDiagonal;
  Real boxPair;
  Real boxFree;
  Real boxMixed;
  Real warmStartSweeps;
  Real coneInside;
  Real conePolar;
  Real coneBoundary;
  Real coneTwo;
  Real coneCoupled;
  Real frictionlessTensile;
  Real frictionlessSliding;
equation
  boxDiagonal = Tests.Assertions.boxQPCheck(Hdiag, gdiag, {-1.0, -1.0, -1.0}, {3.0, 3.0, 3.0},
    {1.0, -1.0, 3.0}, {0, -1, 1});
  boxPair = Tests.Assertions.boxQPCheck(Hpair, gpair, {0.0, 0.0}, {big, 1.0}, {2.0, 0.0}, {0, -1});
  boxFree = Tests.Assertions.boxQPCheck(Hfree, gfree, {-big, -big, -big}, {big, big, big},
    {-4.0 / 18.0, -2.0 / 18.0, -26.0 / 18.0}, {0, 0, 0});
  boxMixed = Tests.Assertions.boxQPCheck(Hmix, gmix, {0.0, 0.0, 0.0}, {1.0, 1.0, 1.0},
    {1.0, 0.0, 0.25}, {1, -1, 0});
  warmStartSweeps = Tests.Assertions.boxQPWarmStartIterations(Hmix, gmix, {0.0, 0.0, 0.0},
    {1.0, 1.0, 1.0}, {1.0, 0.0, 0.25});
  coneInside = Tests.Assertions.frictionConeQPCheck({0.5}, I3, {-2.0, 0.3, -0.4}, {2.0, -0.3, 0.4}, {1});
  conePolar = Tests.Assertions.frictionConeQPCheck({0.5}, I3, {2.0, 0.1, 0.0}, {0.0, 0.0, 0.0}, {0});
  coneBoundary = Tests.Assertions.frictionConeQPCheck({0.5}, I3, {-1.0, -3.0, -4.0}, {2.8, 0.84, 1.12}, {2});
  coneTwo = Tests.Assertions.frictionConeQPCheck({0.6, 0.6}, Atwo, {-4.0, -6.0, 0.0, -1.0, 0.2, 0.1},
    {9.5 / 3.4, 5.7 / 3.4, 0.0, 1.0, -0.2, -0.1}, {2, 1});
  coneCoupled = Tests.Assertions.frictionConeQPCheck({0.4, 0.7}, Acoupled, bcoupled, fcoupled, {2, 2});
  // mu = 0: a pulling (negative) normal must give zero force; a pushing one keeps its normal and drops the tangents.
  frictionlessTensile = Tests.Assertions.frictionConeQPCheck({0.0}, I3, {1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0});
  frictionlessSliding = Tests.Assertions.frictionConeQPCheck({0.0}, I3, {-2.0, 0.5, -0.3}, {2.0, 0.0, 0.0}, {2});

  assert(boxDiagonal < tolerance, "boxQP diagonal case does not match its closed-form optimum");
  assert(boxPair < tolerance, "boxQP coupled 2x2 case does not match its KKT solution");
  assert(boxFree < tolerance, "boxQP with wide bounds does not match -H \\ g");
  assert(boxMixed < tolerance, "boxQP mixed active-set case does not match its KKT solution");
  assert(abs(warmStartSweeps) < 0.5, "boxQP warm-started at the optimum should need no sweeps");
  assert(coneInside < tolerance, "frictionConeQP interior case does not match the cone projection");
  assert(conePolar < tolerance, "frictionConeQP polar-cone case should give zero force");
  assert(coneBoundary < tolerance, "frictionConeQP boundary case does not match the cone projection");
  assert(coneTwo < tolerance, "frictionConeQP two-contact case does not match the per-block optimum");
  assert(coneCoupled < tolerance, "frictionConeQP coupled case does not match the reference optimum");
  assert(frictionlessTensile < tolerance, "frictionConeQP with mu = 0 must not return a tensile normal force");
  assert(frictionlessSliding < tolerance, "frictionConeQP with mu = 0 must keep the normal and remove the tangents");
  annotation(experiment(StopTime = 1.0, Tolerance = 1.0e-8));
end OptimizationTests;
