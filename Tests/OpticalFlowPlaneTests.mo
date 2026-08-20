within Tests;

model OpticalFlowPlaneTests
  "Plane-induced flow, slant range, and analytic range-Jacobian checks"
  function run
    output Boolean passed;
  protected
    constant Real tolerance = 2.0e-5;
    Real flatLos[2];
    Real flatRange;
    Real flatVisible;
    Real inclinedLos[2];
    Real inclinedRange;
    Real inclinedVisible;
    Real inclinedNormal[3];
    Estimation.StrapdownINS.InitialVariances variances;
    Estimation.StrapdownINS.ESKF.State nominal;
    Estimation.StrapdownINS.ESKF.NominalState nominalState;
    Estimation.StrapdownINS.ESKF.NominalState perturbed;
    Real range;
    Real perturbedRange;
    Real H[15];
    Real delta[15];
  algorithm
    inclinedNormal := {0.2, -0.1, sqrt(0.95)};
    (flatLos, flatRange, flatVisible) :=
      Vehicles.Rdd2.simulateOpticalFlowPlane(
        {0.0, 0.0, 2.0}, {1.0, -0.5, 0.2}, identity(3),
        {0.1, -0.2, 0.3}, {0.0, 0.0, 1.0}, 0.0, 0.01, 0.35);
    (inclinedLos, inclinedRange, inclinedVisible) :=
      Vehicles.Rdd2.simulateOpticalFlowPlane(
        {0.0, 0.0, 2.0}, {1.0, -0.5, 0.2}, identity(3),
        {0.1, -0.2, 0.3}, inclinedNormal, 0.0, 0.01, 0.35);
    assert(flatVisible > 0.5 and inclinedVisible > 0.5,
      "The configured ground plane was not visible to the feature grid");
    assert(abs(flatRange - 2.0) < tolerance
        and Tests.Assertions.maxAbsVector(flatLos - {0.0015, 0.007})
          < tolerance,
      "Flat-plane flow is not calibrated to translation/range plus body rotation");
    assert(Tests.Assertions.maxAbsVector(inclinedLos - flatLos) > 1.0e-6,
      "Inclining the observed plane had no effect on the simulated raw image flow");

    variances := Estimation.StrapdownINS.InitialVariances(
      position_m2=fill(1.0, 3), velocity_m2_s2=fill(1.0, 3),
      attitude_rad2=fill(0.1, 3),
      gyroscopeBias_rad2_s2=fill(1.0e-4, 3),
      accelerometerBias_m2_s4=fill(1.0e-2, 3));
    nominal := Estimation.StrapdownINS.ESKF.initialize(
      {1.0, -0.5, 4.0}, {1.0, 0.0, 0.0, 0.0}, variances);
    (range, H) := Estimation.StrapdownINS.ESKF.opticalFlowRangeJacobian(
      nominal, inclinedNormal, 0.3);
    nominalState := Estimation.StrapdownINS.ESKF.NominalState(
      positionWorldEnu_m=nominal.positionWorldEnu_m,
      velocityWorldEnu_m_s=nominal.velocityWorldEnu_m_s,
      quaternionWorldBody=nominal.quaternionWorldBody,
      gyroscopeBiasBodyFlu_rad_s=nominal.gyroscopeBiasBodyFlu_rad_s,
      accelerometerBiasBodyFlu_m_s2=
        nominal.accelerometerBiasBodyFlu_m_s2);
    for index in 1:9 loop
      if index <= 3 or index >= 7 then
        delta := zeros(15);
        delta[index] := 1.0e-6;
        perturbed := Estimation.StrapdownINS.ESKF.inject(nominalState, delta);
        perturbedRange := Estimation.StrapdownINS.rangeToPlane(
          perturbed.positionWorldEnu_m,
          perturbed.quaternionWorldBody,
          inclinedNormal, 0.3);
        assert(abs((perturbedRange - range) / 1.0e-6 - H[index])
            < tolerance,
          "Optical-flow range Jacobian failed finite difference at tangent index "
            + String(index));
      end if;
    end for;
    passed := true;
  end run;

  parameter Boolean passed = run();
equation
  assert(passed, "Optical-flow plane tests did not complete");
end OpticalFlowPlaneTests;
