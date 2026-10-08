within Tests;

model CorrelatedGpsTests
  "Held-input uncertainty and correlated delayed GPS correction"
  function run
    output Boolean passed;
  protected
    constant Real pi = 2.0 * asin(1.0);
    Real A[15, 15];
    Real forwardInput[15, 6];
    Real backwardInput[15, 6];
    Real inputCovariance[6, 6];
    Real H[1, 15];
    Real C[15, 1];
    Estimation.StrapdownINS.ESKF.State prior;
    Estimation.StrapdownINS.ESKF.State corrected;
    Avionics.GpsSample gps;
    Boolean accepted;
    Integer reason;
    Real nis;
    Real seedVariances[3];
  algorithm
    seedVariances := Estimation.StrapdownINS.ESKF.seedPositionVariances(
      0.25 * identity(3), {1.0, 0.0, 0.0, 0.0}, fill(0.04, 3));
    assert(max(abs(seedVariances - fill(0.25, 3))) < 1e-12,
      "A noisy GPS seed retained an overconfident configured position prior");
    seedVariances := Estimation.StrapdownINS.ESKF.seedPositionVariances(
      diagonal({0.01, 0.09, 0.04}),
      {cos(pi / 8), 0.0, 0.0, sin(pi / 8)},
      {0.02, 0.02, 0.10});
    assert(max(abs(seedVariances - {0.09, 0.09, 0.10})) < 1e-12,
      "Seed covariance rotation, off-diagonal majorant or configured floor is wrong");
    seedVariances := Estimation.StrapdownINS.ESKF.seedPositionVariances(
      zeros(3, 3), {1.0, 0.0, 0.0, 0.0}, {0.02, 0.03, 0.04});
    assert(max(abs(seedVariances - {0.02, 0.03, 0.04})) < 1e-12,
      "An absent seed covariance changed the configured initialization policy");
    prior := Estimation.StrapdownINS.ESKF.initialize(
      zeros(3), {1.0, 0.0, 0.0, 0.0},
      Estimation.StrapdownINS.InitialVariances(
        position_m2=fill(0.04, 3), velocity_m2_s2=fill(0.01, 3),
        attitude_rad2=fill(0.01, 3), gyroscopeBias_rad2_s2=fill(0.01, 3),
        accelerometerBias_m2_s4=fill(0.01, 3)),
      zeros(3), zeros(3), zeros(3), 0.25 * identity(3));
    assert(abs(prior.covariance[1, 1] - 0.25) < 1e-12,
      "Initializer discarded the uncertainty of the supplied aiding seed");

    A := Estimation.StrapdownINS.ESKF.continuousTransition(zeros(3), zeros(3));
    forwardInput := Estimation.StrapdownINS.ESKF.heldInputJacobian(A, 0.01);
    backwardInput := Estimation.StrapdownINS.ESKF.heldInputJacobian(A, -0.11);
    assert(abs(forwardInput[4, 4] + 0.01) < 1e-12
      and abs(forwardInput[1, 4] + 0.00005) < 1e-12
      and abs(backwardInput[4, 4] - 0.11) < 1e-12
      and abs(backwardInput[1, 4] + 0.00605) < 1e-12,
      "Held-input transport has the wrong sign or integration interval");

    prior := Estimation.StrapdownINS.ESKF.initialize(
      zeros(3), {1.0, 0.0, 0.0, 0.0},
      Estimation.StrapdownINS.InitialVariances(
        position_m2=fill(0.01, 3), velocity_m2_s2=fill(0.0103, 3),
        attitude_rad2=fill(0.01, 3), gyroscopeBias_rad2_s2=fill(0.01, 3),
        accelerometerBias_m2_s4=fill(0.01, 3)));
    H := zeros(1, 15);
    H[1, 4] := 1.0;
    C := zeros(15, 1);
    C[4, 1] := -0.0033;
    // P=0.0103, C=-0.0033, R=0.0463 give S=0.05 and K=0.14.
    // The generalized Joseph posterior is P-(P+C)^2/S=0.00932.
    (corrected, accepted, reason, nis) :=
      Estimation.StrapdownINS.ESKF.correctLinear(
        prior, {0.1}, H, {{0.0463}}, 0.0, zeros(3), C);
    assert(accepted and abs(corrected.velocityWorldEnu_m_s[1] - 0.014) < 1e-10
      and abs(corrected.covariance[4, 4] - 0.00932) < 1e-10
      and abs(nis - 0.2) < 1e-10,
      "Correlated observation gain, NIS or Joseph covariance is wrong");

    C[4, 1] := 0.04;
    (corrected, accepted, reason, nis) :=
      Estimation.StrapdownINS.ESKF.correctLinear(
        prior, {0.1}, H, {{0.01}}, 0.0, zeros(3), C);
    assert(not accepted
      and reason == Estimation.StrapdownINS.CorrectionRejectedCovarianceUnusable
      and max(abs(corrected.covariance - prior.covariance)) == 0.0
      and max(abs(corrected.velocityWorldEnu_m_s
        - prior.velocityWorldEnu_m_s)) == 0.0,
      "An impossible joint covariance passed a positive innovation check");

    inputCovariance := cat(1, cat(2, zeros(3, 3), zeros(3, 3)),
      cat(2, zeros(3, 3), 3.0 * identity(3)));
    prior := Estimation.StrapdownINS.ESKF.State(
      positionWorldEnu_m=zeros(3), velocityWorldEnu_m_s=zeros(3),
      quaternionWorldBody={1.0, 0.0, 0.0, 0.0},
      gyroscopeBiasBodyFlu_rad_s=zeros(3), accelerometerBiasBodyFlu_m_s2=zeros(3),
      covariance=0.01 * identity(15)
        + forwardInput * inputCovariance * transpose(forwardInput),
        useSquareRootCovariance=false, covarianceRoot=zeros(15, 15));
    gps := Avionics.GpsSample(valid=true, fresh=true, timestamp_s=-0.11,
      positionValid=true, velocityValid=true, geodetic_deg_m=zeros(3),
      positionWorldEnu_m=zeros(3), velocityWorldEnu_m_s={0.1, 0.0, 0.0},
      positionCovarianceWorld_m2=0.25 * identity(3),
      velocityCovarianceWorld_m2_s2=0.01 * identity(3));
    (corrected, accepted, reason, nis) := Estimation.StrapdownINS.ESKF.correctGps(
      prior, gps, 0.0, 0.11, zeros(3), zeros(3), zeros(3), 0.25,
      inputCovariance, 0.01);
    // Independent linear-Gaussian reference for each {position,velocity,bias}
    // block, with the newest sample present in both forward and backward maps.
    assert(accepted
      and abs(corrected.positionWorldEnu_m[1] - 0.00017005291263822) < 1e-10
      and abs(corrected.velocityWorldEnu_m_s[1] - 0.01395448391363927) < 1e-10
      and abs(corrected.covariance[4, 4] - 0.00932104565798844) < 1e-10
      and abs(nis - 0.1996211731027336) < 1e-10,
      "Delayed GPS does not marginalize the held-input uncertainty and correlation");
    passed := true;
  end run;
  parameter Boolean passed = run();
equation
  assert(passed, "Correlated delayed GPS checks did not complete");
end CorrelatedGpsTests;
