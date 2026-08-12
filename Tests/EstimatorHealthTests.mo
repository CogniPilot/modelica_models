within Tests;

model EstimatorHealthTests
  "Precision-scaled pivoting, variance limiting, gating, and auto-reinit"
  function run
    output Boolean passed;
  protected
    constant Real tolerance = 1.0e-6;
    constant Real dt = 0.01;
    constant Integer rejectionLimit = 3;
    constant Real gate = 6.0;
    Real marginal[2, 2];
    Real solution[2, 1];
    Boolean okDefault;
    Boolean okTightened;
    Estimation.MultiSensorInvariant.VarianceLimits limits;
    Estimation.MultiSensorInvariant.Covariance inflated;
    Estimation.MultiSensorInvariant.Covariance limited;
    Estimation.MultiSensorInvariant.InitialVariances initialVariances;
    Estimation.MultiSensorInvariant.State prior;
    Estimation.MultiSensorInvariant.State gatedState;
    Estimation.MultiSensorInvariant.State ungatedState;
    Avionics.GpsSample gpsFar;
    Boolean gatedAccepted;
    Boolean gatedGateRejected;
    Boolean ungatedAccepted;
    Boolean ungatedGateRejected;
    Real position[3];
    Real velocity[3];
    Real quaternion[4];
    Real gyroscopeBias[3];
    Real accelerometerBias[3];
    Estimation.MultiSensorInvariant.Covariance covariance;
    Boolean initialized;
    Real positionNext[3];
    Real velocityNext[3];
    Real quaternionNext[4];
    Real gyroscopeBiasNext[3];
    Real accelerometerBiasNext[3];
    Estimation.MultiSensorInvariant.Covariance covarianceNext;
    Boolean initializedNext;
    Integer rejectionsNext;
    Boolean predictionOk;
    Boolean mocapOk;
    Boolean gpsPositionOk;
    Boolean gpsVelocityOk;
    Boolean flowOk;
    Integer rejections;
    Boolean reinitialized;
    Boolean gateFired;
    Real mocapPosition[3];
    Boolean boundsOk;
    Boolean startupOk;
    Boolean rejectionOk;
    Boolean reinitOk;
    Boolean recoveryOk;
  algorithm
    // Fix 1: the pivot threshold scales with the working precision. A
    // trailing pivot of 1e-8 is comfortably above the binary64 floor
    // (n * eps * scale ~ 4.4e-16), so the default accepts it; an
    // explicit relativeTolerance of 1e-6 must reject the same matrix.
    marginal := [1.0, 1.0; 1.0, 1.0 + 1.0e-8];
    (solution, okDefault) := LinearAlgebra.solveSPD(marginal, [1.0; 0.0]);
    (solution, okTightened) := LinearAlgebra.solveSPD(
      marginal, [1.0; 0.0], 1.0e-6);
    assert(okDefault,
      "Precision-scaled default rejected a representable trailing pivot");
    assert(not okTightened,
      "Explicit relativeTolerance floor did not tighten the pivot check");

    // Fix 2: diagonal variance limiting preserves symmetry and
    // correlation coefficients while capping out-of-envelope growth.
    limits := Estimation.MultiSensorInvariant.VarianceLimits(
      position_m2=fill(1.0e4, 3),
      velocity_m2_s2=fill(4.0e2, 3),
      attitude_rad2=fill(10.0, 3),
      gyroscopeBias_rad2_s2=fill(1.0e-2, 3),
      accelerometerBias_m2_s4=fill(1.0, 3));
    inflated := identity(15) * 0.5;
    inflated[1, 1] := 4.0e4;
    inflated[4, 4] := 1.6e3;
    inflated[1, 4] := 2.0e3;
    inflated[4, 1] := 2.0e3;
    limited := Estimation.MultiSensorInvariant.limitCovariance(
      inflated, limits);
    assert(abs(limited[1, 1] - 1.0e4) < tolerance and
      abs(limited[4, 4] - 4.0e2) < tolerance,
      "Variance limiting did not clamp the inflated diagonal");
    assert(abs(limited[1, 4] - 5.0e2) < tolerance and
      abs(limited[4, 1] - limited[1, 4]) < tolerance,
      "Variance limiting broke symmetry or the preserved correlation");
    assert(abs(limited[2, 2] - 0.5) < tolerance,
      "Variance limiting rescaled an in-envelope variance");

    // Fix 4: chi-square innovation gate. A 100 m residual against a
    // ~1 m2 innovation variance has NIS ~ 1e4, far beyond 6 per degree
    // of freedom, so the gated call must reject without touching the
    // state while the ungated call still accepts.
    initialVariances := Estimation.MultiSensorInvariant.InitialVariances(
      position_m2=fill(1.0, 3),
      velocity_m2_s2=fill(1.0, 3),
      attitude_rad2=fill(0.25, 3),
      gyroscopeBias_rad2_s2=fill(1.0e-4, 3),
      accelerometerBias_m2_s4=fill(1.0e-2, 3));
    prior := Estimation.MultiSensorInvariant.initialize(
      zeros(3), {1.0, 0.0, 0.0, 0.0}, initialVariances);
    gpsFar := Avionics.GpsSample(
      valid=true,
      fresh=true,
      positionValid=true,
      velocityValid=false,
      timestamp_s=dt,
      geodetic_deg_m=zeros(3),
      positionWorldEnu_m={100.0, 0.0, 0.0},
      velocityWorldEnu_m_s=zeros(3),
      positionCovarianceWorld_m2=identity(3) * 0.01,
      velocityCovarianceWorld_m2_s2=identity(3) * 0.01);
    (gatedState, gatedAccepted, gatedGateRejected) :=
      Estimation.MultiSensorInvariant.correctGpsPosition(
        prior, gpsFar, gate);
    (ungatedState, ungatedAccepted, ungatedGateRejected) :=
      Estimation.MultiSensorInvariant.correctGpsPosition(prior, gpsFar);
    assert(not gatedAccepted and gatedGateRejected,
      "The innovation gate did not reject a grossly inconsistent residual");
    assert(Tests.Assertions.maxAbsVector(
        gatedState.positionWorldEnu_m - prior.positionWorldEnu_m)
        < tolerance and
      Tests.Assertions.maxAbsMatrix(
        gatedState.covariance - prior.covariance) < tolerance,
      "A gate-rejected correction modified the state");
    assert(ungatedAccepted and not ungatedGateRejected,
      "Disabling the innovation gate did not restore acceptance");

    // Fix 3: persistent rejection triggers automatic re-initialization
    // through the declared init policy, with the rejection history
    // surfaced at every tick. Tick 1 initializes from mocap at the
    // origin; ticks 2-4 feed a mocap position 1 km away that the gate
    // rejects; tick 5 sees the counter at the limit and re-initializes
    // from the current mocap sample; tick 6 accepts the now-consistent
    // correction.
    position := zeros(3);
    velocity := zeros(3);
    quaternion := {1.0, 0.0, 0.0, 0.0};
    gyroscopeBias := zeros(3);
    accelerometerBias := zeros(3);
    covariance := zeros(15, 15);
    initialized := false;
    rejections := 0;
    predictionOk := false;
    mocapOk := false;
    gpsPositionOk := false;
    gpsVelocityOk := false;
    flowOk := false;
    reinitialized := false;
    gateFired := false;
    mocapPosition := zeros(3);
    positionNext := zeros(3);
    velocityNext := zeros(3);
    quaternionNext := {1.0, 0.0, 0.0, 0.0};
    gyroscopeBiasNext := zeros(3);
    accelerometerBiasNext := zeros(3);
    covarianceNext := zeros(15, 15);
    initializedNext := false;
    rejectionsNext := 0;
    boundsOk := true;
    startupOk := false;
    rejectionOk := true;
    reinitOk := false;
    recoveryOk := false;
    for tick in 1:6 loop
      mocapPosition := if tick == 1 then zeros(3)
        else {1.0e3, 0.0, 0.0};
      (positionNext,
       velocityNext,
       quaternionNext,
       gyroscopeBiasNext,
       accelerometerBiasNext,
       covarianceNext,
       initializedNext,
       predictionOk,
       mocapOk,
       gpsPositionOk,
       gpsVelocityOk,
       flowOk,
       rejectionsNext,
       reinitialized,
       gateFired) := Estimation.MultiSensorInvariant.step(
        initialized,
        position,
        velocity,
        quaternion,
        gyroscopeBias,
        accelerometerBias,
        covariance,
        false,
        true,
        true,
        tick * dt,
        zeros(3),
        {0.0, 0.0, 9.81},
        true,
        true,
        tick * dt,
        mocapPosition,
        {1.0, 0.0, 0.0, 0.0},
        identity(3) * 1.0e-4,
        identity(3) * 1.0e-4,
        false,
        false,
        false,
        false,
        0.0,
        zeros(3),
        zeros(3),
        zeros(3),
        identity(3),
        identity(3),
        false,
        false,
        0.0,
        zeros(2),
        identity(2),
        zeros(2),
        0.0,
        1.0,
        0.0,
        {0.0, 0.0, -9.81},
        dt,
        fill(1.0, 3),
        fill(1.0, 3),
        fill(0.25, 3),
        fill(1.0e-4, 3),
        fill(1.0e-2, 3),
        identity(3) * 1.0e-5,
        identity(3) * 1.0e-3,
        identity(3) * 1.0e-8,
        identity(3) * 1.0e-6,
        zeros(3),
        zeros(3),
        {1.0, 0.0, 0.0, 0.0},
        zeros(3),
        zeros(3),
        fill(1.0e4, 3),
        fill(4.0e2, 3),
        fill(10.0, 3),
        fill(1.0e-2, 3),
        fill(1.0, 3),
        gate,
        rejectionLimit,
        rejections);
      position := positionNext;
      velocity := velocityNext;
      quaternion := quaternionNext;
      gyroscopeBias := gyroscopeBiasNext;
      accelerometerBias := accelerometerBiasNext;
      covariance := covarianceNext;
      initialized := initializedNext;
      rejections := rejectionsNext;
      boundsOk := boundsOk and covariance[1, 1] <= 1.0e4 + tolerance;
      if tick == 1 then
        startupOk := initialized and rejections == 0 and not reinitialized;
      elseif tick <= 4 then
        rejectionOk := rejectionOk and not mocapOk and gateFired
          and rejections == tick - 1
          and abs(position[1]) < 1.0
          and not reinitialized;
      elseif tick == 5 then
        reinitOk := reinitialized and rejections == 0
          and abs(position[1] - 1.0e3) < tolerance
          and abs(covariance[1, 1] - 1.0) < tolerance;
      else
        // The accepted correction must also actually contract the
        // covariance (guards the accepted-branch covariance store).
        recoveryOk := mocapOk and not gateFired and rejections == 0
          and covariance[1, 1] < 0.1;
      end if;
    end for;
    assert(boundsOk,
      "Bounded covariance propagation exceeded the position limit");
    assert(startupOk,
      "Startup initialization did not leave a clean health state");
    assert(rejectionOk,
      "Gate rejection did not advance the health counter, moved the "
      + "estimate, or re-initialized below the rejection limit");
    assert(reinitOk,
      "Persistent rejection did not force re-initialization seeded from "
      + "the aiding source with the declared covariance");
    assert(recoveryOk,
      "The first post-reinitialization correction was not accepted");
    passed := true;
  end run;

  parameter Boolean passed = run();
equation
  assert(passed, "Estimator health tests did not complete");
end EstimatorHealthTests;
