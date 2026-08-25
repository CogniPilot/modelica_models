within Tests;

model EstimatorHardeningTests
  "Affirmative acceptance, per-channel NaN rejection, and window semantics"

  function nanState
    "A well-formed prior state to offer bad measurements against"
    output Estimation.StrapdownINS.ESKF.State prior;
  protected
    Estimation.StrapdownINS.InitialVariances variances;
  algorithm
    variances := Estimation.StrapdownINS.InitialVariances(
      position_m2=fill(1.0, 3),
      velocity_m2_s2=fill(1.0, 3),
      attitude_rad2=fill(0.25, 3),
      gyroscopeBias_rad2_s2=fill(1.0e-4, 3),
      accelerometerBias_m2_s4=fill(1.0e-2, 3));
    prior := Estimation.StrapdownINS.ESKF.initialize(
      zeros(3), {1.0, 0.0, 0.0, 0.0}, variances);
  end nanState;

  function processNoiseFixture
    output Estimation.StrapdownINS.ProcessNoise noise;
  algorithm
    noise := Estimation.StrapdownINS.ProcessNoise(
      gyroscope_rad2_s=identity(3) * 1.0e-5,
      accelerometer_m2_s3=identity(3) * 1.0e-3,
      gyroscopeBias_rad2_s3=identity(3) * 1.0e-8,
      accelerometerBias_m2_s5=identity(3) * 1.0e-6);
  end processNoiseFixture;

  function run
    output Boolean passed;
  protected
    constant Real tolerance = 1.0e-9;
    constant Real gate = 6.0;
    // NON-FINITE STAND-INS.
    //
    // These tests drive the affirmative guards with magnitudes past
    // Estimation.StrapdownINS.ESKF.FiniteMagnitudeLimit rather than
    // with a true NaN, because a genuine NaN here would have to be built
    // as 0.0/0.0 and this suite is evaluated as a folded parameter
    // binding, which rumoca rejects outright (ED019, "cannot be
    // evaluated: division by zero") before any assertion runs.
    //
    // The branch exercised is identical. The guard is the single
    // comparison `abs(x) < FiniteMagnitudeLimit`, which is false for a
    // NaN, for an infinity, and for 1e31 alike -- there is no separate
    // NaN path to miss. True NaN and Inf behaviour on every channel is
    // pinned end to end against the generated flight C by the witness
    // harness kept with this change, which drives real NaN bit patterns
    // through the compiled estimator.
    constant Real notFinite = 1.0e31;
    // Negative definite, so the Cholesky pivot guard rejects it the same
    // way a NaN pivot now does.
    constant Real badVariance = -1.0e3;
    Estimation.StrapdownINS.ESKF.State prior;
    Estimation.StrapdownINS.ESKF.State corrected;
    Avionics.GpsSample gpsSample;
    Avionics.MocapSample mocapSample;
    Avionics.OpticalFlowSample flowSample;
    Boolean accepted;
    Integer reason;
    Estimation.StrapdownINS.ESKF.VarianceLimits limits;
    Estimation.StrapdownINS.ESKF.Covariance inflated;
    Real nis;
    Estimation.StrapdownINS.ESKF.Covariance held;
    Real degenerate[2, 2];
    Real solution[2, 1];
    Boolean factorized;
    Boolean gpsPositionOk;
    Boolean gpsVelocityOk;
    Boolean gpsCovarianceOk;
    Boolean mocapOk;
    Boolean flowOk;
    Boolean flowCovarianceOk;
    Boolean ungatedOk;
    Boolean solveOk;
    Boolean inflateOk;
    Boolean holdOk;
    Boolean guardOk;
  algorithm
    prior := nanState();

    // ---- Affirmative acceptance, per aiding channel ----
    // Each of these was ACCEPTED before the fix, because acceptance was
    // computed as "whatever the rejection predicate did not catch" and
    // every comparison against a NaN is false. The corrected state had to
    // be checked for finiteness too: a rejected correction must leave the
    // prior exactly alone, not merely fail to set a flag.

    // GPS position channel.
    gpsSample := Avionics.GpsSample(
      valid=true, fresh=true, positionValid=true, velocityValid=false,
      timestamp_s=0.0, geodetic_deg_m=zeros(3),
      positionWorldEnu_m={notFinite, 0.0, 0.0},
      velocityWorldEnu_m_s=zeros(3),
      positionCovarianceWorld_m2=identity(3) * 0.25,
      velocityCovarianceWorld_m2_s2=identity(3) * 0.01);
    (corrected, accepted, reason, nis) :=
      Estimation.StrapdownINS.ESKF.correctGpsPosition(
        prior, gpsSample, gate);
    gpsPositionOk := not accepted
      and reason
        == Estimation.StrapdownINS.CorrectionRejectedNotFinite
      and Tests.Assertions.isFiniteVector(corrected.positionWorldEnu_m)
      and Tests.Assertions.maxAbsVector(
        corrected.positionWorldEnu_m - prior.positionWorldEnu_m) < tolerance;

    // GPS velocity channel.
    gpsSample := Avionics.GpsSample(
      valid=true, fresh=true, positionValid=false, velocityValid=true,
      timestamp_s=0.0, geodetic_deg_m=zeros(3),
      positionWorldEnu_m=zeros(3),
      velocityWorldEnu_m_s={notFinite, 0.0, 0.0},
      positionCovarianceWorld_m2=identity(3) * 0.25,
      velocityCovarianceWorld_m2_s2=identity(3) * 0.01);
    (corrected, accepted, reason, nis) :=
      Estimation.StrapdownINS.ESKF.correctGpsVelocity(
        prior, gpsSample, gate);
    gpsVelocityOk := not accepted
      and reason
        == Estimation.StrapdownINS.CorrectionRejectedNotFinite
      and Tests.Assertions.isFiniteVector(corrected.velocityWorldEnu_m_s);

    // GPS covariance channel: the bad value reaches the Cholesky rather
    // than the residual, so it must be caught by the pivot guard in
    // solveSPD and reported as a factorization failure -- the same
    // branch a NaN pivot now takes.
    gpsSample := Avionics.GpsSample(
      valid=true, fresh=true, positionValid=true, velocityValid=false,
      timestamp_s=0.0, geodetic_deg_m=zeros(3),
      positionWorldEnu_m={1.0, 0.0, 0.0},
      velocityWorldEnu_m_s=zeros(3),
      positionCovarianceWorld_m2=identity(3) * badVariance,
      velocityCovarianceWorld_m2_s2=identity(3) * 0.01);
    (corrected, accepted, reason, nis) :=
      Estimation.StrapdownINS.ESKF.correctGpsPosition(
        prior, gpsSample, gate);
    // Caught by the affirmative check on the sensor-reported R, which runs
    // before the Cholesky, so the named cause is CovarianceUnusable rather
    // than a factorization failure.
    gpsCovarianceOk := not accepted
      and reason
        == Estimation.StrapdownINS.CorrectionRejectedCovarianceUnusable
      and Tests.Assertions.isFiniteMatrix(corrected.covariance);

    // Mocap channel.
    mocapSample := Avionics.MocapSample(
      valid=true, fresh=true, timestamp_s=0.0,
      positionWorldEnu_m={notFinite, 0.0, 0.0},
      quaternionWorldBody={1.0, 0.0, 0.0, 0.0},
      positionCovarianceWorld_m2=identity(3) * 1.0e-4,
      attitudeCovarianceBody_rad2=identity(3) * 1.0e-4);
    (corrected, accepted, reason, nis) :=
      Estimation.StrapdownINS.ESKF.correctMocap(prior, mocapSample, gate);
    mocapOk := not accepted
      and reason
        == Estimation.StrapdownINS.CorrectionRejectedNotFinite
      and Tests.Assertions.isFiniteVector(corrected.quaternionWorldBody);

    // Optical-flow channel, measurement and covariance.
    flowSample := Avionics.OpticalFlowSample(
      valid=true, fresh=true, timestamp_s=0.0,
      integratedLineOfSight_rad={notFinite, 0.0},
      integratedLineOfSightCovariance_rad2=identity(2) * 0.01,
      integratedGyroscopeBodyFlu_rad=zeros(3),
      integratedGyroscopeCovariance_rad2=identity(3) * 1.0e-8,
      integrationTime_s=0.01,
      groundDistance_m=1.0, groundDistanceVariance_m2=0.01, quality=1.0);
    (corrected, accepted, reason, nis) :=
      Estimation.StrapdownINS.ESKF.correctOpticalFlow(
        prior, flowSample, gate);
    flowOk := not accepted
      and reason
        == Estimation.StrapdownINS.CorrectionRejectedNotFinite
      and Tests.Assertions.isFiniteVector(corrected.velocityWorldEnu_m_s);

    flowSample := Avionics.OpticalFlowSample(
      valid=true, fresh=true, timestamp_s=0.0,
      integratedLineOfSight_rad={0.0, 0.01},
      integratedLineOfSightCovariance_rad2=identity(2) * badVariance,
      integratedGyroscopeBodyFlu_rad=zeros(3),
      integratedGyroscopeCovariance_rad2=identity(3) * 1.0e-8,
      integrationTime_s=0.01,
      groundDistance_m=1.0, groundDistanceVariance_m2=0.01, quality=1.0);
    (corrected, accepted, reason, nis) :=
      Estimation.StrapdownINS.ESKF.correctOpticalFlow(
        prior, flowSample, gate);
    flowCovarianceOk := not accepted
      and reason
        == Estimation.StrapdownINS.CorrectionRejectedCovarianceUnusable
      and Tests.Assertions.isFiniteMatrix(corrected.covariance);

    // Disabling the gate must not re-open a NaN entry path. This is the
    // case the "NIS >= 0" term exists for: with innovationGate <= 0 the
    // chi-square comparison is skipped entirely, so finiteness has to be
    // established independently of it.
    gpsSample := Avionics.GpsSample(
      valid=true, fresh=true, positionValid=true, velocityValid=false,
      timestamp_s=0.0, geodetic_deg_m=zeros(3),
      positionWorldEnu_m={notFinite, 0.0, 0.0},
      velocityWorldEnu_m_s=zeros(3),
      positionCovarianceWorld_m2=identity(3) * 0.25,
      velocityCovarianceWorld_m2_s2=identity(3) * 0.01);
    (corrected, accepted, reason, nis) :=
      Estimation.StrapdownINS.ESKF.correctGpsPosition(
        prior, gpsSample, 0.0);
    ungatedOk := not accepted
      and Tests.Assertions.isFiniteVector(corrected.positionWorldEnu_m);

    // ---- solveSPD rejects a non-finite pivot affirmatively ----
    degenerate := [badVariance, 0.0; 0.0, 1.0];
    (solution, factorized) :=
      LinearAlgebra.solveSPD(degenerate, [1.0; 0.0]);
    solveOk := not factorized;

    // ---- Envelope inflation moves no state and spares attitude ----
    limits := Estimation.StrapdownINS.ESKF.VarianceLimits(
      position_m2=fill(1.0e4, 3),
      velocity_m2_s2=fill(4.0e2, 3),
      attitude_rad2=fill(10.0, 3),
      gyroscopeBias_rad2_s2=fill(1.0e-2, 3),
      accelerometerBias_m2_s4=fill(1.0, 3));
    // One ramp step with dt = tau grows each variance by exp(2*dt/tau) = e^2,
    // so a prior variance of 1.0 lands on 7.389 -- far below the 1e4 bound.
    // This pins the RAMP: the jump-to-envelope form this replaced would put
    // 1e4 here, and that is exactly the behaviour that made stage 1 worse
    // than taking no action at all.
    inflated := Estimation.StrapdownINS.ESKF.inflateCovariance(
      prior.covariance, limits, 0.1, 0.1);
    inflateOk := abs(inflated[1, 1] - exp(2.0)) < 1.0e-4
      and abs(inflated[4, 4] - exp(2.0)) < 1.0e-4
      and inflated[1, 1] < 1.0e4
      // Attitude and both bias blocks keep their prior variance.
      and abs(inflated[7, 7] - prior.covariance[7, 7]) < tolerance
      and abs(inflated[10, 10] - prior.covariance[10, 10]) < tolerance
      and abs(inflated[13, 13] - prior.covariance[13, 13]) < tolerance
      // Symmetry survives the congruence.
      and Tests.Assertions.maxAbsMatrix(
        inflated - transpose(inflated)) < tolerance;

    // A grossly inconsistent but FINITE residual must still be rejected
    // by the gate rather than by the finiteness guard, so the two
    // mechanisms stay distinguishable in the health record.
    gpsSample := Avionics.GpsSample(
      valid=true, fresh=true, positionValid=true, velocityValid=false,
      timestamp_s=0.0, geodetic_deg_m=zeros(3),
      positionWorldEnu_m={1.0e3, 0.0, 0.0},
      velocityWorldEnu_m_s=zeros(3),
      positionCovarianceWorld_m2=identity(3) * 0.25,
      velocityCovarianceWorld_m2_s2=identity(3) * 0.01);
    (corrected, accepted, reason, nis) :=
      Estimation.StrapdownINS.ESKF.correctGpsPosition(
        prior, gpsSample, gate);
    nis := if reason == Estimation.StrapdownINS.CorrectionRejectedGate
      and nis > gate * 3.0 then 1.0 else 0.0;

    // ---- holdCovariance grows the POSITION block over a blind tick ----
    // The version this replaced added only the process-noise integral, whose
    // position term is O(dt^3) and lands below one ulp of binary32 against a
    // realistic position variance -- so the position block did not grow at
    // all in flight code, while the docstring said it did. The growth that
    // matters is P_vv * dt^2, carried by the dp/dv = I block of the
    // transition, which is kinematics and holds whatever the IMU is doing.
    held := Estimation.StrapdownINS.ESKF.holdCovariance(
      prior.covariance, 0.01, processNoiseFixture());
    holdOk := held[1, 1] > prior.covariance[1, 1]
      and held[4, 4] > prior.covariance[4, 4]
      // The dominant position term is P_vv * dt^2; assert it is present at
      // that order rather than merely non-zero, so an O(dt^3)-only
      // regression cannot pass.
      and held[1, 1] - prior.covariance[1, 1]
        > 0.5 * prior.covariance[4, 4] * 0.01 * 0.01
      and Tests.Assertions.isFiniteMatrix(held);

    // ---- the publication guard ----
    // estimate.valid must be false for a non-finite published state and
    // true for a finite one. This is the only guard covering the prediction
    // side, where no innovation gate exists.
    guardOk := Estimation.StrapdownINS.ESKF.nominalStateFinite(
        zeros(3), zeros(3), {1.0, 0.0, 0.0, 0.0})
      and not Estimation.StrapdownINS.ESKF.nominalStateFinite(
        {notFinite, 0.0, 0.0}, zeros(3), {1.0, 0.0, 0.0, 0.0})
      and not Estimation.StrapdownINS.ESKF.nominalStateFinite(
        zeros(3), {0.0, notFinite, 0.0}, {1.0, 0.0, 0.0, 0.0})
      and not Estimation.StrapdownINS.ESKF.nominalStateFinite(
        zeros(3), zeros(3), {1.0, 0.0, notFinite, 0.0});

    assert(gpsPositionOk,
      "A non-finite GPS position was not rejected as such, or moved the state");
    assert(gpsVelocityOk,
      "An out-of-range GPS velocity was not rejected as not finite");
    assert(gpsCovarianceOk,
      "An unusable GPS position covariance was not rejected as such");
    assert(mocapOk, "A non-finite mocap position was not rejected");
    assert(flowOk, "A non-finite optical-flow velocity was not rejected");
    assert(flowCovarianceOk,
      "An unusable optical-flow covariance was not rejected");
    assert(ungatedOk,
      "Disabling the innovation gate re-opened the non-finite entry path");
    assert(solveOk,
      "solveSPD reported success on a matrix with a bad pivot");
    assert(inflateOk,
      "Covariance inflation did not ramp position and velocity by one "
      + "bounded step while leaving attitude, biases and symmetry alone");
    assert(holdOk,
      "holdCovariance did not grow the position block by the velocity "
      + "term over a blind interval");
    assert(guardOk,
      "The publication guard did not reject a non-finite published state");
    assert(nis > 0.5,
      "A finite but grossly inconsistent residual was not attributed to "
      + "the innovation gate");
    passed := true;
  end run;

  // Asserted INSIDE run(), which OMC constant-folds at translation time.
  //
  // That fold is deliberate and is what keeps this suite cheap. Binding the
  // result into an equation instead stops the fold and emits the whole body
  // into the generated functions unit: measured, that took it from 511 KB to
  // 34.7 MB, with a compile time to match. The named diagnostic survives the
  // fold -- a deliberately broken property is reported by OMC as its own
  // assert message and source line at initialization -- so the equation form
  // bought nothing the failure mode needed and cost roughly seventy times the
  // build.
  parameter Boolean passed = run();
equation
  assert(passed, "Estimator hardening tests did not complete");
end EstimatorHardeningTests;
