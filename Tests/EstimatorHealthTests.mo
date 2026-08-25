within Tests;

model EstimatorHealthTests
  "Precision-scaled pivoting, variance limiting, gating, and auto-reinit"

  function mocapTick
    "One estimator tick against a mocap-only fixture at a fixed sample rate"
    input Boolean initialized;
    input Real position[3];
    input Real velocity[3];
    input Real quaternion[4];
    input Real gyroscopeBias[3];
    input Real accelerometerBias[3];
    input Estimation.StrapdownINS.ESKF.Covariance covariance;
    input Integer rejections;
    input Real rejectionElapsed_s;
    input Real mocapPosition[3];
    input Real timestamp_s;
    input Real dt;
    input Real innovationGate;
    input Real covarianceInflateWindow_s;
    input Real aidingDivergentWindow_s;
    input Real covarianceInflateTimeConstant_s;
    input Real aidingStaleTimeout_s;
    input Integer anchorSourcePrevious;
    output Real positionNext[3];
    output Real velocityNext[3];
    output Real quaternionNext[4];
    output Real gyroscopeBiasNext[3];
    output Real accelerometerBiasNext[3];
    output Estimation.StrapdownINS.ESKF.Covariance covarianceNext;
    output Boolean initializedNext;
    output Integer rejectionsNext;
    output Real rejectionElapsedNext_s;
    output Boolean mocapAccepted;
    output Integer recoveryStage;
    output Integer correctionOutcome;
    output Boolean estimateValid;
  protected
    Boolean predictionOk;
    Boolean gpsPositionOk;
    Boolean gpsVelocityOk;
    Boolean magnetometerOk;
    Boolean barometerOk;
    Boolean flowOk;
    Integer correctionSource;
    Real correctionNis;
    Integer mocapRejections;
    Integer gpsRejections;
    Integer flowRejections;
    Real imuOmegaHeld[3];
    Real imuAccelHeld[3];
    Real imuStampHeld;
    Boolean imuHeldFlag;
    Integer anchorSourceNext;
    Real mocapStaleNext_s;
    Real gpsStaleNext_s;
    Real flowStaleNext_s;
    Real mocapTimestampConsumedNext_s;
    Real gpsTimestampConsumedNext_s;
    Real magnetometerTimestampConsumedNext_s;
    Real barometerTimestampConsumedNext_s;
    Real opticalFlowTimestampConsumedNext_s;
  algorithm
    // Level flight: the specific force exactly cancels gravity, so an
    // accepted prediction leaves the nominal state where it was and every
    // position change below is attributable to the aiding path alone.
    (positionNext,
     velocityNext,
     quaternionNext,
     gyroscopeBiasNext,
     accelerometerBiasNext,
     covarianceNext,
     initializedNext,
     predictionOk,
     mocapAccepted,
     gpsPositionOk,
     gpsVelocityOk,
     magnetometerOk,
     barometerOk,
     flowOk,
     rejectionsNext,
     rejectionElapsedNext_s,
     recoveryStage,
     correctionOutcome,
     correctionSource,
     correctionNis,
     estimateValid,
     mocapRejections,
     gpsRejections,
     flowRejections,
     anchorSourceNext,
     mocapStaleNext_s,
     gpsStaleNext_s,
     flowStaleNext_s,
     imuOmegaHeld,
     imuAccelHeld,
     imuStampHeld,
     imuHeldFlag,
     mocapTimestampConsumedNext_s,
     gpsTimestampConsumedNext_s,
     magnetometerTimestampConsumedNext_s,
     barometerTimestampConsumedNext_s,
     opticalFlowTimestampConsumedNext_s) :=
      Estimation.StrapdownINS.ESKF.step(
      initialized,
      Estimation.StrapdownINS.ESKF.State(
        positionWorldEnu_m=position,
        velocityWorldEnu_m_s=velocity,
        quaternionWorldBody=quaternion,
        gyroscopeBiasBodyFlu_rad_s=gyroscopeBias,
        accelerometerBiasBodyFlu_m_s2=accelerometerBias,
        covariance=covariance),
      false,
      Avionics.ImuSample(
        valid=true,
        fresh=true,
        timestamp_s=timestamp_s,
        angularVelocityBodyFlu_rad_s=zeros(3),
        specificForceBodyFlu_m_s2={0.0, 0.0, 9.81},
        deltaAngleBodyFlu_rad=zeros(3),
        deltaVelocityBodyFlu_m_s={0.0, 0.0, 0.00981},
        deltaPositionBodyFlu_m={0.0, 0.0, 4.905e-6},
        deltaQuaternionBodyFlu={1.0, 0.0, 0.0, 0.0},
        gyroscopeBiasLinearizationBodyFlu_rad_s=zeros(3),
        accelerometerBiasLinearizationBodyFlu_m_s2=zeros(3),
        deltaRotationGyroscopeBiasJacobian_s=-identity(3) * 0.001,
        deltaVelocityGyroscopeBiasJacobian_m=zeros(3, 3),
        deltaVelocityAccelerometerBiasJacobian_s=-identity(3) * 0.001,
        deltaPositionGyroscopeBiasJacobian_m_s=zeros(3, 3),
        deltaPositionAccelerometerBiasJacobian_s2=
          -0.5 * identity(3) * 1.0e-6,
        integrationTime_s=0.001),
      Avionics.MocapSample(
        valid=true,
        fresh=true,
        timestamp_s=timestamp_s,
        positionWorldEnu_m=mocapPosition,
        quaternionWorldBody={1.0, 0.0, 0.0, 0.0},
        positionCovarianceWorld_m2=identity(3) * 1.0e-4,
        attitudeCovarianceBody_rad2=identity(3) * 1.0e-4),
      Avionics.GpsSample(
        valid=false,
        fresh=false,
        positionValid=false,
        velocityValid=false,
        timestamp_s=0.0,
        geodetic_deg_m=zeros(3),
        positionWorldEnu_m=zeros(3),
        velocityWorldEnu_m_s=zeros(3),
        positionCovarianceWorld_m2=identity(3),
        velocityCovarianceWorld_m2_s2=identity(3)),
      Avionics.MagnetometerSample(
        valid=false,
        fresh=false,
        timestamp_s=0.0,
        magneticFieldBodyFlu_T={18.0e-6, 4.0e-6, -47.0e-6},
        covarianceBody_T2=identity(3) * 1.0e-12),
      Avionics.BarometerSample(
        valid=false,
        fresh=false,
        timestamp_s=0.0,
        altitudeWorldEnu_m=0.0,
        variance_m2=1.0),
      Avionics.OpticalFlowSample(
        valid=false,
        fresh=false,
        timestamp_s=0.0,
        integratedLineOfSight_rad=zeros(2),
        integratedLineOfSightCovariance_rad2=identity(2),
        integratedGyroscopeBodyFlu_rad=zeros(3),
        integratedGyroscopeCovariance_rad2=identity(3),
        integrationTime_s=0.0,
        groundDistance_m=1.0,
        groundDistanceVariance_m2=0.01,
        quality=0.0),
      {0.0, 0.0, -9.81},
      dt,
      Estimation.StrapdownINS.ESKF.Tuning(
        initialState=Estimation.StrapdownINS.ESKF.NominalState(
          positionWorldEnu_m=zeros(3),
          velocityWorldEnu_m_s=zeros(3),
          quaternionWorldBody={1.0, 0.0, 0.0, 0.0},
          gyroscopeBiasBodyFlu_rad_s=zeros(3),
          accelerometerBiasBodyFlu_m_s2=zeros(3)),
        initialVariances=Estimation.StrapdownINS.InitialVariances(
          position_m2=fill(1.0, 3),
          velocity_m2_s2=fill(1.0, 3),
          attitude_rad2=fill(0.25, 3),
          gyroscopeBias_rad2_s2=fill(1.0e-4, 3),
          accelerometerBias_m2_s4=fill(1.0e-2, 3)),
        processNoise=Estimation.StrapdownINS.ProcessNoise(
          gyroscope_rad2_s=identity(3) * 1.0e-5,
          accelerometer_m2_s3=identity(3) * 1.0e-3,
          gyroscopeBias_rad2_s3=identity(3) * 1.0e-8,
          accelerometerBias_m2_s5=identity(3) * 1.0e-6),
        varianceLimits=Estimation.StrapdownINS.ESKF.VarianceLimits(
          position_m2=fill(1.0e4, 3),
          velocity_m2_s2=fill(4.0e2, 3),
          attitude_rad2=fill(10.0, 3),
          gyroscopeBias_rad2_s2=fill(1.0e-2, 3),
          accelerometerBias_m2_s4=fill(1.0, 3)),
        innovationGate=innovationGate,
        localMagneticFieldWorldEnu_T={18.0e-6, 4.0e-6, -47.0e-6},
        barometerBias_m=0.0,
        barometerBiasVariance_m2=1.0,
        maximumAidingDelay_s=0.25,
        minimumOpticalFlowQuality=0.2,
        minimumOpticalFlowGroundDistance_m=0.2,
        covarianceInflateWindow_s=covarianceInflateWindow_s,
        covarianceInflateTimeConstant_s=covarianceInflateTimeConstant_s,
        aidingDivergentWindow_s=aidingDivergentWindow_s,
        aidingStaleTimeout_s=aidingStaleTimeout_s),
      rejections,
      rejectionElapsed_s,
      0,
      0,
      0,
      anchorSourcePrevious,
      0.0,
      0.0,
      0.0,
      zeros(3),
      zeros(3),
      0.0,
      timestamp_s - dt,
      -1.0e30,
      -1.0e30,
      -1.0e30,
      -1.0e30);
  end mocapTick;

  function run
    output Boolean passed;
  protected
    constant Real tolerance = 1.0e-6;
    constant Real dt = 0.01;
    // Three rejected ticks of aiding at dt reach the inflate window.
    constant Real inflateWindow = 3.0 * dt;
    constant Real divergentWindow = 8.0 * dt;
    constant Real gate = 6.0;
    constant Real farAway[3] = {1.0e3, 0.0, 0.0};
    // Close enough to be admissible at the covariance one ramp step
    // produces, so tick 6 exercises acceptance through the ordinary
    // correction path while the ladder is active. It is NOT sized to the
    // mission envelope: the ramp reaches the envelope over seconds, not
    // in the single tick this unrolled fixture can express.
    constant Real admissibleFix[3] = {2.0, 0.0, 0.0};
    Real marginal[2, 2];
    Real solution[2, 1];
    Boolean okDefault;
    Boolean okTightened;
    Estimation.StrapdownINS.ESKF.VarianceLimits limits;
    Estimation.StrapdownINS.ESKF.Covariance inflated;
    Estimation.StrapdownINS.ESKF.Covariance limited;
    Estimation.StrapdownINS.InitialVariances initialVariances;
    Estimation.StrapdownINS.ESKF.State prior;
    Estimation.StrapdownINS.ESKF.State gatedState;
    Estimation.StrapdownINS.ESKF.State ungatedState;
    Avionics.GpsSample gpsFar;
    Boolean gatedAccepted;
    Boolean ungatedAccepted;
    Real position[3];
    Real velocity[3];
    Real quaternion[4];
    Real gyroscopeBias[3];
    Real accelerometerBias[3];
    Estimation.StrapdownINS.ESKF.Covariance covariance;
    Boolean initialized;
    Real positionNext[3];
    Real velocityNext[3];
    Real quaternionNext[4];
    Real gyroscopeBiasNext[3];
    Real accelerometerBiasNext[3];
    Estimation.StrapdownINS.ESKF.Covariance covarianceNext;
    Boolean initializedNext;
    Integer rejectionsNext;
    Integer rejections;
    Real rejectionElapsedNext_s;
    Real rejectionElapsed;
    Boolean mocapOk;
    Integer recoveryStage;
    Integer correctionOutcome;
    Boolean estimateValid;
    Boolean boundsOk;
    Boolean startupOk;
    Boolean rejectionOk;
    Boolean inflateOk;
    Boolean recoveryOk;
    Integer gatedReason;
    Integer anchorPrev;
    Real varianceBeforeInflate;
    Real varianceBeforeAccept;
    Integer ungatedReason;
    Real gatedNis;
    Real ungatedNis;
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
    limits := Estimation.StrapdownINS.ESKF.VarianceLimits(
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
    limited := Estimation.StrapdownINS.ESKF.limitCovariance(
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
    initialVariances := Estimation.StrapdownINS.InitialVariances(
      position_m2=fill(1.0, 3),
      velocity_m2_s2=fill(1.0, 3),
      attitude_rad2=fill(0.25, 3),
      gyroscopeBias_rad2_s2=fill(1.0e-4, 3),
      accelerometerBias_m2_s4=fill(1.0e-2, 3));
    prior := Estimation.StrapdownINS.ESKF.initialize(
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
    (gatedState, gatedAccepted, gatedReason, gatedNis) :=
      Estimation.StrapdownINS.ESKF.correctGpsPosition(
        prior, gpsFar, gate);
    (ungatedState, ungatedAccepted, ungatedReason, ungatedNis) :=
      Estimation.StrapdownINS.ESKF.correctGpsPosition(prior, gpsFar);
    assert(not gatedAccepted and
      gatedReason == Estimation.StrapdownINS.CorrectionRejectedGate
      and gatedNis > gate * 3.0,
      "The innovation gate did not reject a grossly inconsistent residual");
    assert(Tests.Assertions.maxAbsVector(
        gatedState.positionWorldEnu_m - prior.positionWorldEnu_m)
        < tolerance and
      Tests.Assertions.maxAbsMatrix(
        gatedState.covariance - prior.covariance) < tolerance,
      "A gate-rejected correction modified the state");
    assert(ungatedAccepted and
      ungatedReason == Estimation.StrapdownINS.CorrectionAccepted,
      "Disabling the innovation gate did not restore acceptance");

    // Fix 3: persistent rejection drives the two-stage recovery ladder,
    // which NEVER re-seeds the state from the stream it is rejecting.
    // Tick 1 initializes from mocap at the origin; ticks 2-4 feed a mocap
    // position 1 km away that the gate rejects, advancing the rejection
    // clock; tick 5 crosses the inflate window, so the covariance is
    // ramped by one bounded step with the state left exactly where it was;
    // tick 6 offers a fix consistent with that ramped covariance, which is
    // admitted through the ordinary correction path and pulls the estimate
    // over continuously.
    //
    // The 1 km fix is never adopted at any point. That is the property
    // this test exists to pin: the previous implementation re-ran the
    // declared initialization policy here and seeded position straight
    // from the rejected 1 km sample while zeroing velocity and resetting
    // attitude to the identity quaternion, all with estimate.valid still
    // true.
    //
    // The six ticks are written out rather than driven by a `for` loop.
    // A loop body has to hand each tick's outputs back to the next tick
    // through `state := stateNext` copies, and rumoca 0.9.20 drops such a
    // copy across the loop back-edge in both the folded-parameter and the
    // compiled simulation paths: every iteration then re-reads the
    // pre-loop value, the estimator re-initializes on every tick, and the
    // health assertions below fail against a correct model. OpenModelica
    // evaluates the loop form correctly. Straight-line copies are
    // compiled correctly, so the unrolled form below tests exactly the
    // intended sequence.
    position := zeros(3);
    velocity := zeros(3);
    quaternion := {1.0, 0.0, 0.0, 0.0};
    gyroscopeBias := zeros(3);
    accelerometerBias := zeros(3);
    covariance := zeros(15, 15);
    initialized := false;
    rejections := 0;
    rejectionElapsed := 0.0;
    anchorPrev := 0;
    boundsOk := true;

    // Tick 1: first sample initializes from mocap at the origin.
    (positionNext, velocityNext, quaternionNext, gyroscopeBiasNext,
     accelerometerBiasNext, covarianceNext, initializedNext, rejectionsNext,
     rejectionElapsedNext_s, mocapOk, recoveryStage, correctionOutcome,
     estimateValid) := mocapTick(
      initialized, position, velocity, quaternion, gyroscopeBias,
      accelerometerBias, covariance, rejections, rejectionElapsed, zeros(3),
      dt, dt, gate, inflateWindow, divergentWindow, 0.5, 0.5, anchorPrev);
    position := positionNext;
    velocity := velocityNext;
    quaternion := quaternionNext;
    gyroscopeBias := gyroscopeBiasNext;
    accelerometerBias := accelerometerBiasNext;
    covariance := covarianceNext;
    initialized := initializedNext;
    rejections := rejectionsNext;
    rejectionElapsed := rejectionElapsedNext_s;
    anchorPrev := Estimation.StrapdownINS.SourceMocap;
    boundsOk := boundsOk and covariance[1, 1] <= 1.0e4 + tolerance;
    startupOk := initialized and rejections == 0 and estimateValid
      and recoveryStage == Estimation.StrapdownINS.RecoveryNominal;

    // Tick 2: the first 1 km mocap sample is gate-rejected; the clock
    // advances by one interval and the estimate must not move.
    (positionNext, velocityNext, quaternionNext, gyroscopeBiasNext,
     accelerometerBiasNext, covarianceNext, initializedNext, rejectionsNext,
     rejectionElapsedNext_s, mocapOk, recoveryStage, correctionOutcome,
     estimateValid) := mocapTick(
      initialized, position, velocity, quaternion, gyroscopeBias,
      accelerometerBias, covariance, rejections, rejectionElapsed, farAway,
      2.0 * dt, dt, gate, inflateWindow, divergentWindow, 0.5, 0.5, anchorPrev);
    position := positionNext;
    velocity := velocityNext;
    quaternion := quaternionNext;
    gyroscopeBias := gyroscopeBiasNext;
    accelerometerBias := accelerometerBiasNext;
    covariance := covarianceNext;
    initialized := initializedNext;
    rejections := rejectionsNext;
    rejectionElapsed := rejectionElapsedNext_s;
    anchorPrev := Estimation.StrapdownINS.SourceMocap;
    boundsOk := boundsOk and covariance[1, 1] <= 1.0e4 + tolerance;
    rejectionOk := not mocapOk and rejections == 1
      and correctionOutcome
        == Estimation.StrapdownINS.CorrectionRejectedGate
      and abs(rejectionElapsed - dt) < tolerance
      and abs(position[1]) < 1.0
      and recoveryStage == Estimation.StrapdownINS.RecoveryNominal;

    // Tick 3: clock reaches two intervals, still below the window.
    (positionNext, velocityNext, quaternionNext, gyroscopeBiasNext,
     accelerometerBiasNext, covarianceNext, initializedNext, rejectionsNext,
     rejectionElapsedNext_s, mocapOk, recoveryStage, correctionOutcome,
     estimateValid) := mocapTick(
      initialized, position, velocity, quaternion, gyroscopeBias,
      accelerometerBias, covariance, rejections, rejectionElapsed, farAway,
      3.0 * dt, dt, gate, inflateWindow, divergentWindow, 0.5, 0.5, anchorPrev);
    position := positionNext;
    velocity := velocityNext;
    quaternion := quaternionNext;
    gyroscopeBias := gyroscopeBiasNext;
    accelerometerBias := accelerometerBiasNext;
    covariance := covarianceNext;
    initialized := initializedNext;
    rejections := rejectionsNext;
    rejectionElapsed := rejectionElapsedNext_s;
    anchorPrev := Estimation.StrapdownINS.SourceMocap;
    boundsOk := boundsOk and covariance[1, 1] <= 1.0e4 + tolerance;
    rejectionOk := rejectionOk and not mocapOk and rejections == 2
      and correctionOutcome
        == Estimation.StrapdownINS.CorrectionRejectedGate
      and abs(rejectionElapsed - 2.0 * dt) < tolerance
      and abs(position[1]) < 1.0
      and recoveryStage == Estimation.StrapdownINS.RecoveryNominal;

    // Tick 4: the clock reaches the inflate window on this tick's OUTPUT,
    // so stage 1 is armed for the next tick but has not fired yet.
    (positionNext, velocityNext, quaternionNext, gyroscopeBiasNext,
     accelerometerBiasNext, covarianceNext, initializedNext, rejectionsNext,
     rejectionElapsedNext_s, mocapOk, recoveryStage, correctionOutcome,
     estimateValid) := mocapTick(
      initialized, position, velocity, quaternion, gyroscopeBias,
      accelerometerBias, covariance, rejections, rejectionElapsed, farAway,
      4.0 * dt, dt, gate, inflateWindow, divergentWindow, 0.5, 0.5, anchorPrev);
    position := positionNext;
    velocity := velocityNext;
    quaternion := quaternionNext;
    gyroscopeBias := gyroscopeBiasNext;
    accelerometerBias := accelerometerBiasNext;
    covariance := covarianceNext;
    initialized := initializedNext;
    rejections := rejectionsNext;
    rejectionElapsed := rejectionElapsedNext_s;
    anchorPrev := Estimation.StrapdownINS.SourceMocap;
    boundsOk := boundsOk and covariance[1, 1] <= 1.0e4 + tolerance;
    varianceBeforeInflate := covariance[1, 1];
    rejectionOk := rejectionOk and not mocapOk and rejections == 3
      and correctionOutcome
        == Estimation.StrapdownINS.CorrectionRejectedGate
      and abs(rejectionElapsed - inflateWindow) < tolerance
      and abs(position[1]) < 1.0
      and recoveryStage == Estimation.StrapdownINS.RecoveryNominal;

    // Tick 5: stage 1 fires. The position covariance RAMPS by one bounded
    // step and the state is UNTOUCHED. The 1 km sample is still rejected,
    // and nothing seeds position from it.
    //
    // The bound is the point. At dt = 0.01 against a 0.5 s time constant
    // one step multiplies a variance by exp(2*dt/tau) = 1.041, so the
    // assertion below pins a few percent of growth. The form this
    // replaced set the variance straight to the 1e4 envelope in this one
    // tick, which is what turned gating into adoption; an upper bound of
    // 2.0 fails loudly against that.
    (positionNext, velocityNext, quaternionNext, gyroscopeBiasNext,
     accelerometerBiasNext, covarianceNext, initializedNext, rejectionsNext,
     rejectionElapsedNext_s, mocapOk, recoveryStage, correctionOutcome,
     estimateValid) := mocapTick(
      initialized, position, velocity, quaternion, gyroscopeBias,
      accelerometerBias, covariance, rejections, rejectionElapsed, farAway,
      5.0 * dt, dt, gate, inflateWindow, divergentWindow, 0.5, 0.5, anchorPrev);
    position := positionNext;
    velocity := velocityNext;
    quaternion := quaternionNext;
    gyroscopeBias := gyroscopeBiasNext;
    accelerometerBias := accelerometerBiasNext;
    covariance := covarianceNext;
    initialized := initializedNext;
    rejections := rejectionsNext;
    rejectionElapsed := rejectionElapsedNext_s;
    anchorPrev := Estimation.StrapdownINS.SourceMocap;
    boundsOk := boundsOk and covariance[1, 1] <= 1.0e4 + tolerance;
    inflateOk := recoveryStage
        == Estimation.StrapdownINS.RecoveryCovarianceInflated
      and not mocapOk
      and abs(position[1]) < 1.0
      and abs(velocity[1]) < tolerance
      and abs(quaternion[1] - 1.0) < tolerance
      and covariance[1, 1] > varianceBeforeInflate
      and covariance[1, 1] < 2.0
      // Attitude is deliberately NOT inflated: a filter told its
      // orientation is unknown explains position residuals as rotation.
      and covariance[7, 7] < 1.0;
    varianceBeforeAccept := covariance[1, 1];

    // Tick 6: a fix close enough to be consistent with the ramped
    // covariance is accepted through the ORDINARY correction path, while
    // the ladder is still active, and pulls the estimate over. Recovery
    // happens by admitting a plausible measurement, never by seeding from
    // an implausible one -- and the active ladder does not block an
    // ordinary correction from the anchor.
    (positionNext, velocityNext, quaternionNext, gyroscopeBiasNext,
     accelerometerBiasNext, covarianceNext, initializedNext, rejectionsNext,
     rejectionElapsedNext_s, mocapOk, recoveryStage, correctionOutcome,
     estimateValid) := mocapTick(
      initialized, position, velocity, quaternion, gyroscopeBias,
      accelerometerBias, covariance, rejections, rejectionElapsed,
      admissibleFix, 6.0 * dt, dt, gate, inflateWindow, divergentWindow,
      0.5, 0.5, anchorPrev);
    position := positionNext;
    velocity := velocityNext;
    quaternion := quaternionNext;
    gyroscopeBias := gyroscopeBiasNext;
    accelerometerBias := accelerometerBiasNext;
    covariance := covarianceNext;
    initialized := initializedNext;
    rejections := rejectionsNext;
    rejectionElapsed := rejectionElapsedNext_s;
    anchorPrev := Estimation.StrapdownINS.SourceMocap;
    boundsOk := boundsOk and covariance[1, 1] <= 1.0e4 + tolerance;
    recoveryOk := mocapOk and rejections == 0
      and correctionOutcome
        == Estimation.StrapdownINS.CorrectionAccepted
      and abs(rejectionElapsed) < tolerance
      // Moved toward the fix without teleporting onto it. The bound is
      // deliberately loose at the low end: if the attitude trust region
      // clamps this correction the state advances in installments, which
      // is correct behaviour and must not fail the test.
      and position[1] > 0.1
      and position[1] < 2.5
      // An accepted correction must CONTRACT the position variance.
      // Stated against the pre-acceptance value rather than an absolute
      // bound, so it holds whether or not the trust region engaged the
      // partial-update covariance blend.
      and covariance[1, 1] < varianceBeforeAccept
      and estimateValid;

    assert(boundsOk,
      "Bounded covariance propagation exceeded the position limit");
    assert(startupOk,
      "Startup initialization did not leave a clean health state");
    assert(rejectionOk,
      "Gate rejection did not advance the rejection clock by one sample "
      + "interval per tick, moved the estimate, or fired the recovery "
      + "ladder before the inflate window elapsed");
    assert(inflateOk,
      "Stage 1 did not ramp position covariance by one bounded step "
      + "while leaving position, velocity, attitude and the attitude "
      + "covariance untouched");
    assert(recoveryOk,
      "A fix consistent with the ramped covariance was not admitted, or "
      + "did not contract the position variance");
    passed := true;
  end run;

  parameter Boolean passed = run();
equation
  assert(passed, "Estimator health tests did not complete");
end EstimatorHealthTests;
