within Tests;

model SensorCorrectionTests
  "Barometer, terrain, magnetometer, and optical-flow structure checks"
  function run
    output Boolean passed;
  protected
    Real mocapAge_s;
    Real mocapVelocity_m_s[3];
    Estimation.StrapdownINS.ESKF.State agedPrior;
    Estimation.StrapdownINS.ESKF.State alignedCorrection;
    Estimation.StrapdownINS.ESKF.State unalignedCorrection;
    Avionics.MocapSample agedMocap;
    Boolean mocapAccepted;
    Integer mocapReason;
    Real nis;
    Real alignedShift_m;
    Real unalignedShift_m;
    Real covariance[15, 15];
    Boolean initialized;
    Real bias_m;
    Real biasVariance_m2;
    Boolean biasAccepted;
    Real terrainAltitude_m;
    Real terrainVariance_m2;
    Boolean terrainAccepted;
    Estimation.StrapdownINS.ESKF.State predicted;
    Estimation.StrapdownINS.ESKF.State correctedNear;
    Estimation.StrapdownINS.ESKF.State correctedFar;
    Boolean acceptedNear;
    Boolean acceptedFar;
    Integer reasonNear;
    Integer reasonFar;
    Real nisNear;
    Real nisFar;
    Avionics.OpticalFlowSample flowNear;
    Avionics.OpticalFlowSample flowFar;
    Real magneticFieldWorldEnu_T[3];
    Real headingQuaternion[4];
    Real perturbedQuaternion[4];
    Real perturbation[3];
    Real headingPlus;
    Real headingMinus;
    Real headingNominal;
    Real headingVariance_rad2;
    Boolean headingUsable;
    Real yawSensitivity[3];
    Real tiltSensitivity[3];
    Real scratchYaw[3];
    Real scratchTilt[3];
    Real scratchVariance;
    Boolean scratchUsable;
    Real numericSensitivity[3];
    Real tiltTransferMagnitude;
  algorithm
    covariance := identity(15) * 0.01;
    (initialized, bias_m, biasVariance_m2, biasAccepted) :=
      Estimation.StrapdownINS.updateBarometerBias(
        false, 0.0, 4.0, {0.0, 0.0, 2.0},
        {1.0, 0.0, 0.0, 0.0}, covariance,
        Avionics.BarometerSample(
          valid=true, fresh=true, timestamp_s=0.0,
          altitudeWorldEnu_m=2.4, variance_m2=0.25),
        0.01, 0.0, 4.0, 1.0e-4);
    assert(initialized and biasAccepted and bias_m > 0.35 and bias_m < 0.4,
      "Barometer bias filter did not learn the pressure-altitude offset");
    assert(biasVariance_m2 > 0.0 and biasVariance_m2 < 4.0,
      "Barometer bias variance did not contract after a valid observation");

    (initialized, terrainAltitude_m, terrainVariance_m2, terrainAccepted) :=
      Estimation.StrapdownINS.updateTerrainAltitude(
        false, 0.0, 4.0, {0.0, 0.0, 2.0},
        {1.0, 0.0, 0.0, 0.0}, covariance,
        Avionics.OpticalFlowSample(
          valid=true, fresh=true, timestamp_s=0.0,
          integratedLineOfSight_rad=zeros(2),
          integratedLineOfSightCovariance_rad2=identity(2) * 1.0e-6,
          integratedGyroscopeBodyFlu_rad=zeros(3),
          integratedGyroscopeCovariance_rad2=identity(3) * 1.0e-8,
          integrationTime_s=0.01, groundDistance_m=3.0,
          groundDistanceVariance_m2=0.01, quality=1.0),
        0.01, 0.0, 4.0, 1.0e-3, 0.5, 0.2);
    assert(initialized and terrainAccepted
        and terrainAltitude_m < -0.95 and terrainAltitude_m > -1.0,
      "Vertical range did not estimate terrain altitude as z minus range");
    assert(terrainVariance_m2 > 0.0 and terrainVariance_m2 < 4.0,
      "Terrain variance did not contract after a valid range observation");

    predicted := Estimation.StrapdownINS.ESKF.State(
      positionWorldEnu_m={0.0, 0.0, 2.0},
      velocityWorldEnu_m_s={1.0, 0.0, 0.0},
      quaternionWorldBody={1.0, 0.0, 0.0, 0.0},
      gyroscopeBiasBodyFlu_rad_s=zeros(3),
      accelerometerBiasBodyFlu_m_s2=zeros(3),
      covariance=covariance);
    flowNear := Avionics.OpticalFlowSample(
      valid=true, fresh=true, timestamp_s=0.0,
      integratedLineOfSight_rad={0.0, 0.005},
      integratedLineOfSightCovariance_rad2=identity(2) * 1.0e-6,
      integratedGyroscopeBodyFlu_rad=zeros(3),
      integratedGyroscopeCovariance_rad2=identity(3) * 1.0e-8,
      integrationTime_s=0.01, groundDistance_m=2.0,
      groundDistanceVariance_m2=0.01, quality=1.0);
    flowFar := Avionics.OpticalFlowSample(
      valid=flowNear.valid, fresh=flowNear.fresh,
      timestamp_s=flowNear.timestamp_s,
      integratedLineOfSight_rad=flowNear.integratedLineOfSight_rad,
      integratedLineOfSightCovariance_rad2=
        flowNear.integratedLineOfSightCovariance_rad2,
      integratedGyroscopeBodyFlu_rad=
        flowNear.integratedGyroscopeBodyFlu_rad,
      integratedGyroscopeCovariance_rad2=
        flowNear.integratedGyroscopeCovariance_rad2,
      integrationTime_s=flowNear.integrationTime_s,
      groundDistance_m=50.0,
      groundDistanceVariance_m2=flowNear.groundDistanceVariance_m2,
      quality=flowNear.quality);
    (correctedNear, acceptedNear, reasonNear, nisNear) :=
      Estimation.StrapdownINS.ESKF.correctOpticalFlow(
        predicted, flowNear, 6.0);
    (correctedFar, acceptedFar, reasonFar, nisFar) :=
      Estimation.StrapdownINS.ESKF.correctOpticalFlow(
        predicted, flowFar, 6.0);
    assert(acceptedNear and not acceptedFar
        and reasonFar == Estimation.StrapdownINS.CorrectionRejectedGate
        and nisFar > nisNear,
      "Optical-flow correction did not use co-timed range to form body velocity");
    // THE MAGNETOMETER HEADING JACOBIAN IS NOT YAW-ONLY.
    //
    // magnetometerYawObservation levels the measured field with the
    // ESTIMATE's roll and pitch, so the heading it reports moves when the
    // attitude error has a tilt component: by about tan(inclination) times
    // that component, which is roughly 2.4 at the RDD2 test site. Pin the
    // analytic sensitivity against a central difference of the observation
    // itself, so a future yaw-only Jacobian cannot silently return and make
    // the heading covariance optimistic again.
    magneticFieldWorldEnu_T := {-1.738e-6, 2.0024e-5, -4.7907e-5};
    headingQuaternion :=
      LieGroups.SO3.EulerB321.to_Quat({0.3, 0.15, -0.1});
    perturbation := zeros(3);
    for axis in 1:3 loop
      perturbation[axis] := 1.0e-6;
      perturbedQuaternion := LieGroups.SO3.Quat.product(
        headingQuaternion, LieGroups.SO3.Quat.exp_map(perturbation));
      (headingPlus, scratchVariance, scratchUsable, scratchYaw,
       scratchTilt) :=
        Estimation.StrapdownINS.magnetometerYawObservation(
          headingQuaternion,
          transpose(LieGroups.SO3.Quat.to_DCM(perturbedQuaternion))
            * magneticFieldWorldEnu_T,
          identity(3) * 1.0e-13, magneticFieldWorldEnu_T);
      perturbation[axis] := -1.0e-6;
      perturbedQuaternion := LieGroups.SO3.Quat.product(
        headingQuaternion, LieGroups.SO3.Quat.exp_map(perturbation));
      (headingMinus, scratchVariance, scratchUsable, scratchYaw,
       scratchTilt) :=
        Estimation.StrapdownINS.magnetometerYawObservation(
          headingQuaternion,
          transpose(LieGroups.SO3.Quat.to_DCM(perturbedQuaternion))
            * magneticFieldWorldEnu_T,
          identity(3) * 1.0e-13, magneticFieldWorldEnu_T);
      numericSensitivity[axis] :=
        (headingPlus - headingMinus) / 2.0e-6;
      perturbation[axis] := 0.0;
    end for;
    (headingNominal, headingVariance_rad2, headingUsable, yawSensitivity,
     tiltSensitivity) :=
      Estimation.StrapdownINS.magnetometerYawObservation(
        headingQuaternion,
        transpose(LieGroups.SO3.Quat.to_DCM(headingQuaternion))
          * magneticFieldWorldEnu_T,
        identity(3) * 1.0e-13, magneticFieldWorldEnu_T);
    assert(headingUsable and abs(headingNominal - 0.3) < 1.0e-9,
      "Tilt-compensated heading did not reproduce the estimate's own yaw");
    for axis in 1:3 loop
      assert(abs(yawSensitivity[axis] + tiltSensitivity[axis]
          - numericSensitivity[axis]) < 1.0e-4,
        "Magnetometer heading Jacobian disagrees with its own observation");
    end for;
    tiltTransferMagnitude := sqrt(tiltSensitivity * tiltSensitivity);
    assert(tiltTransferMagnitude > 1.5,
      "Levelling transfer of the heading observation read as negligible");

    // ---- motion capture is aged, and the alignment stanza does the work ----
    // THE POINT OF THIS ARM. Until the plant grew a mocap transport model the
    // stanza in correctMocap ran with age zero on every call, which is the
    // identity, so it was present and inert. With a 20 ms transport it does
    // real work, and this measures how much.
    //
    // The scenario is built so the right answer is KNOWN rather than merely
    // plausible. A vehicle moves at a constant 5 m/s and the rig reports the
    // pose it truly held 20 ms ago, so the measurement is exactly
    // p_now - v * age. A correction that aligns for the age finds no
    // disagreement and must leave the state where it is; a correction that
    // ignores the age sees a spurious innovation of exactly v * age, which is
    // 0.100 m, and with a mocap covariance far tighter than the prior the gain
    // is 1/(1 + 1e-4) and it drags the state essentially all of that way.
    //
    // So the two calls differ by a quantity this test can state in advance,
    // and the aged one is the one that leaves the state alone.
    mocapAge_s := 0.02;
    mocapVelocity_m_s := {5.0, 0.0, 0.0};
    agedPrior := Estimation.StrapdownINS.ESKF.State(
      positionWorldEnu_m={10.0, 0.0, 2.0},
      velocityWorldEnu_m_s=mocapVelocity_m_s,
      quaternionWorldBody={1.0, 0.0, 0.0, 0.0},
      gyroscopeBiasBodyFlu_rad_s=zeros(3),
      accelerometerBiasBodyFlu_m_s2=zeros(3),
      covariance=identity(15));
    agedMocap := Avionics.MocapSample(
      valid=true,
      fresh=true,
      timestamp_s=0.0,
      positionWorldEnu_m=agedPrior.positionWorldEnu_m
        - mocapVelocity_m_s * mocapAge_s,
      quaternionWorldBody={1.0, 0.0, 0.0, 0.0},
      positionCovarianceWorld_m2=identity(3) * 1.0e-4,
      attitudeCovarianceBody_rad2=identity(3) * 1.0e-4);
    // Aligned for the age: the measurement agrees with where the vehicle was.
    (alignedCorrection, mocapAccepted, mocapReason, nis) :=
      Estimation.StrapdownINS.ESKF.correctMocap(
        agedPrior, agedMocap, 0.0, mocapAge_s,
        zeros(3), {0.0, 0.0, 9.81}, {0.0, 0.0, -9.81}, 0.25);
    assert(mocapAccepted,
      "An aged motion-capture pose inside the delay bound was refused");
    // Not aligned: the same measurement read as though it described now.
    (unalignedCorrection, mocapAccepted, mocapReason, nis) :=
      Estimation.StrapdownINS.ESKF.correctMocap(
        agedPrior, agedMocap, 0.0, 0.0,
        zeros(3), {0.0, 0.0, 9.81}, {0.0, 0.0, -9.81}, 0.25);
    assert(mocapAccepted,
      "The unaligned control call was refused, so the two are not comparable");
    alignedShift_m := sqrt(
      (alignedCorrection.positionWorldEnu_m - agedPrior.positionWorldEnu_m)
      * (alignedCorrection.positionWorldEnu_m - agedPrior.positionWorldEnu_m));
    unalignedShift_m := sqrt(
      (unalignedCorrection.positionWorldEnu_m - agedPrior.positionWorldEnu_m)
      * (unalignedCorrection.positionWorldEnu_m
        - agedPrior.positionWorldEnu_m));
    // The aligned correction leaves the state alone. Measured below 1e-12 m,
    // and it is worth saying why it is that small rather than merely small:
    // this scenario is constant velocity with the specific force exactly
    // cancelling gravity, so the inverse mixed flow is EXACT for it and no
    // cubic-Taylor truncation appears. That makes the limit below a
    // floating-point limit rather than an error budget, and it means this arm
    // proves the stanza recovers the right epoch without also probing the
    // transport's truncation, which is measured under acceleration by the
    // horizon residual work instead.
    assert(alignedShift_m < 1.0e-9,
      "Aligning a motion-capture pose for its own 20 ms age still moved the
       state, so the retrodiction is not recovering the epoch the measurement
       describes");
    // The unaligned correction moves the state by the spurious innovation,
    // v * age = 0.100 m, scaled by a gain of 1/(1 + 1e-4). Measured between
    // 0.09998 and 0.10000 m, which is that product to five figures.
    assert(unalignedShift_m > 0.0999 and unalignedShift_m < 0.1001,
      "Ignoring the age did not produce the v * age displacement this scenario
       was built to produce, so the two calls are not measuring the stanza");
    // THE STANZA IS NOT INERT. Stated as a ratio so a future change that
    // quietly reduced the alignment to the identity fails here rather than
    // passing on two limits that both happen to hold.
    assert(unalignedShift_m > 100.0 * alignedShift_m,
      "The aligned and unaligned motion-capture corrections agree too closely,
       so the age-alignment stanza is doing no work");
    // A pose older than the declared bound is refused BY NAME rather than
    // transported to meet the state.
    (alignedCorrection, mocapAccepted, mocapReason, nis) :=
      Estimation.StrapdownINS.ESKF.correctMocap(
        agedPrior, agedMocap, 0.0, 0.30,
        zeros(3), {0.0, 0.0, 9.81}, {0.0, 0.0, -9.81}, 0.25);
    assert(not mocapAccepted
        and mocapReason
          == Estimation.StrapdownINS.CorrectionRejectedTimestamp,
      "A motion-capture pose older than maximumAidingDelay_s was fused instead
       of being refused on its timestamp");

    passed := true;
  end run;

  parameter Boolean passed = run();
equation
  assert(passed, "Sensor correction assertions did not complete");
end SensorCorrectionTests;
