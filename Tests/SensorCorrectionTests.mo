within Tests;

model SensorCorrectionTests
  "Barometer, terrain, magnetometer, and optical-flow structure checks"
  function run
    output Boolean passed;
  protected
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

    passed := true;
  end run;

  parameter Boolean passed = run();
equation
  assert(passed, "Sensor correction assertions did not complete");
end SensorCorrectionTests;
