within Tests;

model MultiSensorInvariantTests
  "Mixed SE_2(3) prediction, covariance, correction, and interface checks"
  function run
    output Boolean passed;
  protected
    constant Real tolerance = 1.0e-8;
    Estimation.MultiSensorInvariant.InitialVariances initialVariances;
    Estimation.MultiSensorInvariant.ProcessNoise processNoise;
    Estimation.MultiSensorInvariant.State initialized;
    Estimation.MultiSensorInvariant.State hoverPrediction;
    Estimation.MultiSensorInvariant.State gpsPositionCorrection;
    Estimation.MultiSensorInvariant.State gpsVelocityCorrection;
    Estimation.MultiSensorInvariant.State mocapCorrection;
    Estimation.MultiSensorInvariant.State flowPrior;
    Estimation.MultiSensorInvariant.State flowCorrection;
    Estimation.MultiSensorInvariant.NominalState transitionNominal;
    Estimation.MultiSensorInvariant.NominalState transitionPerturbed;
    Estimation.MultiSensorInvariant.NominalState transitionPredicted;
    Estimation.MultiSensorInvariant.NominalState perturbedPredicted;
    Avionics.GpsSample gps;
    Avionics.MocapSample mocap;
    Avionics.OpticalFlowSample opticalFlow;
    Avionics.ImuSample imu;
    Boolean navigationValid;
    Real navigationTimestamp_s;
    Real navigationPositionWorldEnu_m[3];
    Real navigationVelocityWorldEnu_m_s[3];
    Real navigationAccelerationWorldEnu_m_s2[3];
    Real navigationQuaternionWorldBody[4];
    Real navigationRotationWorldBody[3, 3];
    Real navigationEulerRpy_rad[3];
    Real navigationAngularVelocityBodyFlu_rad_s[3];
    Real navigationAngularVelocityWorldEnu_rad_s[3];
    Real A[15, 15];
    Real transition[15, 15];
    Real hoverTransition[15, 15];
    Real numericalTransition[15, 15];
    Real perturbation[15];
    Real nominalExtendedPose[10];
    Real perturbedExtendedPose[10];
    Real invariantError[10];
    Real invariantErrorTangent[9];
    Real measuredAngularVelocity[3];
    Real measuredSpecificForce[3];
    constant Real finiteDifferenceStep = 1.0e-6;
    constant Real transitionDt = 1.0e-3;
    Boolean gpsPositionAccepted;
    Boolean gpsVelocityAccepted;
    Boolean mocapAccepted;
    Boolean flowAccepted;
    Integer gpsPositionReason;
    Integer gpsVelocityReason;
    Integer mocapReason;
    Integer flowReason;
  algorithm
    initialVariances := Estimation.MultiSensorInvariant.InitialVariances(
      position_m2=fill(1.0, 3),
      velocity_m2_s2=fill(0.5, 3),
      attitude_rad2=fill(0.1, 3),
      gyroscopeBias_rad2_s2=fill(0.01, 3),
      accelerometerBias_m2_s4=fill(0.02, 3));
    processNoise := Estimation.MultiSensorInvariant.ProcessNoise(
      gyroscope_rad2_s=identity(3) * 1.0e-5,
      accelerometer_m2_s3=identity(3) * 1.0e-3,
      gyroscopeBias_rad2_s3=identity(3) * 1.0e-8,
      accelerometerBias_m2_s5=identity(3) * 1.0e-6);
    initialized := Estimation.MultiSensorInvariant.initialize(
      zeros(3), {1.0, 0.0, 0.0, 0.0}, initialVariances);
    hoverPrediction := Estimation.MultiSensorInvariant.predict(
      initialized,
      zeros(3),
      {0.0, 0.0, 9.81},
      {0.0, 0.0, -9.81},
      0.01,
      processNoise);
    A := Estimation.MultiSensorInvariant.continuousTransition(
      zeros(3), {0.0, 0.0, 9.81});
    hoverTransition :=
      Estimation.MultiSensorInvariant.discreteTransition(A, 0.01);

    transitionNominal := Estimation.MultiSensorInvariant.NominalState(
      positionWorldEnu_m={1.0, -2.0, 0.5},
      velocityWorldEnu_m_s={0.4, -0.2, 0.1},
      quaternionWorldBody=LieGroups.SO3.Quat.exp_map({0.2, -0.1, 0.3}),
      gyroscopeBiasBodyFlu_rad_s={0.01, -0.02, 0.03},
      accelerometerBiasBodyFlu_m_s2={0.1, -0.05, 0.02});
    measuredAngularVelocity := {0.5, -0.2, 0.4};
    measuredSpecificForce := {1.0, 0.3, 9.0};
    transitionPredicted := Estimation.MultiSensorInvariant.predictNominal(
      transitionNominal,
      measuredAngularVelocity,
      measuredSpecificForce,
      {0.0, 0.0, -9.81},
      transitionDt);
    A := Estimation.MultiSensorInvariant.continuousTransition(
      measuredAngularVelocity
        - transitionNominal.gyroscopeBiasBodyFlu_rad_s,
      measuredSpecificForce
        - transitionNominal.accelerometerBiasBodyFlu_m_s2);
    transition := Estimation.MultiSensorInvariant.discreteTransition(
      A, transitionDt);
    numericalTransition := zeros(15, 15);
    for column in 1:15 loop
      for row in 1:15 loop
        perturbation[row] := if row == column then
            finiteDifferenceStep else 0.0;
      end for;
      transitionPerturbed := Estimation.MultiSensorInvariant.inject(
        transitionNominal, perturbation);
      perturbedPredicted := Estimation.MultiSensorInvariant.predictNominal(
        transitionPerturbed,
        measuredAngularVelocity,
        measuredSpecificForce,
        {0.0, 0.0, -9.81},
        transitionDt);
      nominalExtendedPose := cat(1,
        transitionPredicted.positionWorldEnu_m,
        transitionPredicted.velocityWorldEnu_m_s,
        transitionPredicted.quaternionWorldBody);
      perturbedExtendedPose := cat(1,
        perturbedPredicted.positionWorldEnu_m,
        perturbedPredicted.velocityWorldEnu_m_s,
        perturbedPredicted.quaternionWorldBody);
      invariantError := LieGroups.SE23.Quat.product(
        LieGroups.SE23.Quat.inverse(nominalExtendedPose),
        perturbedExtendedPose);
      invariantErrorTangent := LieGroups.SE23.Quat.log_map(invariantError);
      numericalTransition[1:9, column] :=
        invariantErrorTangent / finiteDifferenceStep;
      numericalTransition[10:12, column] :=
        (perturbedPredicted.gyroscopeBiasBodyFlu_rad_s
          - transitionPredicted.gyroscopeBiasBodyFlu_rad_s)
          / finiteDifferenceStep;
      numericalTransition[13:15, column] :=
        (perturbedPredicted.accelerometerBiasBodyFlu_m_s2
          - transitionPredicted.accelerometerBiasBodyFlu_m_s2)
          / finiteDifferenceStep;
    end for;

    gps := Avionics.GpsSample(
      valid=true,
      fresh=true,
      positionValid=true,
      velocityValid=true,
      timestamp_s=0.01,
      geodetic_deg_m=zeros(3),
      positionWorldEnu_m={1.0, 0.0, 0.0},
      velocityWorldEnu_m_s={0.5, 0.0, 0.0},
      positionCovarianceWorld_m2=identity(3) * 0.01,
      velocityCovarianceWorld_m2_s2=identity(3) * 0.02);
    (gpsPositionCorrection, gpsPositionAccepted, gpsPositionReason) :=
      Estimation.MultiSensorInvariant.correctGpsPosition(
        hoverPrediction, gps);
    (gpsVelocityCorrection, gpsVelocityAccepted, gpsVelocityReason) :=
      Estimation.MultiSensorInvariant.correctGpsVelocity(
        gpsPositionCorrection, gps);

    mocap := Avionics.MocapSample(
      valid=true,
      fresh=true,
      timestamp_s=0.01,
      positionWorldEnu_m=gpsVelocityCorrection.positionWorldEnu_m,
      quaternionWorldBody=LieGroups.SO3.Quat.exp_map({0.0, 0.0, 0.1}),
      positionCovarianceWorld_m2=identity(3) * 0.001,
      attitudeCovarianceBody_rad2=identity(3) * 0.001);
    (mocapCorrection, mocapAccepted, mocapReason) :=
      Estimation.MultiSensorInvariant.correctMocap(
        gpsVelocityCorrection, mocap);

    flowPrior := initialized;
    flowPrior.velocityWorldEnu_m_s := {1.0, 0.0, 0.0};
    opticalFlow := Avionics.OpticalFlowSample(
      valid=true,
      fresh=true,
      timestamp_s=0.01,
      velocityBodyFlu_m_s={0.5, 0.0},
      velocityCovarianceBody_m2_s2=identity(2) * 0.01,
      integratedLineOfSight_rad=zeros(2),
      integrationTime_s=0.01,
      groundDistance_m=1.0,
      quality=1.0);
    (flowCorrection, flowAccepted, flowReason) :=
      Estimation.MultiSensorInvariant.correctOpticalFlow(
        flowPrior, opticalFlow);

    imu := Avionics.ImuSample(
      valid=true,
      fresh=true,
      timestamp_s=0.01,
      angularVelocityBodyFlu_rad_s=zeros(3),
      specificForceBodyFlu_m_s2={0.0, 0.0, 9.81});
    (navigationValid,
     navigationTimestamp_s,
     navigationPositionWorldEnu_m,
     navigationVelocityWorldEnu_m_s,
     navigationAccelerationWorldEnu_m_s2,
     navigationQuaternionWorldBody,
     navigationRotationWorldBody,
     navigationEulerRpy_rad,
     navigationAngularVelocityBodyFlu_rad_s,
     navigationAngularVelocityWorldEnu_rad_s) :=
      Estimation.MultiSensorInvariant.navigationEstimate(
        mocapCorrection, imu, {0.0, 0.0, -9.81}, true);

    assert(Tests.Assertions.maxAbsVector(
        hoverPrediction.positionWorldEnu_m) < tolerance and
      Tests.Assertions.maxAbsVector(
        hoverPrediction.velocityWorldEnu_m_s) < tolerance,
      "Mixed SE_2(3) hover prediction did not cancel gravity");
    assert(abs(hoverTransition[1, 4] - 0.01) < tolerance and
      abs(hoverTransition[4, 8] - 0.0981) < 1.0e-6,
      "Invariant covariance transition omitted position or force coupling");
    assert(Tests.Assertions.maxAbsMatrix(
        transition - numericalTransition) < 2.0e-5,
      "Invariant covariance transition does not linearize mixed prediction");
    assert(Tests.Assertions.maxAbsMatrix(
        hoverPrediction.covariance
          - transpose(hoverPrediction.covariance)) < tolerance,
      "Invariant covariance prediction was not symmetric");
    assert(gpsPositionAccepted and gpsVelocityAccepted and mocapAccepted
      and flowAccepted,
      "A positive-definite multisensor correction was rejected");
    assert(gpsPositionReason
        == Estimation.MultiSensorInvariant.CorrectionAccepted
      and gpsVelocityReason
        == Estimation.MultiSensorInvariant.CorrectionAccepted
      and mocapReason
        == Estimation.MultiSensorInvariant.CorrectionAccepted
      and flowReason
        == Estimation.MultiSensorInvariant.CorrectionAccepted,
      "A disabled innovation gate reported a rejection");
    assert(gpsPositionCorrection.positionWorldEnu_m[1] > 0.0 and
      gpsPositionCorrection.positionWorldEnu_m[1] < 1.0,
      "GPS position correction did not move the estimate toward the sample");
    assert(flowCorrection.velocityWorldEnu_m_s[1] < 1.0 and
      flowCorrection.velocityWorldEnu_m_s[1] > 0.5,
      "Optical-flow correction did not reduce planar velocity error");
    assert(Tests.Assertions.maxAbsMatrix(
        navigationRotationWorldBody
          - LieGroups.SO3.Quat.to_DCM(
              navigationQuaternionWorldBody)) < tolerance and
      abs(navigationEulerRpy_rad[3] - 0.1) < 1.0e-2,
      "Canonical navigation attitude representations are inconsistent");
    // The published estimate is now a positional output list, so a
    // transposed pair of outputs would be well-typed and silently wrong.
    // Every output is checked against the input it must carry.
    assert(navigationValid and
      abs(navigationTimestamp_s - 0.01) < tolerance and
      Tests.Assertions.maxAbsVector(
        navigationPositionWorldEnu_m
          - mocapCorrection.positionWorldEnu_m) < tolerance and
      Tests.Assertions.maxAbsVector(
        navigationVelocityWorldEnu_m_s
          - mocapCorrection.velocityWorldEnu_m_s) < tolerance and
      Tests.Assertions.maxAbsVector(
        navigationQuaternionWorldBody
          - mocapCorrection.quaternionWorldBody) < tolerance and
      Tests.Assertions.maxAbsVector(
        navigationAngularVelocityBodyFlu_rad_s
          - (imu.angularVelocityBodyFlu_rad_s
            - mocapCorrection.gyroscopeBiasBodyFlu_rad_s)) < tolerance and
      Tests.Assertions.maxAbsVector(
        navigationAngularVelocityWorldEnu_rad_s
          - navigationRotationWorldBody
            * navigationAngularVelocityBodyFlu_rad_s) < tolerance and
      Tests.Assertions.maxAbsVector(
        navigationAccelerationWorldEnu_m_s2
          - (navigationRotationWorldBody
            * (imu.specificForceBodyFlu_m_s2
              - mocapCorrection.accelerometerBiasBodyFlu_m_s2)
            + {0.0, 0.0, -9.81})) < tolerance,
      "Published navigation outputs do not match their declared sources");
    passed := true;
  end run;

  parameter Boolean passed = run();
equation
  assert(passed, "Multisensor invariant estimator tests did not complete");
end MultiSensorInvariantTests;
