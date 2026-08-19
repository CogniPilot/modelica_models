within Tests;

model StrapdownESKFTests
  "Aided strapdown INS ESKF prediction, covariance, correction, and interfaces"
  function run
    output Boolean passed;
  protected
    constant Real tolerance = 1.0e-8;
    Estimation.StrapdownINS.InitialVariances initialVariances;
    Estimation.StrapdownINS.ProcessNoise processNoise;
    Estimation.StrapdownINS.ESKF.State initialized;
    Estimation.StrapdownINS.ESKF.State hoverPrediction;
    Estimation.StrapdownINS.ESKF.State gpsPositionCorrection;
    Estimation.StrapdownINS.ESKF.State gpsVelocityCorrection;
    Estimation.StrapdownINS.ESKF.State mocapCorrection;
    Estimation.StrapdownINS.ESKF.State flowPrior;
    Estimation.StrapdownINS.ESKF.State flowCorrection;
    Estimation.StrapdownINS.ESKF.NominalState transitionNominal;
    Estimation.StrapdownINS.ESKF.NominalState transitionPerturbed;
    Estimation.StrapdownINS.ESKF.NominalState transitionPredicted;
    Estimation.StrapdownINS.ESKF.NominalState perturbedPredicted;
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
    Real localGroupError[10];
    Real localErrorTangent[9];
    Real measuredAngularVelocity[3];
    Real measuredSpecificForce[3];
    Real noiseInput[15, 12];
    Real expectedNoiseInput[15, 12];
    Real continuousNoise[12, 12];
    Real discreteNoise[15, 15];
    Real expectedDiscreteNoise[15, 15];
    Real resetCorrection[9];
    Real resetPerturbation[9];
    Real resetGroup[10];
    Real perturbedResetGroup[10];
    Real resetErrorGroup[10];
    Real resetJacobian[9, 9];
    Real numericalResetJacobian[9, 9];
    Real denseResetJacobian[15, 15];
    Real resetPriorCovariance[15, 15];
    Real denseResetCovariance[15, 15];
    Real blockResetCovariance[15, 15];
    Estimation.StrapdownINS.ESKF.State trustPrior;
    Estimation.StrapdownINS.ESKF.State trustCorrected;
    Estimation.StrapdownINS.ESKF.NominalState trustExpectedNominal;
    Real trustResidual[3];
    Real trustH[3, 15];
    Real trustR[3, 3];
    Real trustGain[15, 3];
    Real trustCorrection[15];
    Real trustJosephFactor[15, 15];
    Real trustPosteriorCovariance[15, 15];
    Real trustExpectedCovariance[15, 15];
    Real trustScale;
    Boolean trustAccepted;
    Integer trustReason;
    Real trustNis;
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
    Real nis;
  algorithm
    initialVariances := Estimation.StrapdownINS.InitialVariances(
      position_m2=fill(1.0, 3),
      velocity_m2_s2=fill(0.5, 3),
      attitude_rad2=fill(0.1, 3),
      gyroscopeBias_rad2_s2=fill(0.01, 3),
      accelerometerBias_m2_s4=fill(0.02, 3));
    processNoise := Estimation.StrapdownINS.ProcessNoise(
      gyroscope_rad2_s=identity(3) * 1.0e-5,
      accelerometer_m2_s3=identity(3) * 1.0e-3,
      gyroscopeBias_rad2_s3=identity(3) * 1.0e-8,
      accelerometerBias_m2_s5=identity(3) * 1.0e-6);
    initialized := Estimation.StrapdownINS.ESKF.initialize(
      zeros(3), {1.0, 0.0, 0.0, 0.0}, initialVariances);
    hoverPrediction := Estimation.StrapdownINS.ESKF.predict(
      initialized,
      zeros(3),
      {0.0, 0.0, 9.81},
      {0.0, 0.0, -9.81},
      0.01,
      processNoise);
    A := Estimation.StrapdownINS.ESKF.continuousTransition(
      zeros(3), {0.0, 0.0, 9.81});
    hoverTransition :=
      Estimation.StrapdownINS.ESKF.discreteTransition(A, 0.01);

    // Barfoot SER (2nd ed.), inertial-navigation Eq. (9.208): process noise
    // is ordered gyro, accelerometer, gyro-bias walk, accelerometer-bias
    // walk.  A local/right pose perturbation receives -accelerometer noise
    // in velocity, -gyro noise in attitude, and positive bias walks.
    noiseInput := Estimation.StrapdownINS.ESKF.noiseInputMatrix();
    expectedNoiseInput := cat(1,
      zeros(3, 12),
      cat(2, zeros(3, 3), -identity(3), zeros(3, 6)),
      cat(2, -identity(3), zeros(3, 9)),
      cat(2, zeros(3, 6), identity(3), zeros(3, 3)),
      cat(2, zeros(3, 9), identity(3)));
    continuousNoise :=
      Estimation.StrapdownINS.ESKF.processNoiseMatrix(processNoise);
    discreteNoise := Estimation.StrapdownINS.ESKF.discreteProcessCovariance(
      zeros(15, 15), noiseInput, continuousNoise, 0.01);
    expectedDiscreteNoise := 0.01 * noiseInput * continuousNoise
      * transpose(noiseInput);

    // After right injection x+ = x Exp(delta), the new local error is
    // Log(Exp(-delta) Exp(delta + e)) = Jr(delta)e + O(e^2).  Numerically
    // differentiating that identity catches a left/right Jacobian swap or
    // a sign error in the covariance reset.
    resetCorrection := {0.08, -0.04, 0.03, -0.02, 0.05, 0.01,
      0.12, -0.07, 0.09};
    resetGroup := LieGroups.SE23.Quat.exp_map(resetCorrection);
    resetJacobian := LieGroups.SE23.Quat.right_jacobian(resetCorrection);
    numericalResetJacobian := zeros(9, 9);
    for column in 1:9 loop
      for row in 1:9 loop
        resetPerturbation[row] := if row == column then
            finiteDifferenceStep else 0.0;
      end for;
      perturbedResetGroup := LieGroups.SE23.Quat.exp_map(
        resetCorrection + resetPerturbation);
      resetErrorGroup := LieGroups.SE23.Quat.product(
        LieGroups.SE23.Quat.inverse(resetGroup), perturbedResetGroup);
      numericalResetJacobian[:, column] :=
        LieGroups.SE23.Quat.log_map(resetErrorGroup) / finiteDifferenceStep;
    end for;
    denseResetJacobian := cat(1,
      cat(2, resetJacobian, zeros(9, 6)),
      cat(2, zeros(6, 9), identity(6)));
    resetPriorCovariance := initialized.covariance;
    resetPriorCovariance[1, 10] := 0.004;
    resetPriorCovariance[10, 1] := 0.004;
    denseResetCovariance := denseResetJacobian * resetPriorCovariance
      * transpose(denseResetJacobian);
    blockResetCovariance := Estimation.StrapdownINS.ESKF.conjugateReset(
      resetJacobian, resetPriorCovariance);

    // Exercise the nonlinear trust-region branch.  The effective gain that
    // creates the limited correction must also be the gain in the Joseph
    // covariance update; otherwise the state and covariance describe two
    // different measurement updates.
    trustPrior := initialized;
    trustResidual := {0.0, 0.0, 1.0};
    trustH := cat(2, zeros(3, 6), identity(3), zeros(3, 6));
    trustR := 0.01 * identity(3);
    trustGain := trustPrior.covariance * transpose(trustH)
      * (1.0 / 0.11);
    trustCorrection := trustGain * trustResidual;
    trustScale := Estimation.StrapdownINS.ESKF.MaxAttitudeCorrection_rad
      / sqrt(trustCorrection[7] * trustCorrection[7]
        + trustCorrection[8] * trustCorrection[8]
        + trustCorrection[9] * trustCorrection[9]);
    trustGain := cat(1, trustGain[1:6, :],
      trustScale * trustGain[7:9, :], trustGain[10:15, :]);
    trustCorrection := trustGain * trustResidual;
    trustExpectedNominal := Estimation.StrapdownINS.ESKF.inject(
      Estimation.StrapdownINS.ESKF.NominalState(
        positionWorldEnu_m=trustPrior.positionWorldEnu_m,
        velocityWorldEnu_m_s=trustPrior.velocityWorldEnu_m_s,
        quaternionWorldBody=trustPrior.quaternionWorldBody,
        gyroscopeBiasBodyFlu_rad_s=trustPrior.gyroscopeBiasBodyFlu_rad_s,
        accelerometerBiasBodyFlu_m_s2=
          trustPrior.accelerometerBiasBodyFlu_m_s2),
      trustCorrection);
    trustJosephFactor := identity(15) - trustGain * trustH;
    trustPosteriorCovariance := LinearAlgebra.symmetrize(
      LinearAlgebra.josephUpdate(
        trustJosephFactor, trustPrior.covariance, trustGain, trustR));
    trustExpectedCovariance := LinearAlgebra.symmetrize(
      Estimation.StrapdownINS.ESKF.conjugateReset(
        LieGroups.SE23.Quat.right_jacobian(trustCorrection[1:9]),
        trustPosteriorCovariance));
    (trustCorrected, trustAccepted, trustReason, trustNis) :=
      Estimation.StrapdownINS.ESKF.correctLinear(
        trustPrior, trustResidual, trustH, trustR);

    transitionNominal := Estimation.StrapdownINS.ESKF.NominalState(
      positionWorldEnu_m={1.0, -2.0, 0.5},
      velocityWorldEnu_m_s={0.4, -0.2, 0.1},
      quaternionWorldBody=LieGroups.SO3.Quat.exp_map({0.2, -0.1, 0.3}),
      gyroscopeBiasBodyFlu_rad_s={0.01, -0.02, 0.03},
      accelerometerBiasBodyFlu_m_s2={0.1, -0.05, 0.02});
    measuredAngularVelocity := {0.5, -0.2, 0.4};
    measuredSpecificForce := {1.0, 0.3, 9.0};
    transitionPredicted := Estimation.StrapdownINS.ESKF.predictNominal(
      transitionNominal,
      measuredAngularVelocity,
      measuredSpecificForce,
      {0.0, 0.0, -9.81},
      transitionDt);
    A := Estimation.StrapdownINS.ESKF.continuousTransition(
      measuredAngularVelocity
        - transitionNominal.gyroscopeBiasBodyFlu_rad_s,
      measuredSpecificForce
        - transitionNominal.accelerometerBiasBodyFlu_m_s2);
    transition := Estimation.StrapdownINS.ESKF.discreteTransition(
      A, transitionDt);
    numericalTransition := zeros(15, 15);
    for column in 1:15 loop
      for row in 1:15 loop
        perturbation[row] := if row == column then
            finiteDifferenceStep else 0.0;
      end for;
      transitionPerturbed := Estimation.StrapdownINS.ESKF.inject(
        transitionNominal, perturbation);
      perturbedPredicted := Estimation.StrapdownINS.ESKF.predictNominal(
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
      localGroupError := LieGroups.SE23.Quat.product(
        LieGroups.SE23.Quat.inverse(nominalExtendedPose),
        perturbedExtendedPose);
      localErrorTangent := LieGroups.SE23.Quat.log_map(localGroupError);
      numericalTransition[1:9, column] :=
        localErrorTangent / finiteDifferenceStep;
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
    (gpsPositionCorrection, gpsPositionAccepted, gpsPositionReason, nis) :=
      Estimation.StrapdownINS.ESKF.correctGpsPosition(
        hoverPrediction, gps);
    (gpsVelocityCorrection, gpsVelocityAccepted, gpsVelocityReason, nis) :=
      Estimation.StrapdownINS.ESKF.correctGpsVelocity(
        gpsPositionCorrection, gps);

    mocap := Avionics.MocapSample(
      valid=true,
      fresh=true,
      timestamp_s=0.01,
      positionWorldEnu_m=gpsVelocityCorrection.positionWorldEnu_m,
      quaternionWorldBody=LieGroups.SO3.Quat.exp_map({0.0, 0.0, 0.1}),
      positionCovarianceWorld_m2=identity(3) * 0.001,
      attitudeCovarianceBody_rad2=identity(3) * 0.001);
    (mocapCorrection, mocapAccepted, mocapReason, nis) :=
      Estimation.StrapdownINS.ESKF.correctMocap(
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
      groundDistanceVariance_m2=0.01,
      quality=1.0);
    (flowCorrection, flowAccepted, flowReason, nis) :=
      Estimation.StrapdownINS.ESKF.correctOpticalFlow(
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
      Estimation.StrapdownINS.ESKF.navigationEstimate(
        mocapCorrection, imu, {0.0, 0.0, -9.81}, true);

    assert(Tests.Assertions.maxAbsVector(
        hoverPrediction.positionWorldEnu_m) < tolerance and
      Tests.Assertions.maxAbsVector(
        hoverPrediction.velocityWorldEnu_m_s) < tolerance,
      "Mixed SE_2(3) hover prediction did not cancel gravity");
    assert(abs(hoverTransition[1, 4] - 0.01) < tolerance and
      abs(hoverTransition[4, 8] - 0.0981) < 1.0e-6,
      "Geometric error-state covariance transition omitted position or force coupling");
    assert(Tests.Assertions.maxAbsMatrix(
        transition - numericalTransition) < 2.0e-5,
      "Geometric error-state covariance transition does not linearize mixed prediction");
    assert(Tests.Assertions.maxAbsMatrix(noiseInput - expectedNoiseInput)
        < tolerance,
      "Invariant process-noise input matrix disagrees with the local-error convention");
    assert(Tests.Assertions.maxAbsMatrix(
        discreteNoise - expectedDiscreteNoise) < tolerance,
      "Zero-dynamics process covariance is not the exact Qd = dt G Q G' integral");
    assert(Tests.Assertions.maxAbsMatrix(
        resetJacobian - numericalResetJacobian) < 2.0e-6,
      "Covariance reset is not the differential of right correction injection");
    assert(Tests.Assertions.maxAbsMatrix(
        blockResetCovariance - denseResetCovariance) < tolerance,
      "Block covariance reset differs from dense Jr P Jr' conjugation");
    assert(trustAccepted and trustReason
        == Estimation.StrapdownINS.CorrectionAccepted and
      Tests.Assertions.maxAbsVector(
        trustCorrected.quaternionWorldBody
          - trustExpectedNominal.quaternionWorldBody) < tolerance and
      Tests.Assertions.maxAbsMatrix(
        trustCorrected.covariance - trustExpectedCovariance) < tolerance,
      "Trust-limited correction and Joseph covariance used different effective gains");
    assert(Tests.Assertions.maxAbsMatrix(
        hoverPrediction.covariance
          - transpose(hoverPrediction.covariance)) < tolerance,
      "Geometric error-state covariance prediction was not symmetric");
    assert(gpsPositionAccepted and gpsVelocityAccepted and mocapAccepted
      and flowAccepted,
      "A positive-definite multisensor correction was rejected");
    assert(gpsPositionReason
        == Estimation.StrapdownINS.CorrectionAccepted
      and gpsVelocityReason
        == Estimation.StrapdownINS.CorrectionAccepted
      and mocapReason
        == Estimation.StrapdownINS.CorrectionAccepted
      and flowReason
        == Estimation.StrapdownINS.CorrectionAccepted,
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
  assert(passed, "Geometric error-state estimator tests did not complete");
end StrapdownESKFTests;
