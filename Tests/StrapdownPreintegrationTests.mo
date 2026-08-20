within Tests;

model StrapdownPreintegrationTests
  "Closed-form IMU composition, bias Jacobian, and filter packet regressions"
  function run
    output Boolean passed;
  protected
    constant Real rawDt = 0.001;
    constant Real packetDt = 0.01;
    constant Real finiteDifferenceStep = 1.0e-6;
    Real measuredAngularVelocity_rad_s[3];
    Real measuredSpecificForce_m_s2[3];
    Real deltaPosition_m[3];
    Real deltaVelocity_m_s[3];
    Real deltaQuaternion[4];
    Real rotationGyroscopeBiasJacobian_s[3, 3];
    Real velocityGyroscopeBiasJacobian_m[3, 3];
    Real velocityAccelerometerBiasJacobian_s[3, 3];
    Real positionGyroscopeBiasJacobian_m_s[3, 3];
    Real positionAccelerometerBiasJacobian_s2[3, 3];
    Real perturbedDeltaPosition_m[3];
    Real perturbedDeltaVelocity_m_s[3];
    Real perturbedDeltaQuaternion[4];
    Real perturbedRotationGyroscopeBiasJacobian_s[3, 3];
    Real perturbedVelocityGyroscopeBiasJacobian_m[3, 3];
    Real perturbedVelocityAccelerometerBiasJacobian_s[3, 3];
    Real perturbedPositionGyroscopeBiasJacobian_m_s[3, 3];
    Real perturbedPositionAccelerometerBiasJacobian_s2[3, 3];
    Real expectedExtendedPose[10];
    Real numericalRotationGyroscopeBiasJacobian_s[3, 3];
    Real numericalVelocityGyroscopeBiasJacobian_m[3, 3];
    Real numericalVelocityAccelerometerBiasJacobian_s[3, 3];
    Real numericalPositionGyroscopeBiasJacobian_m_s[3, 3];
    Real numericalPositionAccelerometerBiasJacobian_s2[3, 3];
    Real gyroBiasAnchor_rad_s[3];
    Real accelerometerBiasAnchor_m_s2[3];
    Real relativeQuaternion[4];
    Avionics.ImuSample imu;
    Estimation.StrapdownINS.InitialVariances initialVariances;
    Estimation.StrapdownINS.ProcessNoise processNoise;
    Estimation.StrapdownINS.ESKF.State eskfInitial;
    Estimation.StrapdownINS.ESKF.State eskfPredicted;
    Estimation.StrapdownINS.ESKF.State eskfSequential;
    Estimation.StrapdownINS.UKF.State ukfInitial;
    Estimation.StrapdownINS.UKF.State ukfPredicted;
    Boolean ukfSuccess;
  algorithm
    measuredAngularVelocity_rad_s := {0.35, -0.22, 0.18};
    measuredSpecificForce_m_s2 := {0.7, -0.4, 9.2};
    deltaPosition_m := zeros(3);
    deltaVelocity_m_s := zeros(3);
    deltaQuaternion := {1.0, 0.0, 0.0, 0.0};
    rotationGyroscopeBiasJacobian_s := zeros(3, 3);
    velocityGyroscopeBiasJacobian_m := zeros(3, 3);
    velocityAccelerometerBiasJacobian_s := zeros(3, 3);
    positionGyroscopeBiasJacobian_m_s := zeros(3, 3);
    positionAccelerometerBiasJacobian_s2 := zeros(3, 3);
    for sampleIndex in 1:10 loop
      (deltaPosition_m,
       deltaVelocity_m_s,
       deltaQuaternion,
       rotationGyroscopeBiasJacobian_s,
       velocityGyroscopeBiasJacobian_m,
       velocityAccelerometerBiasJacobian_s,
       positionGyroscopeBiasJacobian_m_s,
       positionAccelerometerBiasJacobian_s2) :=
        Estimation.StrapdownINS.preintegrateImuStep(
          deltaPosition_m,
          deltaVelocity_m_s,
          deltaQuaternion,
          rotationGyroscopeBiasJacobian_s,
          velocityGyroscopeBiasJacobian_m,
          velocityAccelerometerBiasJacobian_s,
          positionGyroscopeBiasJacobian_m_s,
          positionAccelerometerBiasJacobian_s2,
          measuredAngularVelocity_rad_s,
          measuredSpecificForce_m_s2,
          zeros(3), zeros(3), rawDt);
    end for;
    expectedExtendedPose := LieGroups.SE23.Quat.exp_mixed(
      cat(1, zeros(3), zeros(3), {1.0, 0.0, 0.0, 0.0}),
      cat(1, zeros(3), measuredSpecificForce_m_s2 * packetDt,
        measuredAngularVelocity_rad_s * packetDt),
      zeros(9), [0.0, packetDt; 0.0, 0.0]);
    assert(Tests.Assertions.maxAbsVector(
        deltaPosition_m - expectedExtendedPose[1:3]) < 2.0e-10 and
      Tests.Assertions.maxAbsVector(
        deltaVelocity_m_s - expectedExtendedPose[4:6]) < 2.0e-10 and
      Tests.Assertions.maxAbsVector(
        deltaQuaternion - expectedExtendedPose[7:10]) < 2.0e-10,
      "Composed raw IMU intervals differ from one closed-form held-input packet");

    numericalRotationGyroscopeBiasJacobian_s := zeros(3, 3);
    numericalVelocityGyroscopeBiasJacobian_m := zeros(3, 3);
    numericalVelocityAccelerometerBiasJacobian_s := zeros(3, 3);
    numericalPositionGyroscopeBiasJacobian_m_s := zeros(3, 3);
    numericalPositionAccelerometerBiasJacobian_s2 := zeros(3, 3);
    for column in 1:3 loop
      gyroBiasAnchor_rad_s := {if row == column then
          finiteDifferenceStep else 0.0 for row in 1:3};
      accelerometerBiasAnchor_m_s2 := zeros(3);
      perturbedDeltaPosition_m := zeros(3);
      perturbedDeltaVelocity_m_s := zeros(3);
      perturbedDeltaQuaternion := {1.0, 0.0, 0.0, 0.0};
      perturbedRotationGyroscopeBiasJacobian_s := zeros(3, 3);
      perturbedVelocityGyroscopeBiasJacobian_m := zeros(3, 3);
      perturbedVelocityAccelerometerBiasJacobian_s := zeros(3, 3);
      perturbedPositionGyroscopeBiasJacobian_m_s := zeros(3, 3);
      perturbedPositionAccelerometerBiasJacobian_s2 := zeros(3, 3);
      for sampleIndex in 1:10 loop
        (perturbedDeltaPosition_m,
         perturbedDeltaVelocity_m_s,
         perturbedDeltaQuaternion,
         perturbedRotationGyroscopeBiasJacobian_s,
         perturbedVelocityGyroscopeBiasJacobian_m,
         perturbedVelocityAccelerometerBiasJacobian_s,
         perturbedPositionGyroscopeBiasJacobian_m_s,
         perturbedPositionAccelerometerBiasJacobian_s2) :=
          Estimation.StrapdownINS.preintegrateImuStep(
            perturbedDeltaPosition_m,
            perturbedDeltaVelocity_m_s,
            perturbedDeltaQuaternion,
            perturbedRotationGyroscopeBiasJacobian_s,
            perturbedVelocityGyroscopeBiasJacobian_m,
            perturbedVelocityAccelerometerBiasJacobian_s,
            perturbedPositionGyroscopeBiasJacobian_m_s,
            perturbedPositionAccelerometerBiasJacobian_s2,
            measuredAngularVelocity_rad_s,
            measuredSpecificForce_m_s2,
            gyroBiasAnchor_rad_s, zeros(3), rawDt);
      end for;
      relativeQuaternion := LieGroups.SO3.Quat.product(
        LieGroups.SO3.Quat.inverse(deltaQuaternion),
        perturbedDeltaQuaternion);
      numericalRotationGyroscopeBiasJacobian_s[:, column] :=
        LieGroups.SO3.Quat.log_map(relativeQuaternion)
          / finiteDifferenceStep;
      numericalVelocityGyroscopeBiasJacobian_m[:, column] :=
        (perturbedDeltaVelocity_m_s - deltaVelocity_m_s)
          / finiteDifferenceStep;
      numericalPositionGyroscopeBiasJacobian_m_s[:, column] :=
        (perturbedDeltaPosition_m - deltaPosition_m)
          / finiteDifferenceStep;

      accelerometerBiasAnchor_m_s2 := {if row == column then
          finiteDifferenceStep else 0.0 for row in 1:3};
      gyroBiasAnchor_rad_s := zeros(3);
      perturbedDeltaPosition_m := zeros(3);
      perturbedDeltaVelocity_m_s := zeros(3);
      perturbedDeltaQuaternion := {1.0, 0.0, 0.0, 0.0};
      perturbedRotationGyroscopeBiasJacobian_s := zeros(3, 3);
      perturbedVelocityGyroscopeBiasJacobian_m := zeros(3, 3);
      perturbedVelocityAccelerometerBiasJacobian_s := zeros(3, 3);
      perturbedPositionGyroscopeBiasJacobian_m_s := zeros(3, 3);
      perturbedPositionAccelerometerBiasJacobian_s2 := zeros(3, 3);
      for sampleIndex in 1:10 loop
        (perturbedDeltaPosition_m,
         perturbedDeltaVelocity_m_s,
         perturbedDeltaQuaternion,
         perturbedRotationGyroscopeBiasJacobian_s,
         perturbedVelocityGyroscopeBiasJacobian_m,
         perturbedVelocityAccelerometerBiasJacobian_s,
         perturbedPositionGyroscopeBiasJacobian_m_s,
         perturbedPositionAccelerometerBiasJacobian_s2) :=
          Estimation.StrapdownINS.preintegrateImuStep(
            perturbedDeltaPosition_m,
            perturbedDeltaVelocity_m_s,
            perturbedDeltaQuaternion,
            perturbedRotationGyroscopeBiasJacobian_s,
            perturbedVelocityGyroscopeBiasJacobian_m,
            perturbedVelocityAccelerometerBiasJacobian_s,
            perturbedPositionGyroscopeBiasJacobian_m_s,
            perturbedPositionAccelerometerBiasJacobian_s2,
            measuredAngularVelocity_rad_s,
            measuredSpecificForce_m_s2,
            zeros(3), accelerometerBiasAnchor_m_s2, rawDt);
      end for;
      numericalVelocityAccelerometerBiasJacobian_s[:, column] :=
        (perturbedDeltaVelocity_m_s - deltaVelocity_m_s)
          / finiteDifferenceStep;
      numericalPositionAccelerometerBiasJacobian_s2[:, column] :=
        (perturbedDeltaPosition_m - deltaPosition_m)
          / finiteDifferenceStep;
    end for;
    assert(Tests.Assertions.maxAbsMatrix(
        rotationGyroscopeBiasJacobian_s
          - numericalRotationGyroscopeBiasJacobian_s) < 2.0e-8 and
      Tests.Assertions.maxAbsMatrix(
        velocityGyroscopeBiasJacobian_m
          - numericalVelocityGyroscopeBiasJacobian_m) < 5.0e-5 and
      Tests.Assertions.maxAbsMatrix(
        velocityAccelerometerBiasJacobian_s
          - numericalVelocityAccelerometerBiasJacobian_s) < 5.0e-7 and
      Tests.Assertions.maxAbsMatrix(
        positionGyroscopeBiasJacobian_m_s
          - numericalPositionGyroscopeBiasJacobian_m_s) < 5.0e-7 and
      Tests.Assertions.maxAbsMatrix(
        positionAccelerometerBiasJacobian_s2
          - numericalPositionAccelerometerBiasJacobian_s2) < 5.0e-9,
      "Stored packet bias Jacobians disagree with finite differences");

    imu := Avionics.ImuSample(
      valid=true, fresh=true, timestamp_s=packetDt,
      angularVelocityBodyFlu_rad_s=measuredAngularVelocity_rad_s,
      specificForceBodyFlu_m_s2=measuredSpecificForce_m_s2,
      deltaAngleBodyFlu_rad=LieGroups.SO3.Quat.log_map(deltaQuaternion),
      deltaVelocityBodyFlu_m_s=deltaVelocity_m_s,
      deltaPositionBodyFlu_m=deltaPosition_m,
      deltaQuaternionBodyFlu=deltaQuaternion,
      integrationTime_s=packetDt,
      gyroscopeBiasLinearizationBodyFlu_rad_s=zeros(3),
      accelerometerBiasLinearizationBodyFlu_m_s2=zeros(3),
      deltaRotationGyroscopeBiasJacobian_s=
        rotationGyroscopeBiasJacobian_s,
      deltaVelocityGyroscopeBiasJacobian_m=
        velocityGyroscopeBiasJacobian_m,
      deltaVelocityAccelerometerBiasJacobian_s=
        velocityAccelerometerBiasJacobian_s,
      deltaPositionGyroscopeBiasJacobian_m_s=
        positionGyroscopeBiasJacobian_m_s,
      deltaPositionAccelerometerBiasJacobian_s2=
        positionAccelerometerBiasJacobian_s2);
    initialVariances := Estimation.StrapdownINS.InitialVariances(
      position_m2=fill(0.04, 3), velocity_m2_s2=fill(0.01, 3),
      attitude_rad2=fill(0.01, 3),
      gyroscopeBias_rad2_s2=fill(1.0e-5, 3),
      accelerometerBias_m2_s4=fill(1.0e-3, 3));
    processNoise := Estimation.StrapdownINS.ProcessNoise(
      gyroscope_rad2_s=identity(3) * 1.0e-5,
      accelerometer_m2_s3=identity(3) * 1.0e-3,
      gyroscopeBias_rad2_s3=identity(3) * 1.0e-8,
      accelerometerBias_m2_s5=identity(3) * 1.0e-6);
    eskfInitial := Estimation.StrapdownINS.ESKF.initialize(
      zeros(3), {1.0, 0.0, 0.0, 0.0}, initialVariances);
    eskfPredicted := Estimation.StrapdownINS.ESKF.predictPreintegrated(
      eskfInitial, imu, {0.0, 0.0, -9.81}, processNoise);
    eskfSequential := eskfInitial;
    for sampleIndex in 1:10 loop
      eskfSequential := Estimation.StrapdownINS.ESKF.predict(
        eskfSequential, measuredAngularVelocity_rad_s,
        measuredSpecificForce_m_s2, {0.0, 0.0, -9.81}, rawDt,
        processNoise);
    end for;
    assert(Tests.Assertions.maxAbsVector(
        eskfPredicted.positionWorldEnu_m
          - eskfSequential.positionWorldEnu_m) < 2.0e-10 and
      Tests.Assertions.maxAbsVector(
        eskfPredicted.velocityWorldEnu_m_s
          - eskfSequential.velocityWorldEnu_m_s) < 2.0e-10 and
      Tests.Assertions.maxAbsVector(
        eskfPredicted.quaternionWorldBody
          - eskfSequential.quaternionWorldBody) < 2.0e-10 and
      Tests.Assertions.maxAbsMatrix(
        eskfPredicted.covariance - eskfSequential.covariance) < 2.0e-5,
      "Packet prediction disagrees with ten sequential held-IMU covariance steps");
    ukfInitial := Estimation.StrapdownINS.UKF.initialize(
      zeros(3), zeros(3), {1.0, 0.0, 0.0, 0.0},
      zeros(3), zeros(3), initialVariances);
    (ukfPredicted, ukfSuccess) :=
      Estimation.StrapdownINS.UKF.predictPreintegrated(
        ukfInitial, imu, {0.0, 0.0, -9.81}, processNoise);
    assert(ukfSuccess and
      Tests.Assertions.maxAbsVector(
        eskfPredicted.positionWorldEnu_m
          - ukfPredicted.positionWorldEnu_m) < 2.0e-7 and
      Tests.Assertions.maxAbsVector(
        eskfPredicted.velocityWorldEnu_m_s
          - ukfPredicted.velocityWorldEnu_m_s) < 2.0e-7 and
      Tests.Assertions.maxAbsVector(
        eskfPredicted.quaternionWorldBody
          - ukfPredicted.quaternionWorldBody) < 2.0e-7,
      "ESKF and UKF do not apply the same preintegrated nominal motion");
    passed := true;
  end run;

  parameter Boolean passed = run();
equation
  assert(passed, "Strapdown preintegration tests did not complete");
end StrapdownPreintegrationTests;
