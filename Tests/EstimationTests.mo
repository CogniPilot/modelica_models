within Tests;
model EstimationTests "Initialization, prediction, correction, and failure invariants"
  function run "Execute the static estimator assertions"
    output Boolean passed;
  protected
    constant Real tolerance = 1.0e-9;
    Estimation.MocapExternalOdometryIEKF.InitialVariances initialVariances;
    Estimation.MocapExternalOdometryIEKF.InitialVariances zeroVariances;
    Estimation.MocapExternalOdometryIEKF.ProcessNoise noProcessNoise;
    Estimation.MocapExternalOdometryIEKF.ProcessNoise processNoise;
    Estimation.MocapExternalOdometryIEKF.PoseMeasurement exactMeasurement;
    Estimation.MocapExternalOdometryIEKF.PoseMeasurement negativeQuaternionMeasurement;
    Estimation.MocapExternalOdometryIEKF.PoseMeasurementNoise measurementNoise;
    Estimation.MocapExternalOdometryIEKF.PoseMeasurementNoise zeroMeasurementNoise;
    Estimation.MocapExternalOdometryIEKF.State initialized;
    Estimation.MocapExternalOdometryIEKF.State moving;
    Estimation.MocapExternalOdometryIEKF.State zeroDtPrediction;
    Estimation.MocapExternalOdometryIEKF.State predicted;
    Estimation.MocapExternalOdometryIEKF.State corrected;
    Estimation.MocapExternalOdometryIEKF.State signCorrected;
    Estimation.MocapExternalOdometryIEKF.State zeroState;
    Estimation.MocapExternalOdometryIEKF.State rejectedState;
    Estimation.MocapExternalOdometryIEKF.State skippedState;
    Estimation.MocapExternalOdometryIEKF.NominalState injected;
    Estimation.MocapExternalOdometryIEKF.Covariance transition;
    Estimation.MocapExternalOdometryIEKF.Covariance discreteNoise;
    Estimation.MocapExternalOdometryIEKF.MeasurementMatrix measurementMatrix;
    Real expectedAttitudeTransition[3, 3];
    Estimation.PositionVelocityKF.State positionInitialized;
    Estimation.PositionVelocityKF.State positionPrevious;
    Estimation.PositionVelocityKF.State positionPredicted;
    Estimation.PositionVelocityKF.State positionCorrected;
    Estimation.PositionVelocityKF.State positionStepped;
    Estimation.PositionVelocityKF.Gain positionGain;
    Real restAcceleration[3];
    Real positionResidual[3];
    Real stepAcceleration[3];
    Real stepResidual[3];
    Boolean accepted;
    Boolean signAccepted;
    Boolean rejected;
    Boolean skippedAccepted;
    Boolean positionCorrectionApplied;
  algorithm
  initialVariances := Estimation.MocapExternalOdometryIEKF.InitialVariances(
    attitude=0.04,
    velocity=0.09,
    position=0.16,
    angularVelocity=0.25);
  zeroVariances := Estimation.MocapExternalOdometryIEKF.InitialVariances(
    attitude=0.0,
    velocity=0.0,
    position=0.0,
    angularVelocity=0.0);
  noProcessNoise := Estimation.MocapExternalOdometryIEKF.ProcessNoise(
    attitude=0.0,
    velocity=0.0,
    position=0.0,
    angularVelocity=0.0);
  processNoise := Estimation.MocapExternalOdometryIEKF.ProcessNoise(
    attitude=1.0e-5,
    velocity=2.0e-4,
    position=3.0e-6,
    angularVelocity=4.0e-5);
  measurementNoise := Estimation.MocapExternalOdometryIEKF.PoseMeasurementNoise(
    attitude=1.0e-4,
    position=4.0e-4);
  zeroMeasurementNoise := Estimation.MocapExternalOdometryIEKF.PoseMeasurementNoise(
    attitude=0.0,
    position=0.0);

  initialized := Estimation.MocapExternalOdometryIEKF.initialize(
    {1.0, -2.0, 3.0},
    {2.0, 0.0, 0.0, 0.0},
    initialVariances);
  moving.attitude := initialized.attitude;
  moving.velocity := {2.0, -1.0, 0.5};
  moving.position := initialized.position;
  moving.angularVelocity := {0.1, -0.2, 0.3};
  moving.covariance := initialized.covariance;
  zeroDtPrediction := Estimation.MocapExternalOdometryIEKF.predict(
    moving,
    0.0,
    processNoise);
  predicted := Estimation.MocapExternalOdometryIEKF.predict(
    moving,
    0.1,
    processNoise);
  transition := Estimation.MocapExternalOdometryIEKF.tangentTransition(
    0.1, moving.angularVelocity);
  expectedAttitudeTransition := transpose(LieGroups.SO3.Quat.to_DCM(
    LieGroups.SO3.Quat.exp_map(0.1 * moving.angularVelocity)));
  discreteNoise := Estimation.MocapExternalOdometryIEKF.discreteProcessCovariance(
    0.1, processNoise);
  measurementMatrix := Estimation.MocapExternalOdometryIEKF.poseMeasurementMatrix();

  exactMeasurement := Estimation.MocapExternalOdometryIEKF.PoseMeasurement(
    position=predicted.position,
    attitude=predicted.attitude);
  negativeQuaternionMeasurement := Estimation.MocapExternalOdometryIEKF.PoseMeasurement(
    position=predicted.position,
    attitude=-predicted.attitude);
  (corrected, accepted) := Estimation.MocapExternalOdometryIEKF.correct(
    predicted,
    exactMeasurement,
    measurementNoise);
  (signCorrected, signAccepted) := Estimation.MocapExternalOdometryIEKF.correct(
    predicted,
    negativeQuaternionMeasurement,
    measurementNoise);

  zeroState := Estimation.MocapExternalOdometryIEKF.initialize(
    zeros(3),
    {1.0, 0.0, 0.0, 0.0},
    zeroVariances);
  (rejectedState, rejected) := Estimation.MocapExternalOdometryIEKF.correct(
    zeroState,
    Estimation.MocapExternalOdometryIEKF.PoseMeasurement(
      position=zeros(3),
      attitude={1.0, 0.0, 0.0, 0.0}),
    zeroMeasurementNoise);
  (skippedState, skippedAccepted) := Estimation.MocapExternalOdometryIEKF.step(
    moving,
    0.1,
    false,
    exactMeasurement,
    processNoise,
    measurementNoise);
  injected := Estimation.MocapExternalOdometryIEKF.inject(
    Estimation.MocapExternalOdometryIEKF.NominalState(
      attitude=moving.attitude,
      velocity=moving.velocity,
      position=moving.position,
      angularVelocity=moving.angularVelocity),
    {0.0, 0.0, 0.1, 1.0, 2.0, 3.0, -1.0, -2.0, -3.0, 0.4, 0.5, 0.6});

  positionInitialized := Estimation.PositionVelocityKF.initialize(
    {1.0, -2.0, 0.5},
    {2.0, 0.0, -1.0});
  restAcceleration :=
    Estimation.PositionVelocityKF.bodySpecificForceToWorld(
      {1.0, 0.0, 0.0, 0.0},
      {0.0, 0.0, 9.81},
      9.81);
  positionPrevious := positionInitialized;
  positionPredicted := Estimation.PositionVelocityKF.predict(
    positionPrevious,
    {4.0, -2.0, 1.0},
    0.25);
  positionGain := zeros(6, 3);
  for axis in 1:3 loop
    positionGain[axis, axis] := 0.5;
    positionGain[axis + 3, axis] := 0.25;
  end for;
  (positionCorrected, positionResidual) :=
    Estimation.PositionVelocityKF.correct(
      positionPredicted,
      positionPredicted.position + {2.0, -4.0, 1.0},
      positionGain);
  (positionStepped, stepAcceleration, stepResidual,
    positionCorrectionApplied) := Estimation.PositionVelocityKF.step(
      positionPrevious,
      0.1,
      {1.0, 0.0, 0.0, 0.0},
      {0.0, 0.0, 9.81},
      false,
      zeros(3),
      positionGain,
      9.81);

  assert(Tests.Assertions.maxAbsVector(initialized.attitude - {1, 0, 0, 0}) < tolerance,
    "IEKF initialization did not normalize attitude");
  assert(Tests.Assertions.maxAbsVector(initialized.velocity) < tolerance,
    "IEKF initialization velocity was not zero");
  assert(Tests.Assertions.maxAbsVector(initialized.position - {1, -2, 3}) < tolerance,
    "IEKF initialization position was incorrect");
  assert(abs(initialized.covariance[1, 1] - 0.04) < tolerance and
         abs(initialized.covariance[4, 4] - 0.09) < tolerance and
         abs(initialized.covariance[7, 7] - 0.16) < tolerance and
         abs(initialized.covariance[10, 10] - 0.25) < tolerance,
    "IEKF initialization covariance ordering was incorrect");
  assert(Tests.Assertions.maxAbsVector(zeroDtPrediction.position - moving.position) < tolerance and
         Tests.Assertions.maxAbsMatrix(zeroDtPrediction.covariance - moving.covariance) < tolerance,
    "IEKF zero-duration prediction was not an identity");
  assert(Tests.Assertions.maxAbsVector(
      predicted.position - (moving.position + 0.1 * moving.velocity)) < tolerance,
    "IEKF constant-velocity position prediction failed");
  assert(Tests.Assertions.maxAbsMatrix(
           transition[1:3, 1:3] - expectedAttitudeTransition) < tolerance and
         abs(transition[7, 4] - 0.1) < tolerance and
         abs(discreteNoise[1, 1] - 0.1 * processNoise.attitude) < tolerance and
         abs(measurementMatrix[1, 1] - 1.0) < tolerance and
         abs(measurementMatrix[4, 7] - 1.0) < tolerance,
    "Separated estimator transition/noise/measurement maps were inconsistent");
  assert(abs(predicted.attitude * predicted.attitude - 1.0) < tolerance,
    "IEKF prediction did not preserve quaternion norm");
  assert(Tests.Assertions.maxAbsMatrix(
      predicted.covariance - transpose(predicted.covariance)) < tolerance,
    "IEKF prediction covariance was not symmetric");
  assert(accepted and signAccepted,
    "IEKF rejected a positive-definite innovation covariance");
  assert(Tests.Assertions.maxAbsVector(corrected.position - predicted.position) < tolerance and
         Tests.Assertions.maxAbsVector(signCorrected.position - predicted.position) < tolerance,
    "A zero-residual IEKF correction changed the nominal position");
  assert(abs(abs(corrected.attitude * predicted.attitude) - 1.0) < tolerance and
         abs(abs(signCorrected.attitude * predicted.attitude) - 1.0) < tolerance,
    "A zero-residual or sign-equivalent quaternion correction changed attitude");
  assert(corrected.covariance[1, 1] < predicted.covariance[1, 1] and
         corrected.covariance[7, 7] < predicted.covariance[7, 7],
    "IEKF correction did not reduce observed covariance");
  assert(Tests.Assertions.maxAbsMatrix(
      corrected.covariance - transpose(corrected.covariance)) < tolerance,
    "IEKF corrected covariance was not symmetric");
  assert(not rejected and Tests.Assertions.maxAbsMatrix(
         rejectedState.covariance - zeroState.covariance) < tolerance,
    "IEKF singular innovation handling failed");
  assert(not skippedAccepted and Tests.Assertions.maxAbsMatrix(
         skippedState.covariance - predicted.covariance) < tolerance,
    "IEKF skipped correction did not return prediction");
  assert(Tests.Assertions.maxAbsVector(injected.velocity - (moving.velocity + {1, 2, 3})) < tolerance and
         Tests.Assertions.maxAbsVector(injected.position - (moving.position - {1, 2, 3})) < tolerance and
         Tests.Assertions.maxAbsVector(injected.angularVelocity - (moving.angularVelocity + {0.4, 0.5, 0.6})) < tolerance,
    "IEKF tangent injection ordering failed");
  assert(Tests.Assertions.maxAbsVector(
      restAcceleration) < tolerance,
    "Position filter did not remove gravity from a stationary IMU");
  assert(Tests.Assertions.maxAbsVector(
      positionPredicted.position - {1.625, -2.0625, 0.28125}) < tolerance and
      Tests.Assertions.maxAbsVector(
        positionPredicted.velocity - {3.0, -0.5, -0.75}) < tolerance,
    "Position filter constant-acceleration prediction failed");
  assert(Tests.Assertions.maxAbsVector(
      positionResidual - {2.0, -4.0, 1.0}) < tolerance and
      Tests.Assertions.maxAbsVector(
        positionCorrected.position -
          (positionPredicted.position + {1.0, -2.0, 0.5})) < tolerance and
      Tests.Assertions.maxAbsVector(
        positionCorrected.velocity -
          (positionPredicted.velocity + {0.5, -1.0, 0.25})) < tolerance,
    "Position filter fixed-gain correction failed");
  assert(not positionCorrectionApplied and
      Tests.Assertions.maxAbsVector(stepAcceleration) < tolerance and
      Tests.Assertions.maxAbsVector(stepResidual) < tolerance and
      Tests.Assertions.maxAbsVector(
        positionStepped.position - {1.2, -2.0, 0.4}) < tolerance and
      Tests.Assertions.maxAbsVector(
        positionStepped.velocity - positionPrevious.velocity) < tolerance,
    "Position filter prediction-only step failed");
  for i in 1:12 loop
    assert(corrected.covariance[i, i] >= -tolerance,
      "IEKF corrected covariance had a negative diagonal");
  end for;
  passed := true;
  end run;

  parameter Boolean passed = run();
equation
  assert(passed, "Estimator assertion function did not complete");
end EstimationTests;
