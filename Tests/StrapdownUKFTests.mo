within Tests;

model StrapdownUKFTests
  "Manifold UKF sigma-point, prediction, correction, and rejection regressions"
  function run
    output Boolean passed;
  protected
    Estimation.StrapdownINS.InitialVariances initialVariances;
    Estimation.StrapdownINS.ProcessNoise processNoise;
    Estimation.StrapdownINS.UKF.State initialized;
    Estimation.StrapdownINS.UKF.State predicted;
    Estimation.StrapdownINS.UKF.State corrected;
    Estimation.StrapdownINS.UKF.State invalidState;
    Real sigma[15, 31];
    Real reconstructed[15, 15];
    Boolean sigmaSuccess;
    Boolean predictionSuccess;
    Boolean accepted;
    Integer reason;
    Real nis;
    Real nominal[16];
    Real perturbed[16];
    Real perturbation[15];
    Real recovered[15];
    Avionics.GpsSample gps;
  algorithm
    initialVariances := Estimation.StrapdownINS.InitialVariances(
      position_m2=fill(0.04, 3),
      velocity_m2_s2=fill(0.01, 3),
      attitude_rad2=fill(0.007615435494667714, 3),
      gyroscopeBias_rad2_s2=fill(1.0e-5, 3),
      accelerometerBias_m2_s4=fill(1.0e-3, 3));
    processNoise := Estimation.StrapdownINS.ProcessNoise(
      gyroscope_rad2_s=identity(3) * 1.0e-5,
      accelerometer_m2_s3=identity(3) * 1.0e-3,
      gyroscopeBias_rad2_s3=identity(3) * 1.0e-8,
      accelerometerBias_m2_s5=identity(3) * 1.0e-6);
    initialized := Estimation.StrapdownINS.UKF.initialize(
      {0.1, -0.1, 1.0}, zeros(3), {1.0, 0.0, 0.0, 0.0},
      zeros(3), zeros(3), initialVariances);

    (sigma, sigmaSuccess) := Estimation.StrapdownINS.UKF.sigmaTangents(
      initialized.covariance);
    assert(sigmaSuccess, "UKF did not factor a positive-definite prior");
    reconstructed := zeros(15, 15);
    for point in 2:31 loop
      reconstructed := reconstructed
        + Estimation.StrapdownINS.UKF.SigmaWeight
          * Estimation.StrapdownINS.UKF.outerProduct(
            sigma[:, point], sigma[:, point]);
    end for;
    assert(Tests.Assertions.maxAbsMatrix(
        reconstructed - initialized.covariance) < 1.0e-10,
      "UKF sigma points did not reconstruct the declared covariance");

    nominal := Estimation.StrapdownINS.UKF.stateVector(initialized);
    perturbation := {0.01, -0.02, 0.03, 0.04, -0.05, 0.06,
      0.01, -0.015, 0.02, 0.001, -0.002, 0.003,
      -0.004, 0.005, -0.006};
    perturbed := Estimation.StrapdownINS.UKF.injectVector(
      nominal, perturbation);
    recovered := Estimation.StrapdownINS.UKF.localErrorVector(
      nominal, perturbed);
    assert(Tests.Assertions.maxAbsVector(recovered - perturbation) < 1.0e-8,
      "UKF manifold injection and local error were not inverse operations");

    (predicted, predictionSuccess) := Estimation.StrapdownINS.UKF.predict(
      initialized, zeros(3), {0.0, 0.0, 9.81},
      {0.0, 0.0, -9.81}, 0.01, processNoise);
    assert(predictionSuccess, "UKF hover prediction rejected a valid prior");
    assert(Tests.Assertions.maxAbsVector(
        predicted.positionWorldEnu_m - initialized.positionWorldEnu_m)
        < 1.0e-6,
      "UKF stationary strapdown prediction translated the nominal state");
    assert(abs(predicted.quaternionWorldBody
        * predicted.quaternionWorldBody - 1.0) < 1.0e-10,
      "UKF prediction did not preserve a unit quaternion");
    assert(Tests.Assertions.maxAbsMatrix(
        predicted.covariance - transpose(predicted.covariance)) < 1.0e-10,
      "UKF prediction covariance was not symmetric");

    gps := Avionics.GpsSample(
      valid=true, fresh=true, positionValid=true, velocityValid=true,
      timestamp_s=0.01, geodetic_deg_m=zeros(3),
      positionWorldEnu_m={0.0, 0.0, 1.0},
      velocityWorldEnu_m_s=zeros(3),
      positionCovarianceWorld_m2=identity(3) * 0.01,
      velocityCovarianceWorld_m2_s2=identity(3) * 0.0025);
    (corrected, accepted, reason, nis) :=
      Estimation.StrapdownINS.UKF.correctGps(predicted, gps, 0.0);
    assert(accepted and reason == Estimation.StrapdownINS.CorrectionAccepted,
      "UKF rejected a finite positive-definite GPS correction");
    assert(abs(corrected.positionWorldEnu_m[1])
        < abs(predicted.positionWorldEnu_m[1]),
      "UKF GPS correction did not move position toward the measurement");
    assert(corrected.covariance[1, 1] < predicted.covariance[1, 1]
        and corrected.covariance[4, 4] < predicted.covariance[4, 4],
      "UKF GPS correction did not reduce observed covariance");
    assert(nis >= 0.0 and Tests.Assertions.isFiniteMatrix(
        corrected.covariance),
      "UKF GPS correction produced invalid consistency data");

    invalidState := Estimation.StrapdownINS.UKF.State(
      positionWorldEnu_m=initialized.positionWorldEnu_m,
      velocityWorldEnu_m_s=initialized.velocityWorldEnu_m_s,
      quaternionWorldBody=initialized.quaternionWorldBody,
      gyroscopeBiasBodyFlu_rad_s=initialized.gyroscopeBiasBodyFlu_rad_s,
      accelerometerBiasBodyFlu_m_s2=
        initialized.accelerometerBiasBodyFlu_m_s2,
      covariance=initialized.covariance);
    invalidState.covariance[1, 1] := -1.0;
    (sigma, sigmaSuccess) := Estimation.StrapdownINS.UKF.sigmaTangents(
      invalidState.covariance);
    assert(not sigmaSuccess,
      "UKF Cholesky guard accepted a negative covariance pivot");
    passed := true;
  end run;

  parameter Boolean passed = run();
equation
  assert(passed, "Strapdown INS UKF tests did not complete");
end StrapdownUKFTests;
