within Tests;

model AlignmentCovarianceTests
  "Gravity alignment retains yaw uncertainty and attitude-bias correlation"
  function checkAlignment
    output Boolean result;
  protected
    Estimation.StrapdownINS.ESKF.State prior;
    Estimation.StrapdownINS.ESKF.State posterior;
    Real inverseCovariance[15, 15];
    Boolean factorized;
  algorithm
    prior := Estimation.StrapdownINS.ESKF.initialize(
      zeros(3), {1, 0, 0, 0}, Estimation.StrapdownINS.InitialVariances(
        position_m2=fill(1.0, 3), velocity_m2_s2=fill(1.0, 3),
        attitude_rad2=fill(0.25, 3), gyroscopeBias_rad2_s2=fill(1.0e-6, 3),
        accelerometerBias_m2_s4=fill(0.01, 3)));
    posterior := Estimation.StrapdownINS.ESKF.conditionAlignment(
      prior, {0, 0, 9.81}, 0.1 * identity(3), {0, 0, -9.81}, 6.0);
    assert(posterior.covariance[7, 7] < 0.01
      and posterior.covariance[8, 8] < 0.01
      and abs(posterior.covariance[9, 9] - prior.covariance[9, 9]) < 1.0e-12,
      "Gravity alignment did not restrict information to observable tilt");
    assert(Tests.Assertions.maxAbsMatrix(posterior.covariance[7:9, 13:15])
      > 1.0e-5, "Gravity alignment discarded attitude-bias correlation");
    assert(Tests.Assertions.maxAbsVector(posterior.positionWorldEnu_m)
        + Tests.Assertions.maxAbsVector(posterior.velocityWorldEnu_m_s)
      < 1.0e-12, "Alignment invented position or velocity information");
    (inverseCovariance, factorized) := LinearAlgebra.solveSPD(
      posterior.covariance, identity(15));
    assert(factorized, "Conditioned alignment covariance is not positive definite");
    result := true;
  end checkAlignment;
initial algorithm
  assert(checkAlignment(), "Alignment covariance regression failed");
end AlignmentCovarianceTests;
