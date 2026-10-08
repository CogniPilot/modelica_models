within Tests;

model HeadingConsiderTests
  "Heading fusion preserves considered states and a positive covariance"
  function checkHeading
    output Boolean result;
  protected
    Estimation.StrapdownINS.ESKF.State prior;
    Estimation.StrapdownINS.ESKF.State posterior;
    Avionics.MagnetometerSample measurement;
    Real coupling[15];
    Real magneticField[3];
    Real covariance[15, 15];
    Real inverseCovariance[15, 15];
    Real angles[3];
    Boolean accepted;
    Boolean factorized;
    Integer outcome;
    Real nis;
  algorithm
    coupling := fill(0.1, 15);
    covariance := 0.01 * identity(15) + transpose({coupling}) * {coupling};
    prior := Estimation.StrapdownINS.ESKF.State(
      barometerBiasCrossCovariance=zeros(15),
      positionWorldEnu_m={1, 2, 3},
      velocityWorldEnu_m_s={0.1, 0.2, 0.3},
      quaternionWorldBody={1, 0, 0, 0},
      gyroscopeBiasBodyFlu_rad_s={0.01, -0.02, 0.03},
      accelerometerBiasBodyFlu_m_s2={0.02, -0.01, 0.015},
      covariance=covariance,
        useSquareRootCovariance=false, covarianceRoot=zeros(15, 15));
    magneticField := {-1.59e-6, 20.04e-6, -47.91e-6};
    measurement := Avionics.MagnetometerSample(
      valid=true, fresh=true, timestamp_s=0.0,
      magneticFieldBodyFlu_T=transpose(LieGroups.SO3.Quat.to_DCM(
        LieGroups.SO3.EulerB321.to_Quat({0.1, 0, 0}))) * magneticField,
      covarianceBody_T2=1.0e-12 * identity(3));
    (posterior, accepted, outcome, nis) :=
      Estimation.StrapdownINS.ESKF.correctMagnetometer(
        prior, measurement, magneticField, 6.0);
    assert(accepted, "Usable heading observation was rejected");
    assert(Tests.Assertions.maxAbsVector(
        posterior.positionWorldEnu_m - prior.positionWorldEnu_m) < 1.0e-12
      and Tests.Assertions.maxAbsVector(
        posterior.velocityWorldEnu_m_s - prior.velocityWorldEnu_m_s) < 1.0e-12
      and Tests.Assertions.maxAbsVector(
        posterior.accelerometerBiasBodyFlu_m_s2
          - prior.accelerometerBiasBodyFlu_m_s2) < 1.0e-12,
      "Heading observation changed a considered translation or accelerometer bias");
    assert(Tests.Assertions.maxAbsVector(
        posterior.gyroscopeBiasBodyFlu_rad_s[1:2]
          - prior.gyroscopeBiasBodyFlu_rad_s[1:2]) < 1.0e-12
      and abs(posterior.gyroscopeBiasBodyFlu_rad_s[3]
        - prior.gyroscopeBiasBodyFlu_rad_s[3]) > 1.0e-8,
      "Heading observation did not restrict gyro-bias learning to the vertical");
    angles := LieGroups.SO3.EulerB321.from_Quat(posterior.quaternionWorldBody);
    assert(angles[1] > 0.0 and angles[1] < 0.1
      and abs(angles[2]) + abs(angles[3]) < 1.0e-12,
      "Heading correction did not rotate only about the vertical");
    assert(Tests.Assertions.maxAbsMatrix(posterior.covariance[13:15, 13:15]
        - prior.covariance[13:15, 13:15]) < 1.0e-12,
      "Heading observation reduced a considered accelerometer-bias variance");
    (inverseCovariance, factorized) := LinearAlgebra.solveSPD(
      posterior.covariance, identity(15));
    assert(factorized, "Constrained heading gain lost positive definiteness");
    result := true;
  end checkHeading;
initial algorithm
  assert(checkHeading(), "Heading consider regression failed");
end HeadingConsiderTests;
