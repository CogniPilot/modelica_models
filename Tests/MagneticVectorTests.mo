within Tests;

model MagneticVectorTests
  "Symmetric magnetic output has cubic error and respects the field's nullspace"
  function linearizationError
    input Real angle;
    input Real field[3];
    output Real error;
  protected
    Real rotation[3];
    Real measured[3];
    Real residual[3];
    Real H[3, 3];
    Real R[3, 3];
    Boolean usable;
  algorithm
    rotation := angle * {0.3, -0.4, 0.5} / sqrt(0.5);
    measured := transpose(LieGroups.SO3.Quat.to_DCM(
      LieGroups.SO3.Quat.exp_map(rotation))) * field;
    (residual, H, R, usable) :=
      Estimation.StrapdownINS.magnetometerVectorObservation(
        {1, 0, 0, 0}, measured, 9.0e-14 * identity(3), field);
    assert(usable, "Finite magnetic vector observation was rejected");
    error := sqrt((residual - H * rotation) * (residual - H * rotation));
  end linearizationError;

  function checkObservation
    output Boolean result;
  protected
    Real field[3];
    Real axis[3];
    Real residual[3];
    Real H[3, 3];
    Real R[3, 3];
    Real errors[2];
    Boolean usable;
    Boolean accepted;
    Integer outcome;
    Real nis;
    Estimation.StrapdownINS.ESKF.State prior;
    Estimation.StrapdownINS.ESKF.State posterior;
    Avionics.MagnetometerSample measurement;
  algorithm
    field := {-1.59e-6, 20.04e-6, -47.91e-6};
    errors := {linearizationError(0.4, field), linearizationError(0.2, field)};
    assert(errors[1] / errors[2] > 7.5 and errors[1] / errors[2] < 8.5,
      "Symmetric SO(3) output approximation lost cubic error scaling");

    prior := Estimation.StrapdownINS.ESKF.initialize(
      zeros(3), {1, 0, 0, 0}, Estimation.StrapdownINS.InitialVariances(
        position_m2=fill(1.0, 3), velocity_m2_s2=fill(1.0, 3),
        attitude_rad2=fill(0.25, 3), gyroscopeBias_rad2_s2=fill(1.0e-6, 3),
        accelerometerBias_m2_s4=fill(0.01, 3)));
    measurement := Avionics.MagnetometerSample(
      valid=true, fresh=true, timestamp_s=0,
      magneticFieldBodyFlu_T=field,
      covarianceBody_T2=9.0e-14 * identity(3));
    (posterior, accepted, outcome, nis) :=
      Estimation.StrapdownINS.ESKF.correctMagnetometer(
        prior, measurement, field, 6.0, 0.0, zeros(3), zeros(3),
        {0, 0, -9.81}, 0.25, true);
    axis := field / sqrt(field * field);
    assert(accepted and abs(axis * posterior.covariance[7:9, 7:9] * axis
      - axis * prior.covariance[7:9, 7:9] * axis) < 1.0e-12,
      "A single magnetic vector falsely observed rotation about itself");
    assert(sum({posterior.covariance[axis, axis] for axis in 7:9})
      < sum({prior.covariance[axis, axis] for axis in 7:9}),
      "Magnetic vector failed to reduce observable attitude uncertainty");

    prior := Estimation.StrapdownINS.ESKF.initialize(
      zeros(3), LieGroups.SO3.EulerB321.to_Quat({0, acos(-1.0) / 2, 0}),
      Estimation.StrapdownINS.InitialVariances(
        position_m2=fill(1.0, 3), velocity_m2_s2=fill(1.0, 3),
        attitude_rad2=fill(0.25, 3), gyroscopeBias_rad2_s2=fill(1.0e-6, 3),
        accelerometerBias_m2_s4=fill(0.01, 3)));
    measurement := Avionics.MagnetometerSample(
      valid=true, fresh=true, timestamp_s=0,
      magneticFieldBodyFlu_T=transpose(
        LieGroups.SO3.Quat.to_DCM(prior.quaternionWorldBody)) * field,
      covarianceBody_T2=9.0e-14 * identity(3));
    (posterior, accepted, outcome, nis) :=
      Estimation.StrapdownINS.ESKF.correctMagnetometer(
        prior, measurement, field, 6.0, 0.0, zeros(3), zeros(3),
        {0, 0, -9.81}, 0.25, true);
    assert(accepted, "Vector fusion inherited an Euler pitch singularity");

    measurement := Avionics.MagnetometerSample(
      valid=true, fresh=true, timestamp_s=0,
      magneticFieldBodyFlu_T=10 * field,
      covarianceBody_T2=9.0e-14 * identity(3));
    (posterior, accepted, outcome, nis) :=
      Estimation.StrapdownINS.ESKF.correctMagnetometer(
        prior, measurement, field, 6.0, 0.0, zeros(3), zeros(3),
        {0, 0, -9.81}, 0.25, true);
    assert(not accepted and outcome == Estimation.StrapdownINS.CorrectionRejectedGate,
      "Magnetic field disturbance bypassed the innovation gate");
    (residual, H, R, usable) :=
      Estimation.StrapdownINS.magnetometerVectorObservation(
        {1, 0, 0, 0}, field, 9.0e-14 * identity(3), zeros(3));
    assert(not usable, "A zero reference field was accepted");
    result := true;
  end checkObservation;
initial algorithm
  assert(checkObservation(), "Magnetic vector regression failed");
end MagneticVectorTests;
