within Tests;

model MahonyFilterStudy
  "Mahony explicit complementary filter against fine-grid attitude truth"

  model FilterCase
    "One scenario: truth kinematics, ideal IMU synthesis, one filter"
    constant Real pi = 3.1415926535897932384626433832795;
    parameter Real samplePeriod_s = 0.005
      "Filter tick, 200 Hz to match the small-MCU deployment rate";
    parameter Real gravity_m_s2 = 9.81;
    parameter Real initialErrorAngle_rad = 0.0
      "Initial estimate-to-truth attitude error angle";
    parameter Real initialErrorAxis[3] = {0.0, 0.0, 1.0};
    parameter Real gyroBiasTruth_rad_s[3] = zeros(3)
      "Constant rate bias added to the synthesized gyro sample";
    parameter Real vibrationFrequency_Hz = 0.0
      "Angular-vibration frequency of the two-axis coning pair";
    parameter Real vibrationAmplitude_rad = 0.0
      "Angular-displacement amplitude of each coning axis";
    parameter Real vibrationSpecificForce_m_s2 = 0.0
      "Specific-force vibration amplitude phase-locked to the pair";
    parameter Real maneuverRate_rad_s = 0.0
      "Slow maneuver angular-rate amplitude on all three axes";
    parameter Boolean useMagnetometer = false;
    parameter Real magneticFieldWorldEnu_T[3] =
      {18.0e-6, 4.0e-6, -47.0e-6};
    parameter Real quaternionNormGain = 1.0;
    parameter Real attitudeErrorLimit_rad = 1.0e9
      "Terminal full-attitude error assertion; huge disables";
    parameter Real tiltErrorLimit_rad = 1.0e9
      "Terminal gravity-direction error assertion; huge disables";
    parameter Real biasErrorLimit_rad_s = 1.0e9
      "Terminal gyro-bias error assertion; huge disables";
    final parameter Real initialErrorAxisNorm = max(
      MathUtilities.norm3(initialErrorAxis), 1.0e-12);
    final parameter Real quaternionTruthStart[4] = {
      cos(0.5 * initialErrorAngle_rad),
      sin(0.5 * initialErrorAngle_rad)
        * initialErrorAxis[1] / initialErrorAxisNorm,
      sin(0.5 * initialErrorAngle_rad)
        * initialErrorAxis[2] / initialErrorAxisNorm,
      sin(0.5 * initialErrorAngle_rad)
        * initialErrorAxis[3] / initialErrorAxisNorm}
      "Truth starts rotated away from the identity-initialized filter";
    final parameter Real vibrationRate_rad_s = 2.0 * pi
      * vibrationFrequency_Hz * vibrationAmplitude_rad;

    output Real attitudeError_rad
      "Angle of the truth-to-estimate error rotation";
    output Real tiltError_rad
      "Angle between true and estimated gravity direction";
    output Real biasError_rad_s
      "Distance between estimated and true gyro bias";

    Estimation.Mahony.Filter filter(
      samplePeriod = samplePeriod_s,
      gravity_m_s2 = gravity_m_s2,
      useMagnetometer = useMagnetometer,
      magneticFieldWorldEnu_T = magneticFieldWorldEnu_T);

  protected
    Real vibrationPhase_rad;
    Real angularVelocityTruth_rad_s[3];
    Real quaternionTruth[4](start = quaternionTruthStart,
      each fixed = true);
    Real rotationTruth[3, 3];
    Real gravityBodyTruth[3];
    Real gravityBodyEstimate[3];
  equation
    vibrationPhase_rad = 2.0 * pi * vibrationFrequency_Hz * time;
    // Two vibration axes 90 degrees out of phase (the coning pair) plus
    // a smooth sub-hertz maneuver on all three axes, mirroring
    // Tests.PreintegrationHoldStudy.
    angularVelocityTruth_rad_s = {
      vibrationRate_rad_s * cos(vibrationPhase_rad)
        + maneuverRate_rad_s * sin(2.0 * pi * 0.30 * time),
      vibrationRate_rad_s * sin(vibrationPhase_rad)
        + 0.6 * maneuverRate_rad_s * sin(2.0 * pi * 0.23 * time + 0.7),
      maneuverRate_rad_s * sin(2.0 * pi * 0.17 * time + 1.3)};
    der(quaternionTruth) = LieGroups.SO3.Quat.kinematics(
      quaternionTruth, angularVelocityTruth_rad_s)
      - quaternionNormGain * (quaternionTruth * quaternionTruth - 1.0)
        * quaternionTruth;
    rotationTruth = LieGroups.SO3.Quat.to_DCM(quaternionTruth);

    // Ideal IMU on a hovering vehicle: the specific force is gravity
    // seen in body axes, plus a body-frame linear-vibration disturbance
    // phase-locked to the coning pair.
    filter.gyro_rad_s = angularVelocityTruth_rad_s + gyroBiasTruth_rad_s;
    filter.accel_m_s2 = transpose(rotationTruth)
      * {0.0, 0.0, gravity_m_s2}
      + vibrationSpecificForce_m_s2
        * {cos(vibrationPhase_rad + 0.5),
           sin(vibrationPhase_rad + 0.5), 0.0};
    filter.magneticFieldBodyFlu_T = transpose(rotationTruth)
      * magneticFieldWorldEnu_T;
    filter.reset = false;

    attitudeError_rad = MathUtilities.norm3(LieGroups.SO3.Quat.log_map(
      LieGroups.SO3.Quat.product(
        LieGroups.SO3.Quat.inverse(quaternionTruth), filter.quaternion)));
    gravityBodyTruth = transpose(rotationTruth) * {0.0, 0.0, 1.0};
    gravityBodyEstimate = transpose(LieGroups.SO3.Quat.to_DCM(
      filter.quaternion)) * {0.0, 0.0, 1.0};
    tiltError_rad = acos(MathUtilities.clip(
      gravityBodyTruth * gravityBodyEstimate, -1.0, 1.0));
    biasError_rad_s = MathUtilities.norm3(
      filter.gyroBias_rad_s - gyroBiasTruth_rad_s);

    when terminal() then
      assert(attitudeError_rad < attitudeErrorLimit_rad,
        "Mahony terminal attitude error exceeded its limit");
      assert(tiltError_rad < tiltErrorLimit_rad,
        "Mahony terminal tilt error exceeded its limit");
      assert(biasError_rad_s < biasErrorLimit_rad_s,
        "Mahony terminal gyro-bias error exceeded its limit");
    end when;
  end FilterCase;

  // Almost-global convergence demonstration (the paper proves it for the
  // continuous filter; here the discrete implementation recovers from a
  // 170 degree error with heading aiding for full-attitude
  // observability).
  FilterCase caseLargeError(
    initialErrorAngle_rad = 170.0 * 3.1415926535897932 / 180.0,
    initialErrorAxis = {1.0, 1.0, 1.0},
    useMagnetometer = true,
    attitudeErrorLimit_rad = 0.02,
    tiltErrorLimit_rad = 0.01,
    biasErrorLimit_rad_s = 0.01);
  // Accelerometer-only recovery: yaw is unobservable, so only the
  // gravity-direction (tilt) error is asserted.
  FilterCase caseTiltRecovery(
    initialErrorAngle_rad = 120.0 * 3.1415926535897932 / 180.0,
    initialErrorAxis = {1.0, 0.5, 0.0},
    tiltErrorLimit_rad = 0.01,
    biasErrorLimit_rad_s = 0.01);
  // Steady-state accuracy under a 25 Hz coning pair with phase-locked
  // linear vibration leaking into the accelerometer. The tilt error
  // settles near one milliradian; the product of the vibrating
  // measured direction with the vibrating estimated direction
  // rectifies into a small constant yaw innovation, and with no
  // heading aiding that unobservable yaw component drifts at a few
  // milliradians per second, so only tilt and bias are asserted.
  FilterCase caseVibration(
    vibrationFrequency_Hz = 25.0,
    vibrationAmplitude_rad = 2.0e-3,
    vibrationSpecificForce_m_s2 = 2.0,
    tiltErrorLimit_rad = 0.02,
    biasErrorLimit_rad_s = 0.02);
  // Slow maneuver with a constant gyro bias: the integral channel must
  // find the bias while heading aiding keeps yaw and its bias
  // observable.
  FilterCase caseManeuverBias(
    maneuverRate_rad_s = 0.5,
    gyroBiasTruth_rad_s = {0.01, -0.02, 0.015},
    useMagnetometer = true,
    attitudeErrorLimit_rad = 0.05,
    tiltErrorLimit_rad = 0.03,
    biasErrorLimit_rad_s = 0.005);

  output Real attitudeErrors_rad[4];
  output Real tiltErrors_rad[4];
  output Real biasErrors_rad_s[4];
equation
  attitudeErrors_rad = {
    caseLargeError.attitudeError_rad,
    caseTiltRecovery.attitudeError_rad,
    caseVibration.attitudeError_rad,
    caseManeuverBias.attitudeError_rad};
  tiltErrors_rad = {
    caseLargeError.tiltError_rad,
    caseTiltRecovery.tiltError_rad,
    caseVibration.tiltError_rad,
    caseManeuverBias.tiltError_rad};
  biasErrors_rad_s = {
    caseLargeError.biasError_rad_s,
    caseTiltRecovery.biasError_rad_s,
    caseVibration.biasError_rad_s,
    caseManeuverBias.biasError_rad_s};

  annotation(experiment(StartTime = 0.0, StopTime = 30.0,
    Tolerance = 1.0e-8, Interval = 0.005));
end MahonyFilterStudy;
