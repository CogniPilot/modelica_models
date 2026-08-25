within Tests;

model PreintegrationHoldStudy
  "Zero-order versus first-order-hold IMU composition against fine-grid truth"

  model HoldCase
    "One coning parameter point integrated by both hold models"
    constant Real pi = 3.1415926535897932384626433832795;
    parameter Real vibrationFrequency_Hz
      "Angular-vibration frequency of the two-axis coning pair";
    parameter Real vibrationAmplitude_rad
      "Angular-displacement amplitude of each coning axis";
    parameter Real samplePeriod_s = 0.001
      "Sub-sample composition interval fed to the preintegrator";
    parameter Real vibrationSpecificForce_m_s2 = 2.0
      "Specific-force vibration amplitude phase-locked to the coning pair";
    parameter Real maneuverRate_rad_s = 0.5
      "Slow maneuver angular-rate amplitude";
    parameter Real maneuverSpecificForce_m_s2 = 0.5
      "Slow maneuver specific-force amplitude";
    parameter Real quaternionNormGain = 1.0;
    final parameter Real vibrationRate_rad_s = 2.0 * pi
      * vibrationFrequency_Hz * vibrationAmplitude_rad
      "Angular-rate amplitude of the coning pair";
    final parameter Real trueConingRate_rad_s = 0.5
      * vibrationAmplitude_rad * vibrationAmplitude_rad
      * 2.0 * pi * vibrationFrequency_Hz
      "Approximate net coning rate of the vibration pair";
    final parameter Real predictedZohResidualRate_rad_s =
      trueConingRate_rad_s
        * (2.0 * pi * vibrationFrequency_Hz * samplePeriod_s)^2 / 12.0
      "Composed zero-order-hold coning residual, (2 pi f h)^2/12 scaling";
    final parameter Real predictedFohRelativeResidual =
      0.1 * (2.0 * pi * vibrationFrequency_Hz)^2 * vibrationAmplitude_rad
        * samplePeriod_s * samplePeriod_s
      + (vibrationRate_rad_s * samplePeriod_s)^2 / 15.0
      "Residual of the corrected increment relative to the correction";

    output Real zohAttitudeError_rad(start = 0.0, fixed = true);
    output Real fohAttitudeError_rad(start = 0.0, fixed = true);
    output Real zohVelocityError_m_s(start = 0.0, fixed = true);
    output Real fohVelocityError_m_s(start = 0.0, fixed = true);
    output Real zohPositionError_m(start = 0.0, fixed = true);
    output Real fohPositionError_m(start = 0.0, fixed = true);

  protected
    Real vibrationPhase_rad;
    Real angularVelocityTruth_rad_s[3];
    Real specificForceTruth_m_s2[3];
    Real quaternionTruth[4](start = {1.0, 0.0, 0.0, 0.0},
      each fixed = true);
    Real velocityTruth_m_s[3](each start = 0.0, each fixed = true);
    Real positionTruth_m[3](each start = 0.0, each fixed = true);
    discrete Real zohDeltaPosition_m[3](each start = 0.0,
      each fixed = true);
    discrete Real zohDeltaVelocity_m_s[3](each start = 0.0,
      each fixed = true);
    discrete Real zohDeltaQuaternion[4](start = {1.0, 0.0, 0.0, 0.0},
      each fixed = true);
    discrete Real fohDeltaPosition_m[3](each start = 0.0,
      each fixed = true);
    discrete Real fohDeltaVelocity_m_s[3](each start = 0.0,
      each fixed = true);
    discrete Real fohDeltaQuaternion[4](start = {1.0, 0.0, 0.0, 0.0},
      each fixed = true);
    discrete Real previousAngularVelocitySample_rad_s[3](
      each start = 0.0, each fixed = true);
    discrete Real previousSpecificForceSample_m_s2[3](
      each start = 0.0, each fixed = true);
    discrete Real scratchRotationJacobian[3, 3](each start = 0.0,
      each fixed = true);
    discrete Real scratchVelocityGyroscopeJacobian[3, 3](
      each start = 0.0, each fixed = true);
    discrete Real scratchVelocityAccelerometerJacobian[3, 3](
      each start = 0.0, each fixed = true);
    discrete Real scratchPositionGyroscopeJacobian[3, 3](
      each start = 0.0, each fixed = true);
    discrete Real scratchPositionAccelerometerJacobian[3, 3](
      each start = 0.0, each fixed = true);
  equation
    vibrationPhase_rad = 2.0 * pi * vibrationFrequency_Hz * time;
    // Two vibration axes 90 degrees out of phase (the coning pair) plus a
    // smooth sub-hertz maneuver on all three axes.
    angularVelocityTruth_rad_s = {
      vibrationRate_rad_s * cos(vibrationPhase_rad)
        + maneuverRate_rad_s * sin(2.0 * pi * 0.30 * time),
      vibrationRate_rad_s * sin(vibrationPhase_rad)
        + 0.6 * maneuverRate_rad_s * sin(2.0 * pi * 0.23 * time + 0.7),
      maneuverRate_rad_s * sin(2.0 * pi * 0.17 * time + 1.3)};
    // Specific-force vibration phase-locked to the rate pair excites the
    // sculling and scrolling channels; the constant carries thrust.
    specificForceTruth_m_s2 = {
      vibrationSpecificForce_m_s2 * cos(vibrationPhase_rad + 0.5)
        + maneuverSpecificForce_m_s2 * sin(2.0 * pi * 0.21 * time),
      vibrationSpecificForce_m_s2 * sin(vibrationPhase_rad + 0.5)
        + maneuverSpecificForce_m_s2 * sin(2.0 * pi * 0.19 * time + 0.9),
      9.81};
    der(quaternionTruth) = LieGroups.SO3.Quat.kinematics(
      quaternionTruth, angularVelocityTruth_rad_s)
      - quaternionNormGain * (quaternionTruth * quaternionTruth - 1.0)
        * quaternionTruth;
    der(velocityTruth_m_s) = LieGroups.SO3.Quat.to_DCM(quaternionTruth)
      * specificForceTruth_m_s2;
    der(positionTruth_m) = velocityTruth_m_s;

  algorithm
    when sample(0.0, samplePeriod_s) then
      if time < 0.5 * samplePeriod_s then
        zohDeltaPosition_m := zeros(3);
        zohDeltaVelocity_m_s := zeros(3);
        zohDeltaQuaternion := {1.0, 0.0, 0.0, 0.0};
        fohDeltaPosition_m := zeros(3);
        fohDeltaVelocity_m_s := zeros(3);
        fohDeltaQuaternion := {1.0, 0.0, 0.0, 0.0};
      else
        // Mirror the vehicle driver: the sample closing the interval is
        // held over it under the zero-order hold, while the first-order
        // hold interpolates from the sample that opened it.
        (zohDeltaPosition_m,
         zohDeltaVelocity_m_s,
         zohDeltaQuaternion,
         scratchRotationJacobian,
         scratchVelocityGyroscopeJacobian,
         scratchVelocityAccelerometerJacobian,
         scratchPositionGyroscopeJacobian,
         scratchPositionAccelerometerJacobian) :=
          Estimation.StrapdownINS.preintegrateImuStep(
            pre(zohDeltaPosition_m),
            pre(zohDeltaVelocity_m_s),
            pre(zohDeltaQuaternion),
            zeros(3, 3), zeros(3, 3), zeros(3, 3), zeros(3, 3),
            zeros(3, 3),
            angularVelocityTruth_rad_s,
            specificForceTruth_m_s2,
            zeros(3), zeros(3), samplePeriod_s);
        (fohDeltaPosition_m,
         fohDeltaVelocity_m_s,
         fohDeltaQuaternion,
         scratchRotationJacobian,
         scratchVelocityGyroscopeJacobian,
         scratchVelocityAccelerometerJacobian,
         scratchPositionGyroscopeJacobian,
         scratchPositionAccelerometerJacobian) :=
          Estimation.StrapdownINS.preintegrateImuStep(
            pre(fohDeltaPosition_m),
            pre(fohDeltaVelocity_m_s),
            pre(fohDeltaQuaternion),
            zeros(3, 3), zeros(3, 3), zeros(3, 3), zeros(3, 3),
            zeros(3, 3),
            angularVelocityTruth_rad_s,
            specificForceTruth_m_s2,
            zeros(3), zeros(3), samplePeriod_s,
            true,
            pre(previousAngularVelocitySample_rad_s),
            pre(previousSpecificForceSample_m_s2));
      end if;
      previousAngularVelocitySample_rad_s := angularVelocityTruth_rad_s;
      previousSpecificForceSample_m_s2 := specificForceTruth_m_s2;
      zohAttitudeError_rad := MathUtilities.norm3(
        LieGroups.SO3.Quat.log_map(LieGroups.SO3.Quat.product(
          LieGroups.SO3.Quat.inverse(quaternionTruth),
          zohDeltaQuaternion)));
      fohAttitudeError_rad := MathUtilities.norm3(
        LieGroups.SO3.Quat.log_map(LieGroups.SO3.Quat.product(
          LieGroups.SO3.Quat.inverse(quaternionTruth),
          fohDeltaQuaternion)));
      zohVelocityError_m_s := MathUtilities.norm3(
        zohDeltaVelocity_m_s - velocityTruth_m_s);
      fohVelocityError_m_s := MathUtilities.norm3(
        fohDeltaVelocity_m_s - velocityTruth_m_s);
      zohPositionError_m := MathUtilities.norm3(
        zohDeltaPosition_m - positionTruth_m);
      fohPositionError_m := MathUtilities.norm3(
        fohDeltaPosition_m - positionTruth_m);
    end when;
  end HoldCase;

  // The vehicle architecture composes raw IMU samples at 1 kHz
  // (WaypointVehicleSystem.imuSamplePeriod), so the study runs the paper's
  // frequency/amplitude table at that rate; one repeated point at the
  // paper's 5.76 kHz maps the results onto its table through the
  // (2 pi f h) scaling.
  HoldCase case100Hz05mrad(
    vibrationFrequency_Hz = 100.0, vibrationAmplitude_rad = 0.5e-3);
  HoldCase case200Hz05mrad(
    vibrationFrequency_Hz = 200.0, vibrationAmplitude_rad = 0.5e-3);
  HoldCase case400Hz05mrad(
    vibrationFrequency_Hz = 400.0, vibrationAmplitude_rad = 0.5e-3);
  HoldCase case100Hz2mrad(
    vibrationFrequency_Hz = 100.0, vibrationAmplitude_rad = 2.0e-3);
  HoldCase case200Hz2mrad(
    vibrationFrequency_Hz = 200.0, vibrationAmplitude_rad = 2.0e-3);
  HoldCase case400Hz2mrad(
    vibrationFrequency_Hz = 400.0, vibrationAmplitude_rad = 2.0e-3);
  HoldCase case200Hz05mradFast(
    vibrationFrequency_Hz = 200.0, vibrationAmplitude_rad = 0.5e-3,
    samplePeriod_s = 1.0 / 5760.0);

  output Real zohAttitudeErrors_rad[7];
  output Real fohAttitudeErrors_rad[7];
  output Real zohVelocityErrors_m_s[7];
  output Real fohVelocityErrors_m_s[7];
  output Real zohPositionErrors_m[7];
  output Real fohPositionErrors_m[7];
equation
  zohAttitudeErrors_rad = {
    case100Hz05mrad.zohAttitudeError_rad,
    case200Hz05mrad.zohAttitudeError_rad,
    case400Hz05mrad.zohAttitudeError_rad,
    case100Hz2mrad.zohAttitudeError_rad,
    case200Hz2mrad.zohAttitudeError_rad,
    case400Hz2mrad.zohAttitudeError_rad,
    case200Hz05mradFast.zohAttitudeError_rad};
  fohAttitudeErrors_rad = {
    case100Hz05mrad.fohAttitudeError_rad,
    case200Hz05mrad.fohAttitudeError_rad,
    case400Hz05mrad.fohAttitudeError_rad,
    case100Hz2mrad.fohAttitudeError_rad,
    case200Hz2mrad.fohAttitudeError_rad,
    case400Hz2mrad.fohAttitudeError_rad,
    case200Hz05mradFast.fohAttitudeError_rad};
  zohVelocityErrors_m_s = {
    case100Hz05mrad.zohVelocityError_m_s,
    case200Hz05mrad.zohVelocityError_m_s,
    case400Hz05mrad.zohVelocityError_m_s,
    case100Hz2mrad.zohVelocityError_m_s,
    case200Hz2mrad.zohVelocityError_m_s,
    case400Hz2mrad.zohVelocityError_m_s,
    case200Hz05mradFast.zohVelocityError_m_s};
  fohVelocityErrors_m_s = {
    case100Hz05mrad.fohVelocityError_m_s,
    case200Hz05mrad.fohVelocityError_m_s,
    case400Hz05mrad.fohVelocityError_m_s,
    case100Hz2mrad.fohVelocityError_m_s,
    case200Hz2mrad.fohVelocityError_m_s,
    case400Hz2mrad.fohVelocityError_m_s,
    case200Hz05mradFast.fohVelocityError_m_s};
  zohPositionErrors_m = {
    case100Hz05mrad.zohPositionError_m,
    case200Hz05mrad.zohPositionError_m,
    case400Hz05mrad.zohPositionError_m,
    case100Hz2mrad.zohPositionError_m,
    case200Hz2mrad.zohPositionError_m,
    case400Hz2mrad.zohPositionError_m,
    case200Hz05mradFast.zohPositionError_m};
  fohPositionErrors_m = {
    case100Hz05mrad.fohPositionError_m,
    case200Hz05mrad.fohPositionError_m,
    case400Hz05mrad.fohPositionError_m,
    case100Hz2mrad.fohPositionError_m,
    case200Hz2mrad.fohPositionError_m,
    case400Hz2mrad.fohPositionError_m,
    case200Hz05mradFast.fohPositionError_m};

  annotation(experiment(StartTime = 0.0, StopTime = 10.0,
    Tolerance = 1.0e-10, Interval = 0.001));
end PreintegrationHoldStudy;
