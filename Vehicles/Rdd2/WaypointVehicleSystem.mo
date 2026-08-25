within Vehicles.Rdd2;

// SPDX-License-Identifier: Apache-2.0

model WaypointVehicleSystem
  "Closed-loop plant and ideal RTOS composition driven by a waypoint plan"

  parameter Integer maxWaypoints(min = 2) = 8;
  parameter Integer waypointCount(min = 2, max = maxWaypoints) = 8;
  parameter Boolean useGlobalWaypoints = false;
  parameter Integer navigationSource(min = 0, max = 2) = 0
    "Guidance feedback: 0 truth baseline, 1 GPS-aided estimator, 2 optical-flow-aided estimator";
  parameter Geodesy.GeodeticOrigin origin = Geodesy.GeodeticOrigin(
    latitude_deg = 40.4237,
    longitude_deg = -86.9212,
    altitude_m = 180.0);
  parameter Real localRoute[maxWaypoints, 3];
  parameter Real globalRoute[maxWaypoints, 3] =
    Geodesy.routeLocalEnuToGeodetic(origin, localRoute);
  parameter Real waypointVelocityEnu[maxWaypoints, 3] = zeros(maxWaypoints, 3);
  parameter Real waypointYaw[maxWaypoints] = zeros(maxWaypoints);
  parameter Real nominalSpeed(unit = "m/s") = 1.0;
  parameter Real minSegmentDuration(unit = "s") = 1.0;
  parameter Real armTime_s = 1.0;
  parameter Real disarmDelay_s = 3.0;
  parameter Real planningPeriod(unit = "s") = 0.02;
  parameter Real guidancePeriod(unit = "s") = 0.005;
  parameter Real ratePeriod(unit = "s") = 0.001;
  parameter Real imuSamplePeriod(unit = "s") = 0.001
    "1 kHz raw IMU sampling interval";
  parameter Real estimatorSamplePeriod(unit = "s") = 0.005
    "200 Hz estimator service interval for prediction and aiding";
  parameter Real imuPreintegrationPeriod(unit = "s") = 0.01
    "100 Hz coning/sculling-corrected IMU packet interval";
  parameter Boolean useFirstOrderHoldImu = false
    "True composes each raw IMU interval under a first-order hold with the
     coning, sculling, and scrolling cross terms; false keeps the
     zero-order hold of the current sample";
  parameter Real gpsSamplePeriod(unit = "s") = 0.1
    "10 Hz GPS update interval";
  parameter Real gpsLatency_s(unit = "s") = 0.1
    "End-to-end age of each delivered GPS measurement";
  parameter Real opticalFlowSamplePeriod(unit = "s") = 0.01;
  parameter Real opticalFlowLatency_s(unit = "s") = 0.01
    "Age of the optical-flow integration midpoint at packet delivery";
  parameter Real magnetometerSamplePeriod(unit = "s") = 0.05;
  parameter Real magnetometerTransportDelay_s(unit = "s") = 0.02;
  parameter Real barometerSamplePeriod(unit = "s") = 0.02
    "50 Hz pressure-altitude update interval";
  parameter Real barometerTransportDelay_s(unit = "s") = 0.04;
  parameter Real barometerVariance_m2 = 0.25;
  parameter Real barometerBias_m = 0.4
    "Constant pressure-altitude bias learned by the estimator";
  parameter Boolean useWorldMagneticModel = true;
  parameter Real magneticModelDecimalYear = 2026.63;
  parameter Real configuredLocalMagneticFieldWorldEnu_T[3] =
    {18.0e-6, 4.0e-6, -47.0e-6};
  parameter Real magnetometerCovarianceBody_T2[3, 3] =
    diagonal({0.16e-12, 0.25e-12, 0.36e-12});
  parameter Real magnetometerBiasBodyFlu_T[3] = zeros(3)
    "Residual calibrated hard-iron offset";
  parameter Real gravity_m_s2 = 9.80665;
  parameter Boolean enableSensorNoise = true
    "Enable deterministic-seeded Gaussian measurement noise";
  parameter Real sensorNoiseSeed = 20260818.0;
  parameter Real gpsPositionCovarianceWorld_m2[3, 3] = identity(3) * 0.25;
  parameter Real gpsVelocityCovarianceWorld_m2_s2[3, 3] = identity(3) * 0.01;
  parameter Real opticalFlowRateCovariance_rad2_s2[2, 2] =
    identity(2) * 0.0025;
  parameter Real opticalFlowGyroscopeRateCovariance_rad2_s2[3, 3] =
    identity(3) * 1.0e-6;
  parameter Real opticalFlowGroundDistanceVariance_m2 = 0.0025;
  parameter Real opticalFlowMinimumGroundDistance_m = 0.2
    "Near-field cutoff of the downward range/flow unit";
  parameter Real opticalFlowGroundNormalWorldEnu[3] = {0.0, 0.0, 1.0}
    "Unit normal of the locally planar terrain seen by the nadir camera";
  parameter Real opticalFlowGroundPlaneOffset_m = 0.0
    "Plane offset d in normal' * position = d";
  parameter Real opticalFlowNormalizedImageRadius = 0.35
    "Half-width of the calibrated synthetic feature grid";
  parameter Real estimatorInitialPositionWorldEnu_m[3] = zeros(3)
    "Initial Kalman-filter position before absolute aiding is available";
  parameter Estimation.StrapdownINS.InitialVariances estimatorInitialVariances =
    Estimation.StrapdownINS.InitialVariances(
      position_m2=fill(0.04, 3),
      velocity_m2_s2=fill(0.01, 3),
      attitude_rad2=fill(0.007615435494667714, 3),
      gyroscopeBias_rad2_s2=fill(1.0e-5, 3),
      accelerometerBias_m2_s4=fill(1.0e-3, 3))
    "Shared mission prior: 20 cm, 0.1 m/s, 5 deg, and conservative bias standard deviations";

  parameter Real segmentDuration[maxWaypoints - 1] =
    Planning.Bezier.waypointDurations(
      localRoute, nominalSpeed, minSegmentDuration);
  parameter Real trajectoryDuration(unit = "s") = sum(segmentDuration);
  parameter Real disarmTime_s = armTime_s + trajectoryDuration + disarmDelay_s;
  final parameter Real localMagneticFieldWorldEnu_T[3] =
    if useWorldMagneticModel then Geodesy.WMM2025.magneticFieldEnu(
      origin.latitude_deg, origin.longitude_deg, origin.altitude_m,
      magneticModelDecimalYear) else configuredLocalMagneticFieldWorldEnu_T;

  Vehicles.Rdd2.Plant plant;
  Vehicles.Rdd2.AvionicsSystem avionics(
    maxWaypoints = maxWaypoints,
    planningPeriod = planningPeriod,
    guidancePeriod = guidancePeriod,
    ratePeriod = ratePeriod);
  replaceable block EstimatorModel = Vehicles.Rdd2.NavigationEstimator
    constrainedby Estimation.StrapdownINS.PartialEstimator
    "Algorithm selected for aided strapdown navigation";
  EstimatorModel estimator(
    samplePeriod=estimatorSamplePeriod,
    gravityWorldEnu_m_s2 = {0.0, 0.0, -gravity_m_s2},
    initialPositionWorldEnu_m = estimatorInitialPositionWorldEnu_m,
    initialVariances=estimatorInitialVariances,
    minimumOpticalFlowGroundDistance_m=opticalFlowMinimumGroundDistance_m,
    opticalFlowGroundNormalWorldEnu=opticalFlowGroundNormalWorldEnu,
    opticalFlowGroundPlaneOffset_m=opticalFlowGroundPlaneOffset_m,
    localMagneticFieldWorldEnu_T=localMagneticFieldWorldEnu_T);

  output Boolean armed;
  output Real time_s;
  output Real imuSamplePeriod_s;
  output Real estimatorUpdatePeriod_s;
  output Real position_m[3];
  output Real velocity_m_s[3];
  output Real euler_rad[3];
  output Real motorCommand[4];
  output Real thrust_N;
  output Real geodetic[3]
    "{latitude_deg, longitude_deg, altitude_m}";
  output Real missionPhase;
  output Real navigationError_m
    "Distance between the guidance navigation position and truth";
  output Real controllerEstimatorFeedbackError_m
    "Distance between controller navigation feedback and the Kalman estimate";
  output Real gpsPositionNoise_m[3];
  output Real gpsVelocityNoise_m_s[3];
  output Real opticalFlowIntegratedNoise_rad[2];
  output Real opticalFlowGyroscopeNoise_rad_s[3];
  output Real opticalFlowGyroscopeIntegratedNoise_rad[3];
  output Real opticalFlowGroundDistanceNoise_m;
  output Real opticalFlowIdealIntegratedLineOfSight_rad[2];
  output Real opticalFlowIdealGroundDistance_m;
  output Real magnetometerNoiseBodyFlu_T[3];
  output Real magnetometerIdealFieldBodyFlu_T[3];
  output Real barometerNoise_m;
  output Real barometerIdealAltitudeWorldEnu_m;
  output Boolean opticalFlowSurfaceVisible;
  output Real imuAngularVelocityNoise_rad_s[3];
  output Real imuSpecificForceNoise_m_s2[3];
  output Real imuPreintegratedDeltaAngle_rad[3];
  output Real imuPreintegratedDeltaVelocity_m_s[3];
  output Real imuPreintegratedDeltaPosition_m[3];
  output Real imuPreintegrationTime_s;
  output Real imuGyroscopeBias_rad_s[3](each start = 0.0, each fixed = true);
  output Real imuAccelerometerBias_m_s2[3](each start = 0.0, each fixed = true);
  output Real imuAngularVelocityNoiseVariance_rad2_s2[3];
  output Real imuSpecificForceNoiseVariance_m2_s4[3];
  output Real imuGyroscopeBiasIncrementVariance_rad2_s2[3];
  output Real imuAccelerometerBiasIncrementVariance_m2_s4[3];

protected
  Real opticalFlowSurfaceVisibility;
  Real imuGyroscopeBiasDrivingNoise[3]
    "Independent unit-normal samples driving gyro-bias random walks";
  Real imuAccelerometerBiasDrivingNoise[3]
    "Independent unit-normal samples driving accelerometer-bias random walks";
  Real gpsCapturePositionWorldEnu_m[3];
  Real gpsCaptureVelocityWorldEnu_m_s[3];
  Real opticalFlowCapturePositionWorldEnu_m[3];
  Real opticalFlowCaptureVelocityWorldEnu_m_s[3];
  Real opticalFlowCaptureRotationWorldBody[3, 3];
  Real opticalFlowCaptureAngularVelocityBodyFlu_rad_s[3];
  Real magnetometerCaptureQuaternionWorldBody[4];
  Real magnetometerCaptureRotationWorldBody[3, 3];
  Real barometerCaptureAltitudeWorldEnu_m;
  discrete Real gpsPacketTimestamp_s(start = -1.0e30, fixed = true);
  discrete Real gpsPacketPositionWorldEnu_m[3](
    each start = 0.0, each fixed = true);
  discrete Real gpsPacketVelocityWorldEnu_m_s[3](
    each start = 0.0, each fixed = true);
  discrete Real opticalFlowPacketTimestamp_s(
    start = -1.0e30, fixed = true);
  discrete Real opticalFlowPacketIntegratedLineOfSight_rad[2](
    each start = 0.0, each fixed = true);
  discrete Real opticalFlowPacketIntegratedGyroscope_rad[3](
    each start = 0.0, each fixed = true);
  discrete Real opticalFlowPacketGroundDistance_m(start = 1.0, fixed = true);
  discrete Real opticalFlowPacketSurfaceVisibility(
    start = 0.0, fixed = true);
  discrete Real magnetometerPacketTimestamp_s(
    start = -1.0e30, fixed = true);
  discrete Real magnetometerPacketFieldBodyFlu_T[3](
    each start = 0.0, each fixed = true);
  discrete Real barometerPacketTimestamp_s(
    start = -1.0e30, fixed = true);
  discrete Real barometerPacketAltitudeWorldEnu_m(
    start = 0.0, fixed = true);
  discrete Real imuAccumulatedQuaternion[4](
    start = {1.0, 0.0, 0.0, 0.0}, each fixed = true);
  discrete Real imuAccumulatedDeltaVelocity_m_s[3](
    each start = 0.0, each fixed = true);
  discrete Real imuAccumulatedDeltaPosition_m[3](
    each start = 0.0, each fixed = true);
  discrete Real imuAccumulatedTime_s(start = 0.0, fixed = true);
  discrete Real imuMeasuredAngularVelocity_rad_s[3](
    each start = 0.0, each fixed = true)
    "Bias- and noise-corrupted gyroscope sample closing this interval";
  discrete Real imuMeasuredSpecificForce_m_s2[3](
    each start = 0.0, each fixed = true)
    "Bias- and noise-corrupted accelerometer sample closing this interval";
  discrete Real imuPreviousMeasuredAngularVelocity_rad_s[3](
    each start = 0.0, each fixed = true)
    "Measured gyroscope sample that opened this interval (first-order hold)";
  discrete Real imuPreviousMeasuredSpecificForce_m_s2[3](
    each start = 0.0, each fixed = true)
    "Measured accelerometer sample that opened this interval (first-order hold)";
  discrete Real imuUpdatedQuaternion[4](
    start = {1.0, 0.0, 0.0, 0.0}, each fixed = true);
  discrete Real imuUpdatedDeltaVelocity_m_s[3](
    each start = 0.0, each fixed = true);
  discrete Real imuUpdatedDeltaPosition_m[3](
    each start = 0.0, each fixed = true);
  discrete Real imuAccumulatedGyroscopeBiasLinearization_rad_s[3](
    each start = 0.0, each fixed = true);
  discrete Real imuAccumulatedAccelerometerBiasLinearization_m_s2[3](
    each start = 0.0, each fixed = true);
  discrete Real imuAccumulatedRotationGyroscopeBiasJacobian_s[3, 3](
    each start = 0.0, each fixed = true);
  discrete Real imuAccumulatedVelocityGyroscopeBiasJacobian_m[3, 3](
    each start = 0.0, each fixed = true);
  discrete Real imuAccumulatedVelocityAccelerometerBiasJacobian_s[3, 3](
    each start = 0.0, each fixed = true);
  discrete Real imuAccumulatedPositionGyroscopeBiasJacobian_m_s[3, 3](
    each start = 0.0, each fixed = true);
  discrete Real imuAccumulatedPositionAccelerometerBiasJacobian_s2[3, 3](
    each start = 0.0, each fixed = true);
  discrete Real imuUpdatedRotationGyroscopeBiasJacobian_s[3, 3](
    each start = 0.0, each fixed = true);
  discrete Real imuUpdatedVelocityGyroscopeBiasJacobian_m[3, 3](
    each start = 0.0, each fixed = true);
  discrete Real imuUpdatedVelocityAccelerometerBiasJacobian_s[3, 3](
    each start = 0.0, each fixed = true);
  discrete Real imuUpdatedPositionGyroscopeBiasJacobian_m_s[3, 3](
    each start = 0.0, each fixed = true);
  discrete Real imuUpdatedPositionAccelerometerBiasJacobian_s2[3, 3](
    each start = 0.0, each fixed = true);
  discrete Real imuPacketDeltaAngle_rad[3](
    each start = 0.0, each fixed = true);
  discrete Real imuPacketDeltaVelocity_m_s[3](
    each start = 0.0, each fixed = true);
  discrete Real imuPacketDeltaPosition_m[3](
    each start = 0.0, each fixed = true);
  discrete Real imuPacketDeltaQuaternion[4](
    start = {1.0, 0.0, 0.0, 0.0}, each fixed = true);
  discrete Real imuPacketGyroscopeBiasLinearization_rad_s[3](
    each start = 0.0, each fixed = true);
  discrete Real imuPacketAccelerometerBiasLinearization_m_s2[3](
    each start = 0.0, each fixed = true);
  discrete Real imuPacketRotationGyroscopeBiasJacobian_s[3, 3](
    each start = 0.0, each fixed = true);
  discrete Real imuPacketVelocityGyroscopeBiasJacobian_m[3, 3](
    each start = 0.0, each fixed = true);
  discrete Real imuPacketVelocityAccelerometerBiasJacobian_s[3, 3](
    each start = 0.0, each fixed = true);
  discrete Real imuPacketPositionGyroscopeBiasJacobian_m_s[3, 3](
    each start = 0.0, each fixed = true);
  discrete Real imuPacketPositionAccelerometerBiasJacobian_s2[3, 3](
    each start = 0.0, each fixed = true);
  discrete Real imuPacketIntegrationTime_s(start = 0.0, fixed = true);
  discrete Real imuPacketTimestamp_s(start = 0.0, fixed = true);
  // Estimator bias estimate, mirrored on the estimator clock so the IMU
  // packet clause can linearize around the PREVIOUS estimator step's value
  // through pre() (see the packet-boundary branch below).
  discrete Real estimatorGyroscopeBiasMirror_rad_s[3](
    each start = 0.0, each fixed = true);
  discrete Real estimatorAccelerometerBiasMirror_m_s2[3](
    each start = 0.0, each fixed = true);

equation
  opticalFlowGyroscopeIntegratedNoise_rad =
    opticalFlowGyroscopeNoise_rad_s * opticalFlowSamplePeriod;
  gpsCapturePositionWorldEnu_m = {
    delay(plant.truth.positionWorldEnu_m[1], gpsLatency_s),
    delay(plant.truth.positionWorldEnu_m[2], gpsLatency_s),
    delay(plant.truth.positionWorldEnu_m[3], gpsLatency_s)};
  gpsCaptureVelocityWorldEnu_m_s = {
    delay(plant.truth.velocityWorldEnu_m_s[1], gpsLatency_s),
    delay(plant.truth.velocityWorldEnu_m_s[2], gpsLatency_s),
    delay(plant.truth.velocityWorldEnu_m_s[3], gpsLatency_s)};
  opticalFlowCapturePositionWorldEnu_m = {
    delay(plant.truth.positionWorldEnu_m[1], opticalFlowLatency_s),
    delay(plant.truth.positionWorldEnu_m[2], opticalFlowLatency_s),
    delay(plant.truth.positionWorldEnu_m[3], opticalFlowLatency_s)};
  opticalFlowCaptureVelocityWorldEnu_m_s = {
    delay(plant.truth.velocityWorldEnu_m_s[1], opticalFlowLatency_s),
    delay(plant.truth.velocityWorldEnu_m_s[2], opticalFlowLatency_s),
    delay(plant.truth.velocityWorldEnu_m_s[3], opticalFlowLatency_s)};
  opticalFlowCaptureAngularVelocityBodyFlu_rad_s = {
    delay(plant.truth.angularVelocityBodyFlu_rad_s[1],
      opticalFlowLatency_s),
    delay(plant.truth.angularVelocityBodyFlu_rad_s[2],
      opticalFlowLatency_s),
    delay(plant.truth.angularVelocityBodyFlu_rad_s[3],
      opticalFlowLatency_s)};
  opticalFlowCaptureRotationWorldBody = [
    delay(plant.truth.rotationWorldBody[1, 1], opticalFlowLatency_s),
    delay(plant.truth.rotationWorldBody[1, 2], opticalFlowLatency_s),
    delay(plant.truth.rotationWorldBody[1, 3], opticalFlowLatency_s);
    delay(plant.truth.rotationWorldBody[2, 1], opticalFlowLatency_s),
    delay(plant.truth.rotationWorldBody[2, 2], opticalFlowLatency_s),
    delay(plant.truth.rotationWorldBody[2, 3], opticalFlowLatency_s);
    delay(plant.truth.rotationWorldBody[3, 1], opticalFlowLatency_s),
    delay(plant.truth.rotationWorldBody[3, 2], opticalFlowLatency_s),
    delay(plant.truth.rotationWorldBody[3, 3], opticalFlowLatency_s)];
  magnetometerCaptureQuaternionWorldBody = {
    delay(plant.truth.quaternionWorldBody[1], magnetometerTransportDelay_s),
    delay(plant.truth.quaternionWorldBody[2], magnetometerTransportDelay_s),
    delay(plant.truth.quaternionWorldBody[3], magnetometerTransportDelay_s),
    delay(plant.truth.quaternionWorldBody[4], magnetometerTransportDelay_s)};
  magnetometerCaptureRotationWorldBody = LieGroups.SO3.Quat.to_DCM(
    magnetometerCaptureQuaternionWorldBody);
  magnetometerIdealFieldBodyFlu_T =
    transpose(magnetometerCaptureRotationWorldBody)
      * localMagneticFieldWorldEnu_T;
  barometerCaptureAltitudeWorldEnu_m = delay(
    plant.truth.positionWorldEnu_m[3], barometerTransportDelay_s);
  barometerIdealAltitudeWorldEnu_m = barometerCaptureAltitudeWorldEnu_m;

  avionics.plan.valid = time >= armTime_s;
  avionics.plan.sequence = 0;
  avionics.plan.waypointCount = waypointCount;
  avionics.plan.globalFrame = useGlobalWaypoints;
  avionics.plan.originGeodetic = {
    origin.latitude_deg,
    origin.longitude_deg,
    origin.altitude_m};
  if useGlobalWaypoints then
    avionics.plan.waypoint = globalRoute;
  else
    avionics.plan.waypoint = localRoute;
  end if;
  avionics.plan.velocityEnu = waypointVelocityEnu;
  avionics.plan.yaw = waypointYaw;
  avionics.plan.nominalSpeed = nominalSpeed;
  avionics.plan.minSegmentDuration = minSegmentDuration;

  armed = avionics.reference.valid and time < disarmTime_s;
  avionics.armed = armed;
  avionics.mode = 2;
  avionics.pilot.stick = zeros(3);
  avionics.pilot.throttle = 0.0;
  connect(avionics.motorCommands, plant.commands);

  // Onboard sensing: the estimator always predicts on the plant IMU and is
  // aided by one synthetic sensor. Each Gaussian generator is scaled by the
  // exact covariance supplied to the selected filter and can be disabled.
  estimator.reset = false;
  estimator.imu.valid = plant.imu.valid
    and imuPacketIntegrationTime_s > 0.0;
  // Hold the packet until it is replaced; timestamp novelty is the
  // exactly-once handshake across the sensor and estimator clocks.
  estimator.imu.fresh = estimator.imu.valid;
  estimator.imu.timestamp_s = imuPacketTimestamp_s;
  estimator.imu.angularVelocityBodyFlu_rad_s =
    imuPacketDeltaAngle_rad / max(imuPacketIntegrationTime_s, 1.0e-9);
  estimator.imu.specificForceBodyFlu_m_s2 =
    imuPacketDeltaVelocity_m_s / max(imuPacketIntegrationTime_s, 1.0e-9);
  estimator.imu.deltaAngleBodyFlu_rad = imuPacketDeltaAngle_rad;
  estimator.imu.deltaVelocityBodyFlu_m_s = imuPacketDeltaVelocity_m_s;
  estimator.imu.deltaPositionBodyFlu_m = imuPacketDeltaPosition_m;
  estimator.imu.deltaQuaternionBodyFlu = imuPacketDeltaQuaternion;
  estimator.imu.integrationTime_s = imuPacketIntegrationTime_s;
  estimator.imu.gyroscopeBiasLinearizationBodyFlu_rad_s =
    imuPacketGyroscopeBiasLinearization_rad_s;
  estimator.imu.accelerometerBiasLinearizationBodyFlu_m_s2 =
    imuPacketAccelerometerBiasLinearization_m_s2;
  estimator.imu.deltaRotationGyroscopeBiasJacobian_s =
    imuPacketRotationGyroscopeBiasJacobian_s;
  estimator.imu.deltaVelocityGyroscopeBiasJacobian_m =
    imuPacketVelocityGyroscopeBiasJacobian_m;
  estimator.imu.deltaVelocityAccelerometerBiasJacobian_s =
    imuPacketVelocityAccelerometerBiasJacobian_s;
  estimator.imu.deltaPositionGyroscopeBiasJacobian_m_s =
    imuPacketPositionGyroscopeBiasJacobian_m_s;
  estimator.imu.deltaPositionAccelerometerBiasJacobian_s2 =
    imuPacketPositionAccelerometerBiasJacobian_s2;

  estimator.mocap.valid = false;
  estimator.mocap.fresh = false;
  estimator.mocap.timestamp_s = time;
  estimator.mocap.positionWorldEnu_m = zeros(3);
  estimator.mocap.quaternionWorldBody = {1.0, 0.0, 0.0, 0.0};
  estimator.mocap.positionCovarianceWorld_m2 = identity(3);
  estimator.mocap.attitudeCovarianceBody_rad2 = identity(3);

  estimator.gps.valid = navigationSource == 1;
  estimator.gps.fresh = navigationSource == 1
    and sample(0.0, gpsSamplePeriod);
  estimator.gps.positionValid = navigationSource == 1;
  estimator.gps.velocityValid = navigationSource == 1;
  estimator.gps.timestamp_s = gpsPacketTimestamp_s;
  estimator.gps.geodetic_deg_m = geodetic;
  estimator.gps.positionWorldEnu_m = gpsPacketPositionWorldEnu_m;
  estimator.gps.velocityWorldEnu_m_s = gpsPacketVelocityWorldEnu_m_s;
  estimator.gps.positionCovarianceWorld_m2 = gpsPositionCovarianceWorld_m2;
  estimator.gps.velocityCovarianceWorld_m2_s2 =
    gpsVelocityCovarianceWorld_m2_s2;

  (opticalFlowIdealIntegratedLineOfSight_rad,
   opticalFlowIdealGroundDistance_m,
   opticalFlowSurfaceVisibility) = Vehicles.Rdd2.simulateOpticalFlowPlane(
     opticalFlowCapturePositionWorldEnu_m,
     opticalFlowCaptureVelocityWorldEnu_m_s,
     opticalFlowCaptureRotationWorldBody,
     opticalFlowCaptureAngularVelocityBodyFlu_rad_s,
     opticalFlowGroundNormalWorldEnu,
     opticalFlowGroundPlaneOffset_m,
     opticalFlowSamplePeriod,
     opticalFlowNormalizedImageRadius);
  opticalFlowSurfaceVisible = opticalFlowSurfaceVisibility > 0.5;
  estimator.opticalFlow.valid = navigationSource == 2
    and opticalFlowPacketSurfaceVisibility > 0.5
    and opticalFlowPacketGroundDistance_m
      >= opticalFlowMinimumGroundDistance_m;
  estimator.opticalFlow.fresh = navigationSource == 2
    and opticalFlowSurfaceVisible
    and sample(0.0, opticalFlowSamplePeriod);
  estimator.opticalFlow.timestamp_s = opticalFlowPacketTimestamp_s;
  estimator.opticalFlow.integratedLineOfSight_rad =
    opticalFlowPacketIntegratedLineOfSight_rad;
  estimator.opticalFlow.integratedLineOfSightCovariance_rad2 =
    opticalFlowRateCovariance_rad2_s2
      * opticalFlowSamplePeriod * opticalFlowSamplePeriod;
  estimator.opticalFlow.integratedGyroscopeBodyFlu_rad =
    opticalFlowPacketIntegratedGyroscope_rad;
  estimator.opticalFlow.integratedGyroscopeCovariance_rad2 =
    opticalFlowGyroscopeRateCovariance_rad2_s2
      * opticalFlowSamplePeriod * opticalFlowSamplePeriod;
  estimator.opticalFlow.integrationTime_s = opticalFlowSamplePeriod;
  estimator.opticalFlow.groundDistance_m =
    opticalFlowPacketGroundDistance_m;
  estimator.opticalFlow.groundDistanceVariance_m2 =
    opticalFlowGroundDistanceVariance_m2;
  estimator.opticalFlow.quality = 1.0;

  // Raw calibrated magnetic field. The estimator tilt-compensates it into a
  // yaw-only observation; it is never wired directly to the controller.
  estimator.magnetometer.valid = navigationSource <> 0;
  estimator.magnetometer.fresh = navigationSource <> 0
    and sample(0.012, magnetometerSamplePeriod);
  estimator.magnetometer.timestamp_s = magnetometerPacketTimestamp_s;
  estimator.magnetometer.magneticFieldBodyFlu_T =
    magnetometerPacketFieldBodyFlu_T;
  estimator.magnetometer.covarianceBody_T2 = magnetometerCovarianceBody_T2;

  // The barometer front end provides pressure-derived altitude. Its slowly
  // varying offset is estimated separately before vertical-state fusion.
  estimator.barometer.valid = navigationSource <> 0;
  estimator.barometer.fresh = navigationSource <> 0
    and sample(0.005, barometerSamplePeriod);
  estimator.barometer.timestamp_s = barometerPacketTimestamp_s;
  estimator.barometer.altitudeWorldEnu_m =
    barometerPacketAltitudeWorldEnu_m;
  estimator.barometer.variance_m2 = barometerVariance_m2;

  // The truth branch is an explicit controller/plant baseline. Production-like
  // missions select one of the two estimator branches so a comparison can
  // separate controller failures from estimator or aiding failures.
  if navigationSource == 0 then
    avionics.navigation.valid = plant.truth.valid;
    avionics.navigation.timestamp_s = plant.truth.timestamp_s;
    avionics.navigation.positionWorldEnu_m = plant.truth.positionWorldEnu_m;
    avionics.navigation.velocityWorldEnu_m_s =
      plant.truth.velocityWorldEnu_m_s;
    avionics.navigation.accelerationWorldEnu_m_s2 =
      plant.truth.accelerationWorldEnu_m_s2;
    avionics.navigation.quaternionWorldBody =
      plant.truth.quaternionWorldBody;
    avionics.navigation.rotationWorldBody = plant.truth.rotationWorldBody;
    avionics.navigation.eulerRpy_rad = plant.truth.eulerRpy_rad;
    avionics.navigation.angularVelocityBodyFlu_rad_s =
      plant.truth.angularVelocityBodyFlu_rad_s;
    avionics.navigation.angularVelocityWorldEnu_rad_s =
      plant.truth.angularVelocityWorldEnu_rad_s;
  else
    // The avionics tasks tick together with the estimator and act on the
    // estimate its PREVIOUS step published: pre() states that causal order
    // explicitly. Without it the tasks' same-instant read closes a discrete
    // loop through the estimator and the sensor packet clauses.
    avionics.navigation.valid = pre(estimator.estimate.valid);
    avionics.navigation.timestamp_s = pre(estimator.estimate.timestamp_s);
    avionics.navigation.positionWorldEnu_m =
      pre(estimator.estimate.positionWorldEnu_m);
    avionics.navigation.velocityWorldEnu_m_s =
      pre(estimator.estimate.velocityWorldEnu_m_s);
    avionics.navigation.accelerationWorldEnu_m_s2 =
      pre(estimator.estimate.accelerationWorldEnu_m_s2);
    avionics.navigation.quaternionWorldBody =
      pre(estimator.estimate.quaternionWorldBody);
    avionics.navigation.rotationWorldBody =
      pre(estimator.estimate.rotationWorldBody);
    avionics.navigation.eulerRpy_rad = pre(estimator.estimate.eulerRpy_rad);
    avionics.navigation.angularVelocityBodyFlu_rad_s =
      pre(estimator.estimate.angularVelocityBodyFlu_rad_s);
    avionics.navigation.angularVelocityWorldEnu_rad_s =
      pre(estimator.estimate.angularVelocityWorldEnu_rad_s);
  end if;

  time_s = time;
  imuSamplePeriod_s = imuSamplePeriod;
  estimatorUpdatePeriod_s = estimatorSamplePeriod;
  imuPreintegratedDeltaAngle_rad = imuPacketDeltaAngle_rad;
  imuPreintegratedDeltaVelocity_m_s = imuPacketDeltaVelocity_m_s;
  imuPreintegratedDeltaPosition_m = imuPacketDeltaPosition_m;
  imuPreintegrationTime_s = imuPacketIntegrationTime_s;
  position_m = plant.truth.positionWorldEnu_m;
  velocity_m_s = plant.truth.velocityWorldEnu_m_s;
  euler_rad = plant.truth.eulerRpy_rad;
  motorCommand = avionics.motorCommands.motor;
  thrust_N = avionics.thrust_N;
  geodetic = Geodesy.localEnuToGeodetic(
    origin, position_m[1], position_m[2], position_m[3]);
  navigationError_m = MathUtilities.norm3(
    avionics.navigation.positionWorldEnu_m
      - plant.truth.positionWorldEnu_m);
  controllerEstimatorFeedbackError_m = MathUtilities.norm3(
    avionics.navigation.positionWorldEnu_m
      - estimator.estimate.positionWorldEnu_m);
  for i in 1:3 loop
    imuAngularVelocityNoiseVariance_rad2_s2[i] =
      estimator.processNoise.gyroscope_rad2_s[i, i] / imuSamplePeriod;
    imuSpecificForceNoiseVariance_m2_s4[i] =
      estimator.processNoise.accelerometer_m2_s3[i, i] / imuSamplePeriod;
    imuGyroscopeBiasIncrementVariance_rad2_s2[i] =
      estimator.processNoise.gyroscopeBias_rad2_s3[i, i]
        * imuSamplePeriod;
    imuAccelerometerBiasIncrementVariance_m2_s4[i] =
      estimator.processNoise.accelerometerBias_m2_s5[i, i]
        * imuSamplePeriod;
    imuAngularVelocityNoise_rad_s[i] = if enableSensorNoise then
      sqrt(estimator.processNoise.gyroscope_rad2_s[i, i]
        / imuSamplePeriod)
        * Vehicles.Rdd2.standardNormalNoise(
          time / imuSamplePeriod, i + 8.0, sensorNoiseSeed) else 0.0;
    imuSpecificForceNoise_m_s2[i] = if enableSensorNoise then
      sqrt(estimator.processNoise.accelerometer_m2_s3[i, i]
        / imuSamplePeriod)
        * Vehicles.Rdd2.standardNormalNoise(
          time / imuSamplePeriod, i + 11.0, sensorNoiseSeed) else 0.0;
    gpsPositionNoise_m[i] = if enableSensorNoise then
      sqrt(gpsPositionCovarianceWorld_m2[i, i])
        * Vehicles.Rdd2.standardNormalNoise(
          time / gpsSamplePeriod, 1.0 * i, sensorNoiseSeed) else 0.0;
    gpsVelocityNoise_m_s[i] = if enableSensorNoise then
      sqrt(gpsVelocityCovarianceWorld_m2_s2[i, i])
        * Vehicles.Rdd2.standardNormalNoise(
          time / gpsSamplePeriod, i + 3.0, sensorNoiseSeed) else 0.0;
    opticalFlowGyroscopeNoise_rad_s[i] = if enableSensorNoise then
      sqrt(opticalFlowGyroscopeRateCovariance_rad2_s2[i, i])
        * Vehicles.Rdd2.standardNormalNoise(
          time / opticalFlowSamplePeriod, i + 21.0,
          sensorNoiseSeed) else 0.0;
    magnetometerNoiseBodyFlu_T[i] = if enableSensorNoise then
      sqrt(magnetometerCovarianceBody_T2[i, i])
        * Vehicles.Rdd2.standardNormalNoise(
          time / magnetometerSamplePeriod, i + 24.0,
          sensorNoiseSeed) else 0.0;
    assert(gpsPositionCovarianceWorld_m2[i, i] > 0.0
      and gpsVelocityCovarianceWorld_m2_s2[i, i] > 0.0,
      "GPS measurement covariance diagonal must be positive");
    assert(opticalFlowGyroscopeRateCovariance_rad2_s2[i, i] > 0.0
      and magnetometerCovarianceBody_T2[i, i] > 0.0,
      "Optical-flow gyro and magnetometer covariance diagonals must be positive");
    for j in 1:3 loop
      if i <> j then
        assert(abs(gpsPositionCovarianceWorld_m2[i, j]) < 1.0e-15
          and abs(gpsVelocityCovarianceWorld_m2_s2[i, j]) < 1.0e-15,
          "GPS noise fixture currently requires diagonal filter covariance");
        assert(abs(opticalFlowGyroscopeRateCovariance_rad2_s2[i, j])
            < 1.0e-30 and abs(magnetometerCovarianceBody_T2[i, j])
            < 1.0e-30,
          "Gyroscope and magnetometer noise fixtures require diagonal covariance");
      end if;
    end for;
  end for;
  for i in 1:2 loop
    opticalFlowIntegratedNoise_rad[i] = if enableSensorNoise then
      sqrt(opticalFlowRateCovariance_rad2_s2[i, i])
        * opticalFlowSamplePeriod
        * Vehicles.Rdd2.standardNormalNoise(
          time / opticalFlowSamplePeriod, i + 6.0, sensorNoiseSeed) else 0.0;
    assert(opticalFlowRateCovariance_rad2_s2[i, i] > 0.0,
      "Optical-flow measurement covariance diagonal must be positive");
    for j in 1:2 loop
      if i <> j then
        assert(abs(opticalFlowRateCovariance_rad2_s2[i, j]) < 1.0e-30,
          "Optical-flow noise fixture currently requires diagonal filter covariance");
      end if;
    end for;
  end for;
  opticalFlowGroundDistanceNoise_m = if enableSensorNoise then
    sqrt(opticalFlowGroundDistanceVariance_m2)
      * Vehicles.Rdd2.standardNormalNoise(
        time / opticalFlowSamplePeriod, 20.0, sensorNoiseSeed) else 0.0;
  barometerNoise_m = if enableSensorNoise then sqrt(barometerVariance_m2)
    * Vehicles.Rdd2.standardNormalNoise(
      time / barometerSamplePeriod, 28.0, sensorNoiseSeed) else 0.0;
  // Keep the six function calls outside the clocked algorithm and give each
  // one a literal channel. This preserves independent axes in Rumoca, whose
  // algorithm lowering may otherwise hoist an iterator-dependent call.
  imuGyroscopeBiasDrivingNoise[1] =
    Vehicles.Rdd2.standardNormalNoise(
      time / imuSamplePeriod, 29.0, sensorNoiseSeed);
  imuGyroscopeBiasDrivingNoise[2] =
    Vehicles.Rdd2.standardNormalNoise(
      time / imuSamplePeriod, 30.0, sensorNoiseSeed);
  imuGyroscopeBiasDrivingNoise[3] =
    Vehicles.Rdd2.standardNormalNoise(
      time / imuSamplePeriod, 31.0, sensorNoiseSeed);
  imuAccelerometerBiasDrivingNoise[1] =
    Vehicles.Rdd2.standardNormalNoise(
      time / imuSamplePeriod, 32.0, sensorNoiseSeed);
  imuAccelerometerBiasDrivingNoise[2] =
    Vehicles.Rdd2.standardNormalNoise(
      time / imuSamplePeriod, 33.0, sensorNoiseSeed);
  imuAccelerometerBiasDrivingNoise[3] =
    Vehicles.Rdd2.standardNormalNoise(
      time / imuSamplePeriod, 34.0, sensorNoiseSeed);
  assert(opticalFlowGroundDistanceVariance_m2 > 0.0,
    "Optical-flow ground-distance variance must be positive");
  assert(barometerVariance_m2 > 0.0,
    "Barometer altitude variance must be positive");
  assert(abs(opticalFlowGroundNormalWorldEnu
      * opticalFlowGroundNormalWorldEnu - 1.0) < 1.0e-9,
    "Optical-flow ground-plane normal must have unit length");
  if not armed then
    missionPhase = 0.0;
  elseif avionics.reference.complete then
    missionPhase = 2.0;
  else
    missionPhase = 1.0;
  end if;

algorithm
  when sample(0.0, gpsSamplePeriod) then
    gpsPacketTimestamp_s := time - gpsLatency_s;
    gpsPacketPositionWorldEnu_m :=
      gpsCapturePositionWorldEnu_m + gpsPositionNoise_m;
    gpsPacketVelocityWorldEnu_m_s :=
      gpsCaptureVelocityWorldEnu_m_s + gpsVelocityNoise_m_s;
  end when;

  when sample(0.0, opticalFlowSamplePeriod) then
    opticalFlowPacketTimestamp_s := time - opticalFlowLatency_s;
    opticalFlowPacketIntegratedLineOfSight_rad :=
      opticalFlowIdealIntegratedLineOfSight_rad
        + opticalFlowIntegratedNoise_rad;
    opticalFlowPacketIntegratedGyroscope_rad :=
      (opticalFlowCaptureAngularVelocityBodyFlu_rad_s
        + opticalFlowGyroscopeNoise_rad_s) * opticalFlowSamplePeriod;
    opticalFlowPacketGroundDistance_m :=
      opticalFlowIdealGroundDistance_m + opticalFlowGroundDistanceNoise_m;
    opticalFlowPacketSurfaceVisibility := opticalFlowSurfaceVisibility;
  end when;

  when sample(0.012, magnetometerSamplePeriod) then
    magnetometerPacketTimestamp_s := time - magnetometerTransportDelay_s;
    magnetometerPacketFieldBodyFlu_T :=
      magnetometerIdealFieldBodyFlu_T + magnetometerBiasBodyFlu_T
        + magnetometerNoiseBodyFlu_T;
  end when;

  when sample(0.005, barometerSamplePeriod) then
    barometerPacketTimestamp_s := time - barometerTransportDelay_s;
    barometerPacketAltitudeWorldEnu_m :=
      barometerIdealAltitudeWorldEnu_m + barometerBias_m + barometerNoise_m;
  end when;

  when sample(0.0, imuSamplePeriod) then
    imuMeasuredAngularVelocity_rad_s :=
      plant.imu.angularVelocityBodyFlu_rad_s
        + pre(imuGyroscopeBias_rad_s) + imuAngularVelocityNoise_rad_s;
    imuMeasuredSpecificForce_m_s2 := plant.imu.specificForceBodyFlu_m_s2
      + pre(imuAccelerometerBias_m_s2) + imuSpecificForceNoise_m_s2;
    if time < 0.5 * imuSamplePeriod then
      imuUpdatedQuaternion := {1.0, 0.0, 0.0, 0.0};
      imuUpdatedDeltaVelocity_m_s := zeros(3);
      imuUpdatedDeltaPosition_m := zeros(3);
      imuAccumulatedQuaternion := {1.0, 0.0, 0.0, 0.0};
      imuAccumulatedDeltaVelocity_m_s := zeros(3);
      imuAccumulatedDeltaPosition_m := zeros(3);
      imuAccumulatedTime_s := 0.0;
      imuAccumulatedGyroscopeBiasLinearization_rad_s :=
        estimator.initialGyroscopeBiasBodyFlu_rad_s;
      imuAccumulatedAccelerometerBiasLinearization_m_s2 :=
        estimator.initialAccelerometerBiasBodyFlu_m_s2;
      imuAccumulatedRotationGyroscopeBiasJacobian_s := zeros(3, 3);
      imuAccumulatedVelocityGyroscopeBiasJacobian_m := zeros(3, 3);
      imuAccumulatedVelocityAccelerometerBiasJacobian_s := zeros(3, 3);
      imuAccumulatedPositionGyroscopeBiasJacobian_m_s := zeros(3, 3);
      imuAccumulatedPositionAccelerometerBiasJacobian_s2 := zeros(3, 3);
      imuUpdatedRotationGyroscopeBiasJacobian_s := zeros(3, 3);
      imuUpdatedVelocityGyroscopeBiasJacobian_m := zeros(3, 3);
      imuUpdatedVelocityAccelerometerBiasJacobian_s := zeros(3, 3);
      imuUpdatedPositionGyroscopeBiasJacobian_m_s := zeros(3, 3);
      imuUpdatedPositionAccelerometerBiasJacobian_s2 := zeros(3, 3);
      imuPacketDeltaAngle_rad := zeros(3);
      imuPacketDeltaVelocity_m_s := zeros(3);
      imuPacketDeltaPosition_m := zeros(3);
      imuPacketDeltaQuaternion := {1.0, 0.0, 0.0, 0.0};
      imuPacketGyroscopeBiasLinearization_rad_s :=
        estimator.initialGyroscopeBiasBodyFlu_rad_s;
      imuPacketAccelerometerBiasLinearization_m_s2 :=
        estimator.initialAccelerometerBiasBodyFlu_m_s2;
      imuPacketRotationGyroscopeBiasJacobian_s := zeros(3, 3);
      imuPacketVelocityGyroscopeBiasJacobian_m := zeros(3, 3);
      imuPacketVelocityAccelerometerBiasJacobian_s := zeros(3, 3);
      imuPacketPositionGyroscopeBiasJacobian_m_s := zeros(3, 3);
      imuPacketPositionAccelerometerBiasJacobian_s2 := zeros(3, 3);
      imuPacketIntegrationTime_s := 0.0;
      imuPacketTimestamp_s := time;
    else
      (imuUpdatedDeltaPosition_m,
       imuUpdatedDeltaVelocity_m_s,
       imuUpdatedQuaternion,
       imuUpdatedRotationGyroscopeBiasJacobian_s,
       imuUpdatedVelocityGyroscopeBiasJacobian_m,
       imuUpdatedVelocityAccelerometerBiasJacobian_s,
       imuUpdatedPositionGyroscopeBiasJacobian_m_s,
       imuUpdatedPositionAccelerometerBiasJacobian_s2) :=
        Estimation.StrapdownINS.preintegrateImuStep(
          pre(imuAccumulatedDeltaPosition_m),
          pre(imuAccumulatedDeltaVelocity_m_s),
          pre(imuAccumulatedQuaternion),
          pre(imuAccumulatedRotationGyroscopeBiasJacobian_s),
          pre(imuAccumulatedVelocityGyroscopeBiasJacobian_m),
          pre(imuAccumulatedVelocityAccelerometerBiasJacobian_s),
          pre(imuAccumulatedPositionGyroscopeBiasJacobian_m_s),
          pre(imuAccumulatedPositionAccelerometerBiasJacobian_s2),
          imuMeasuredAngularVelocity_rad_s,
          imuMeasuredSpecificForce_m_s2,
          pre(imuAccumulatedGyroscopeBiasLinearization_rad_s),
          pre(imuAccumulatedAccelerometerBiasLinearization_m_s2),
          imuSamplePeriod,
          useFirstOrderHoldImu,
          pre(imuPreviousMeasuredAngularVelocity_rad_s),
          pre(imuPreviousMeasuredSpecificForce_m_s2));
      if abs(time / imuPreintegrationPeriod
          - floor(time / imuPreintegrationPeriod + 0.5)) < 1.0e-7 then
        imuPacketDeltaAngle_rad := LieGroups.SO3.Quat.log_map(
          imuUpdatedQuaternion);
        imuPacketDeltaVelocity_m_s := imuUpdatedDeltaVelocity_m_s;
        imuPacketDeltaPosition_m := imuUpdatedDeltaPosition_m;
        imuPacketDeltaQuaternion := imuUpdatedQuaternion;
        imuPacketGyroscopeBiasLinearization_rad_s :=
          pre(imuAccumulatedGyroscopeBiasLinearization_rad_s);
        imuPacketAccelerometerBiasLinearization_m_s2 :=
          pre(imuAccumulatedAccelerometerBiasLinearization_m_s2);
        imuPacketRotationGyroscopeBiasJacobian_s :=
          imuUpdatedRotationGyroscopeBiasJacobian_s;
        imuPacketVelocityGyroscopeBiasJacobian_m :=
          imuUpdatedVelocityGyroscopeBiasJacobian_m;
        imuPacketVelocityAccelerometerBiasJacobian_s :=
          imuUpdatedVelocityAccelerometerBiasJacobian_s;
        imuPacketPositionGyroscopeBiasJacobian_m_s :=
          imuUpdatedPositionGyroscopeBiasJacobian_m_s;
        imuPacketPositionAccelerometerBiasJacobian_s2 :=
          imuUpdatedPositionAccelerometerBiasJacobian_s2;
        imuPacketIntegrationTime_s := pre(imuAccumulatedTime_s)
          + imuSamplePeriod;
        imuPacketTimestamp_s := time;
        imuAccumulatedQuaternion := {1.0, 0.0, 0.0, 0.0};
        imuAccumulatedDeltaVelocity_m_s := zeros(3);
        imuAccumulatedDeltaPosition_m := zeros(3);
        imuAccumulatedTime_s := 0.0;
        // Linearize the next preintegration window around the bias the
        // estimator published on its PREVIOUS step: the packet clause and
        // the estimator tick together at every packet boundary, and pre()
        // of the mirror states that causal order explicitly instead of
        // leaving a simultaneous discrete loop for the scheduler to
        // serialize.
        imuAccumulatedGyroscopeBiasLinearization_rad_s :=
          pre(estimatorGyroscopeBiasMirror_rad_s);
        imuAccumulatedAccelerometerBiasLinearization_m_s2 :=
          pre(estimatorAccelerometerBiasMirror_m_s2);
        imuAccumulatedRotationGyroscopeBiasJacobian_s := zeros(3, 3);
        imuAccumulatedVelocityGyroscopeBiasJacobian_m := zeros(3, 3);
        imuAccumulatedVelocityAccelerometerBiasJacobian_s := zeros(3, 3);
        imuAccumulatedPositionGyroscopeBiasJacobian_m_s := zeros(3, 3);
        imuAccumulatedPositionAccelerometerBiasJacobian_s2 := zeros(3, 3);
      else
        imuPacketDeltaAngle_rad := pre(imuPacketDeltaAngle_rad);
        imuPacketDeltaVelocity_m_s := pre(imuPacketDeltaVelocity_m_s);
        imuPacketDeltaPosition_m := pre(imuPacketDeltaPosition_m);
        imuPacketDeltaQuaternion := pre(imuPacketDeltaQuaternion);
        imuPacketGyroscopeBiasLinearization_rad_s :=
          pre(imuPacketGyroscopeBiasLinearization_rad_s);
        imuPacketAccelerometerBiasLinearization_m_s2 :=
          pre(imuPacketAccelerometerBiasLinearization_m_s2);
        imuPacketRotationGyroscopeBiasJacobian_s :=
          pre(imuPacketRotationGyroscopeBiasJacobian_s);
        imuPacketVelocityGyroscopeBiasJacobian_m :=
          pre(imuPacketVelocityGyroscopeBiasJacobian_m);
        imuPacketVelocityAccelerometerBiasJacobian_s :=
          pre(imuPacketVelocityAccelerometerBiasJacobian_s);
        imuPacketPositionGyroscopeBiasJacobian_m_s :=
          pre(imuPacketPositionGyroscopeBiasJacobian_m_s);
        imuPacketPositionAccelerometerBiasJacobian_s2 :=
          pre(imuPacketPositionAccelerometerBiasJacobian_s2);
        imuPacketIntegrationTime_s := pre(imuPacketIntegrationTime_s);
        imuPacketTimestamp_s := pre(imuPacketTimestamp_s);
        imuAccumulatedQuaternion := imuUpdatedQuaternion;
        imuAccumulatedDeltaVelocity_m_s := imuUpdatedDeltaVelocity_m_s;
        imuAccumulatedDeltaPosition_m := imuUpdatedDeltaPosition_m;
        imuAccumulatedTime_s := pre(imuAccumulatedTime_s) + imuSamplePeriod;
        imuAccumulatedGyroscopeBiasLinearization_rad_s :=
          pre(imuAccumulatedGyroscopeBiasLinearization_rad_s);
        imuAccumulatedAccelerometerBiasLinearization_m_s2 :=
          pre(imuAccumulatedAccelerometerBiasLinearization_m_s2);
        imuAccumulatedRotationGyroscopeBiasJacobian_s :=
          imuUpdatedRotationGyroscopeBiasJacobian_s;
        imuAccumulatedVelocityGyroscopeBiasJacobian_m :=
          imuUpdatedVelocityGyroscopeBiasJacobian_m;
        imuAccumulatedVelocityAccelerometerBiasJacobian_s :=
          imuUpdatedVelocityAccelerometerBiasJacobian_s;
        imuAccumulatedPositionGyroscopeBiasJacobian_m_s :=
          imuUpdatedPositionGyroscopeBiasJacobian_m_s;
        imuAccumulatedPositionAccelerometerBiasJacobian_s2 :=
          imuUpdatedPositionAccelerometerBiasJacobian_s2;
      end if;
    end if;
    // The sample closing this interval opens the next one; the first tick
    // seeds the history so the first-order hold starts from a zero slope.
    imuPreviousMeasuredAngularVelocity_rad_s :=
      imuMeasuredAngularVelocity_rad_s;
    imuPreviousMeasuredSpecificForce_m_s2 := imuMeasuredSpecificForce_m_s2;
    for i in 1:3 loop
      imuGyroscopeBias_rad_s[i] := if enableSensorNoise then
        pre(imuGyroscopeBias_rad_s[i])
          + sqrt(estimator.processNoise.gyroscopeBias_rad2_s3[i, i]
            * imuSamplePeriod) * imuGyroscopeBiasDrivingNoise[i] else 0.0;
      imuAccelerometerBias_m_s2[i] := if enableSensorNoise then
        pre(imuAccelerometerBias_m_s2[i])
          + sqrt(estimator.processNoise.accelerometerBias_m2_s5[i, i]
            * imuSamplePeriod) * imuAccelerometerBiasDrivingNoise[i] else 0.0;
    end for;
  end when;

algorithm
  // Ticks with the estimator and holds its freshly published bias estimate;
  // consumers read the previous step's value through pre() of this mirror.
  when sample(0.0, estimatorSamplePeriod) then
    estimatorGyroscopeBiasMirror_rad_s :=
      estimator.gyroscopeBiasBodyFlu_rad_s;
    estimatorAccelerometerBiasMirror_m_s2 :=
      estimator.accelerometerBiasBodyFlu_m_s2;
  end when;

  annotation(Documentation(info = "<html>
    <p>Reusable closed-loop integration of the RDD2 physical plant and
    <code>AvionicsSystem</code>. It supplies a fixed-capacity waypoint
    message and exposes flight observations; qualification models only need to
    select the route, the navigation source, and experiment limits.</p>
    <p><code>navigationSource</code> selects a plant-truth comparison baseline,
    GPS-aided inertial-estimator feedback, or optical-flow-aided inertial
    estimator feedback. Qualification flies all three so a controller/plant regression
    can be distinguished from an estimator or aiding regression. The
    deterministic-seeded Gaussian sensor noise and estimator initial offset
    make the feedback paths observable. Noise can be disabled, and its scaling
    uses the same covariance matrices supplied to the selected filter.</p>
  </html>"));
end WaypointVehicleSystem;
