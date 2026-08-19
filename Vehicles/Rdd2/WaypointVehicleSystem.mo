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
  parameter Real gpsSamplePeriod(unit = "s") = 0.02;
  parameter Real opticalFlowSamplePeriod(unit = "s") = 0.01;
  parameter Real gravity_m_s2 = 9.80665;
  parameter Boolean enableSensorNoise = true
    "Enable deterministic-seeded Gaussian measurement noise";
  parameter Real sensorNoiseSeed = 20260818.0;
  parameter Real gpsPositionCovarianceWorld_m2[3, 3] = identity(3) * 0.25;
  parameter Real gpsVelocityCovarianceWorld_m2_s2[3, 3] = identity(3) * 0.01;
  parameter Real opticalFlowVelocityCovarianceBody_m2_s2[2, 2] =
    identity(2) * 0.01;
  parameter Real opticalFlowGroundDistanceVariance_m2 = 0.0025;
  parameter Real opticalFlowGroundNormalWorldEnu[3] = {0.0, 0.0, 1.0}
    "Unit normal of the locally planar terrain seen by the nadir camera";
  parameter Real opticalFlowGroundPlaneOffset_m = 0.0
    "Plane offset d in normal' * position = d";
  parameter Real opticalFlowNormalizedImageRadius = 0.35
    "Half-width of the synthetic feature grid in normalized image coordinates";
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
  final parameter Real estimatorSamplePeriod(unit = "s") =
    if navigationSource == 1 then gpsSamplePeriod
    elseif navigationSource == 2 then opticalFlowSamplePeriod
    else guidancePeriod;

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
    opticalFlowGroundNormalWorldEnu=opticalFlowGroundNormalWorldEnu,
    opticalFlowGroundPlaneOffset_m=opticalFlowGroundPlaneOffset_m);

  output Boolean armed;
  output Real time_s;
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
  output Real opticalFlowVelocityNoise_m_s[2];
  output Real opticalFlowGroundDistanceNoise_m;
  output Real opticalFlowIdealVelocityBodyFlu_m_s[2];
  output Real opticalFlowIdealGroundDistance_m;
  output Boolean opticalFlowSurfaceVisible;
  output Real imuAngularVelocityNoise_rad_s[3];
  output Real imuSpecificForceNoise_m_s2[3];
  output Real imuGyroscopeBias_rad_s[3](each start = 0.0, each fixed = true);
  output Real imuAccelerometerBias_m_s2[3](each start = 0.0, each fixed = true);
  output Real imuAngularVelocityNoiseVariance_rad2_s2[3];
  output Real imuSpecificForceNoiseVariance_m2_s4[3];
  output Real imuGyroscopeBiasIncrementVariance_rad2_s2[3];
  output Real imuAccelerometerBiasIncrementVariance_m2_s4[3];

protected
  Real opticalFlowIntegratedLineOfSight_rad[2];
  Real opticalFlowSurfaceVisibility;

equation
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
  estimator.imu.valid = plant.imu.valid;
  estimator.imu.fresh = plant.imu.fresh;
  estimator.imu.timestamp_s = plant.imu.timestamp_s;
  estimator.imu.angularVelocityBodyFlu_rad_s =
    plant.imu.angularVelocityBodyFlu_rad_s
      + imuGyroscopeBias_rad_s + imuAngularVelocityNoise_rad_s;
  estimator.imu.specificForceBodyFlu_m_s2 =
    plant.imu.specificForceBodyFlu_m_s2
      + imuAccelerometerBias_m_s2 + imuSpecificForceNoise_m_s2;

  estimator.mocap.valid = false;
  estimator.mocap.fresh = false;
  estimator.mocap.timestamp_s = time;
  estimator.mocap.positionWorldEnu_m = zeros(3);
  estimator.mocap.quaternionWorldBody = {1.0, 0.0, 0.0, 0.0};
  estimator.mocap.positionCovarianceWorld_m2 = identity(3);
  estimator.mocap.attitudeCovarianceBody_rad2 = identity(3);

  estimator.gps.valid = navigationSource == 1;
  // The estimator itself executes on this sensor's period. Holding fresh
  // true between those releases therefore presents exactly one new sample
  // to each estimator tick without relying on cross-block event-pulse order.
  estimator.gps.fresh = navigationSource == 1;
  estimator.gps.positionValid = navigationSource == 1;
  estimator.gps.velocityValid = navigationSource == 1;
  estimator.gps.timestamp_s = time;
  estimator.gps.geodetic_deg_m = geodetic;
  estimator.gps.positionWorldEnu_m =
    plant.truth.positionWorldEnu_m + gpsPositionNoise_m;
  estimator.gps.velocityWorldEnu_m_s =
    plant.truth.velocityWorldEnu_m_s + gpsVelocityNoise_m_s;
  estimator.gps.positionCovarianceWorld_m2 = gpsPositionCovarianceWorld_m2;
  estimator.gps.velocityCovarianceWorld_m2_s2 =
    gpsVelocityCovarianceWorld_m2_s2;

  (opticalFlowIdealVelocityBodyFlu_m_s,
   opticalFlowIntegratedLineOfSight_rad,
   opticalFlowIdealGroundDistance_m,
   opticalFlowSurfaceVisibility) = Vehicles.Rdd2.simulateOpticalFlowPlane(
     plant.truth.positionWorldEnu_m,
     plant.truth.velocityWorldEnu_m_s,
     plant.truth.rotationWorldBody,
     plant.truth.angularVelocityBodyFlu_rad_s,
     opticalFlowGroundNormalWorldEnu,
     opticalFlowGroundPlaneOffset_m,
     opticalFlowSamplePeriod,
     opticalFlowNormalizedImageRadius);
  opticalFlowSurfaceVisible = opticalFlowSurfaceVisibility > 0.5;
  estimator.opticalFlow.valid = navigationSource == 2
    and opticalFlowSurfaceVisible;
  estimator.opticalFlow.fresh = navigationSource == 2
    and opticalFlowSurfaceVisible;
  estimator.opticalFlow.timestamp_s = time;
  estimator.opticalFlow.velocityBodyFlu_m_s =
    opticalFlowIdealVelocityBodyFlu_m_s + opticalFlowVelocityNoise_m_s;
  estimator.opticalFlow.velocityCovarianceBody_m2_s2 =
    opticalFlowVelocityCovarianceBody_m2_s2;
  estimator.opticalFlow.integratedLineOfSight_rad =
    opticalFlowIntegratedLineOfSight_rad;
  estimator.opticalFlow.integrationTime_s = opticalFlowSamplePeriod;
  estimator.opticalFlow.groundDistance_m =
    opticalFlowIdealGroundDistance_m + opticalFlowGroundDistanceNoise_m;
  estimator.opticalFlow.groundDistanceVariance_m2 =
    opticalFlowGroundDistanceVariance_m2;
  estimator.opticalFlow.quality = 1.0;

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
    avionics.navigation.valid = estimator.estimate.valid;
    avionics.navigation.timestamp_s = estimator.estimate.timestamp_s;
    avionics.navigation.positionWorldEnu_m =
      estimator.estimate.positionWorldEnu_m;
    avionics.navigation.velocityWorldEnu_m_s =
      estimator.estimate.velocityWorldEnu_m_s;
    avionics.navigation.accelerationWorldEnu_m_s2 =
      estimator.estimate.accelerationWorldEnu_m_s2;
    avionics.navigation.quaternionWorldBody =
      estimator.estimate.quaternionWorldBody;
    avionics.navigation.rotationWorldBody =
      estimator.estimate.rotationWorldBody;
    avionics.navigation.eulerRpy_rad = estimator.estimate.eulerRpy_rad;
    avionics.navigation.angularVelocityBodyFlu_rad_s =
      estimator.estimate.angularVelocityBodyFlu_rad_s;
    avionics.navigation.angularVelocityWorldEnu_rad_s =
      estimator.estimate.angularVelocityWorldEnu_rad_s;
  end if;

  time_s = time;
  estimatorUpdatePeriod_s = estimatorSamplePeriod;
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
      estimator.processNoise.gyroscope_rad2_s[i, i] / estimatorSamplePeriod;
    imuSpecificForceNoiseVariance_m2_s4[i] =
      estimator.processNoise.accelerometer_m2_s3[i, i] / estimatorSamplePeriod;
    imuGyroscopeBiasIncrementVariance_rad2_s2[i] =
      estimator.processNoise.gyroscopeBias_rad2_s3[i, i]
        * estimatorSamplePeriod;
    imuAccelerometerBiasIncrementVariance_m2_s4[i] =
      estimator.processNoise.accelerometerBias_m2_s5[i, i]
        * estimatorSamplePeriod;
    imuAngularVelocityNoise_rad_s[i] = if enableSensorNoise then
      sqrt(estimator.processNoise.gyroscope_rad2_s[i, i]
        / estimatorSamplePeriod)
        * Vehicles.Rdd2.standardNormalNoise(
          time / estimatorSamplePeriod, i + 8.0, sensorNoiseSeed) else 0.0;
    imuSpecificForceNoise_m_s2[i] = if enableSensorNoise then
      sqrt(estimator.processNoise.accelerometer_m2_s3[i, i]
        / estimatorSamplePeriod)
        * Vehicles.Rdd2.standardNormalNoise(
          time / estimatorSamplePeriod, i + 11.0, sensorNoiseSeed) else 0.0;
    gpsPositionNoise_m[i] = if enableSensorNoise then
      sqrt(gpsPositionCovarianceWorld_m2[i, i])
        * Vehicles.Rdd2.standardNormalNoise(
          time / gpsSamplePeriod, 1.0 * i, sensorNoiseSeed) else 0.0;
    gpsVelocityNoise_m_s[i] = if enableSensorNoise then
      sqrt(gpsVelocityCovarianceWorld_m2_s2[i, i])
        * Vehicles.Rdd2.standardNormalNoise(
          time / gpsSamplePeriod, i + 3.0, sensorNoiseSeed) else 0.0;
    assert(gpsPositionCovarianceWorld_m2[i, i] > 0.0
      and gpsVelocityCovarianceWorld_m2_s2[i, i] > 0.0,
      "GPS measurement covariance diagonal must be positive");
    for j in 1:3 loop
      if i <> j then
        assert(abs(gpsPositionCovarianceWorld_m2[i, j]) < 1.0e-15
          and abs(gpsVelocityCovarianceWorld_m2_s2[i, j]) < 1.0e-15,
          "GPS noise fixture currently requires diagonal filter covariance");
      end if;
    end for;
  end for;
  for i in 1:2 loop
    opticalFlowVelocityNoise_m_s[i] = if enableSensorNoise then
      sqrt(opticalFlowVelocityCovarianceBody_m2_s2[i, i])
        * Vehicles.Rdd2.standardNormalNoise(
          time / opticalFlowSamplePeriod, i + 6.0, sensorNoiseSeed) else 0.0;
    assert(opticalFlowVelocityCovarianceBody_m2_s2[i, i] > 0.0,
      "Optical-flow measurement covariance diagonal must be positive");
    for j in 1:2 loop
      if i <> j then
        assert(abs(opticalFlowVelocityCovarianceBody_m2_s2[i, j]) < 1.0e-15,
          "Optical-flow noise fixture currently requires diagonal filter covariance");
      end if;
    end for;
  end for;
  opticalFlowGroundDistanceNoise_m = if enableSensorNoise then
    sqrt(opticalFlowGroundDistanceVariance_m2)
      * Vehicles.Rdd2.standardNormalNoise(
        time / opticalFlowSamplePeriod, 20.0, sensorNoiseSeed) else 0.0;
  assert(opticalFlowGroundDistanceVariance_m2 > 0.0,
    "Optical-flow ground-distance variance must be positive");
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
  when sample(0.0, estimatorSamplePeriod) then
    for i in 1:3 loop
      imuGyroscopeBias_rad_s[i] := if enableSensorNoise then
        pre(imuGyroscopeBias_rad_s[i])
          + sqrt(estimator.processNoise.gyroscopeBias_rad2_s3[i, i]
            * estimatorSamplePeriod) * Vehicles.Rdd2.standardNormalNoise(
              time / estimatorSamplePeriod, i + 14.0, sensorNoiseSeed) else 0.0;
      imuAccelerometerBias_m_s2[i] := if enableSensorNoise then
        pre(imuAccelerometerBias_m_s2[i])
          + sqrt(estimator.processNoise.accelerometerBias_m2_s5[i, i]
            * estimatorSamplePeriod) * Vehicles.Rdd2.standardNormalNoise(
              time / estimatorSamplePeriod, i + 17.0, sensorNoiseSeed) else 0.0;
    end for;
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
