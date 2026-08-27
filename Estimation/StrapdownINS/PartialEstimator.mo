within Estimation.StrapdownINS;

partial block PartialEstimator
  "Common replaceable interface for strapdown navigation estimators"
  extends Avionics.PartialNavigationEstimator;

  parameter Real gravityWorldEnu_m_s2[3] = {0.0, 0.0, -9.81};
  parameter Real initialPositionWorldEnu_m[3] = zeros(3);
  parameter Real initialVelocityWorldEnu_m_s[3] = zeros(3);
  parameter Real initialQuaternionWorldBody[4] = {1.0, 0.0, 0.0, 0.0};
  parameter Real initialGyroscopeBiasBodyFlu_rad_s[3] = zeros(3);
  parameter Real initialAccelerometerBiasBodyFlu_m_s2[3] = zeros(3);
  parameter Estimation.StrapdownINS.InitialVariances initialVariances =
    Estimation.StrapdownINS.InitialVariances(
      position_m2=fill(1.0, 3),
      velocity_m2_s2=fill(1.0, 3),
      attitude_rad2=fill(0.25, 3),
      gyroscopeBias_rad2_s2=fill(1.0e-4, 3),
      accelerometerBias_m2_s4=fill(1.0e-2, 3));
  parameter Estimation.StrapdownINS.ProcessNoise processNoise =
    Estimation.StrapdownINS.ProcessNoise(
      gyroscope_rad2_s=identity(3) * 1.0e-5,
      accelerometer_m2_s3=identity(3) * 1.0e-3,
      gyroscopeBias_rad2_s3=identity(3) * 1.0e-8,
      accelerometerBias_m2_s5=identity(3) * 1.0e-6);
  parameter Real opticalFlowGroundNormalWorldEnu[3] = {0.0, 0.0, 1.0}
    "Unit normal of the locally planar surface observed by the nadir camera";
  parameter Real opticalFlowGroundPlaneOffset_m = 0.0
    "Plane offset d in normal' * position = d";
  parameter Real localMagneticFieldWorldEnu_T[3] =
    {-1.59e-6, 20.04e-6, -47.91e-6}
    "Local geomagnetic field reference used only for yaw";
  // ENU, so the components are {east, north, up} and the horizontal pair is
  // what sets declination. The previous default {18.0e-6, 4.0e-6, -47.0e-6}
  // had east and north transposed: it declares a field pointing 77.5 degrees
  // east of north, which occurs nowhere outside the polar regions, and yaw is
  // taken directly from this reference, so every heading was biased by the
  // difference. Only the horizontal pair was wrong, and inclination looked
  // right either way, which is what let it stand.
  //
  // These are WMM2025 evaluated at the RDD2 reference origin (40.4237 N,
  // 86.9212 W, 2026.6): declination -4.5 degrees, inclination 67.2 degrees,
  // magnitude 52.0 uT. A deployment away from that site must supply its own
  // reference, ideally by evaluating Geodesy.WMM2025.magneticFieldEnu at the
  // mission origin as Vehicles.Rdd2.WaypointVehicleSystem does, because no
  // fixed default is right everywhere.
  //
  // NOTE FOR TEST DESIGN: a simulated magnetometer synthesized from this same
  // vector cancels any error in it exactly, so no mission that generates its
  // measurement from the estimator's own reference can detect a wrong one.
  parameter Real maximumAidingDelay_s(unit = "s") = 0.25
    "Reject aiding whose timestamp is older than this fixed-lag horizon";
  parameter Real minimumOpticalFlowQuality = 0.2;
  parameter Real minimumOpticalFlowGroundDistance_m = 0.2
    "Reject flow/range velocity observations inside the range sensor near field";
  parameter Real initialBarometerBias_m = 0.0;
  parameter Real initialBarometerBiasVariance_m2 = 4.0;
  parameter Real barometerBiasProcessNoise_m2_s = 1.0e-4;
  parameter Integer barometerBiasCalibrationSamples(min = 1) = 25
    "Stationary pressure samples used to establish the local altitude datum";
  parameter Real initialTerrainAltitudeWorldEnu_m =
    opticalFlowGroundPlaneOffset_m;
  parameter Real initialTerrainVariance_m2 = 4.0;
  parameter Real terrainProcessNoise_m2_s = 1.0e-3;
  parameter Real minimumRangeCosTilt = 0.5
    "Reject downward range when the boresight is more than 60 degrees from vertical";
  output Real navigationCovarianceLocal[6, 6]
    "Position/velocity covariance in body-local tangent axes";
  output Real gyroscopeBiasBodyFlu_rad_s[3];
  output Real accelerometerBiasBodyFlu_m_s2[3];
  output Real barometerBias_m;
  output Real barometerBiasVariance_m2;
  output Real terrainAltitudeWorldEnu_m;
  output Real terrainAltitudeVariance_m2;
  output Real heightAboveTerrain_m;

  annotation(Documentation(info = "<html>
    <p>Every algorithm under <code>Estimation.StrapdownINS</code> implements this
    boundary. Inputs are time-stamped IMU, mocap, GPS, and optical-flow samples
    plus an explicit reset; outputs are the canonical ENU/FLU navigation
    estimate and algorithm-independent health status.</p>
    <p>Algorithm-specific covariance coordinates, bias representations, and
    diagnostics remain inside each implementation. That separation permits an
    ESKF, UKF, or another estimator to replace one another without pretending
    their internal uncertainty states have identical meanings.</p>
  </html>"));
end PartialEstimator;
