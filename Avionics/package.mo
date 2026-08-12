within;

package Avionics
  "Pure-Modelica sensor and navigation contracts"

  record ImuSample
    "Calibrated inertial sample in body Forward-Left-Up axes"
    Boolean valid "True when the sample can be used";
    Boolean fresh "True for one estimator tick per new sample";
    Real timestamp_s(unit = "s");
    Real angularVelocityBodyFlu_rad_s[3](each unit = "rad/s");
    Real specificForceBodyFlu_m_s2[3](each unit = "m/s2")
      "Accelerometer specific force; stationary level value is +g on body z";
  end ImuSample;

  record MocapSample
    "Motion-capture pose in the local East-North-Up world frame"
    Boolean valid;
    Boolean fresh;
    Real timestamp_s(unit = "s");
    Real positionWorldEnu_m[3](each unit = "m");
    Real quaternionWorldBody[4](each unit = "1")
      "Scalar-first Hamilton quaternion {w,x,y,z}, body FLU to world ENU";
    Real positionCovarianceWorld_m2[3, 3](each unit = "m2");
    Real attitudeCovarianceBody_rad2[3, 3](each unit = "rad2")
      "Small-angle covariance expressed in body FLU";
  end MocapSample;

  record GpsSample
    "GPS solution mapped into the estimator local frame"
    Boolean valid;
    Boolean fresh;
    Boolean positionValid;
    Boolean velocityValid;
    Real timestamp_s(unit = "s");
    Real geodetic_deg_m[3]
      "Optional source coordinates {latitude_deg,longitude_deg,altitude_m}";
    Real positionWorldEnu_m[3](each unit = "m");
    Real velocityWorldEnu_m_s[3](each unit = "m/s");
    Real positionCovarianceWorld_m2[3, 3](each unit = "m2");
    Real velocityCovarianceWorld_m2_s2[3, 3](each unit = "m2/s2");
  end GpsSample;

  record OpticalFlowSample
    "Optical-flow odometry mapped to planar body-frame velocity"
    Boolean valid;
    Boolean fresh;
    Real timestamp_s(unit = "s");
    Real velocityBodyFlu_m_s[2](each unit = "m/s")
      "Planar {forward,left} velocity after camera/range compensation";
    Real velocityCovarianceBody_m2_s2[2, 2](each unit = "m2/s2");
    Real integratedLineOfSight_rad[2](each unit = "rad")
      "Optional raw flow integral retained for transport adapters";
    Real integrationTime_s(unit = "s");
    Real groundDistance_m(unit = "m");
    Real quality(min = 0.0, max = 1.0)
      "Normalized driver quality indicator";
  end OpticalFlowSample;

  record NavigationEstimate
    "Estimator-independent state consumed by guidance and control"
    Boolean valid;
    Real timestamp_s(unit = "s");
    Real positionWorldEnu_m[3](each unit = "m");
    Real velocityWorldEnu_m_s[3](each unit = "m/s");
    Real accelerationWorldEnu_m_s2[3](each unit = "m/s2");
    Real quaternionWorldBody[4](each unit = "1")
      "Scalar-first Hamilton quaternion {w,x,y,z}, body FLU to world ENU";
    Real rotationWorldBody[3, 3](each unit = "1")
      "Direction-cosine matrix mapping body FLU vectors into world ENU";
    Real eulerRpy_rad[3](each unit = "rad")
      "Intrinsic 3-2-1 attitude ordered {roll,pitch,yaw}";
    Real angularVelocityBodyFlu_rad_s[3](each unit = "rad/s");
    Real angularVelocityWorldEnu_rad_s[3](each unit = "rad/s");
  end NavigationEstimate;

  record EstimatorStatus
    "Algorithm-neutral estimator health and correction status"
    Boolean initialized;
    Boolean predictionAccepted;
    Boolean mocapCorrectionAccepted;
    Boolean gpsPositionCorrectionAccepted;
    Boolean gpsVelocityCorrectionAccepted;
    Boolean opticalFlowCorrectionAccepted;
    Integer consecutiveRejectedCorrections
      "Consecutive attempted aiding corrections rejected by conditioning
       or innovation gating; zero after any accepted correction";
    Boolean covarianceReinitialized
      "True on a tick where persistent rejection forced the estimator to
       re-run its declared initialization policy";
    Boolean innovationGateRejected
      "True when the most recent attempted correction was rejected by the
       chi-square innovation gate rather than by factorization failure";
  end EstimatorStatus;

  connector ImuSampleInput = input Avionics.ImuSample;
  connector ImuSampleOutput = output Avionics.ImuSample;
  connector MocapSampleInput = input Avionics.MocapSample;
  connector MocapSampleOutput = output Avionics.MocapSample;
  connector GpsSampleInput = input Avionics.GpsSample;
  connector GpsSampleOutput = output Avionics.GpsSample;
  connector OpticalFlowSampleInput = input Avionics.OpticalFlowSample;
  connector OpticalFlowSampleOutput = output Avionics.OpticalFlowSample;
  connector NavigationEstimateInput = input Avionics.NavigationEstimate;
  connector NavigationEstimateOutput = output Avionics.NavigationEstimate;
  connector EstimatorStatusOutput = output Avionics.EstimatorStatus;

  partial block PartialNavigationEstimator
    "Stable boundary implemented by interchangeable navigation algorithms"
    parameter Real samplePeriod(unit = "s", min = 1.0e-9) = 0.001;
    input Boolean reset;
    Avionics.ImuSampleInput imu;
    Avionics.MocapSampleInput mocap;
    Avionics.GpsSampleInput gps;
    Avionics.OpticalFlowSampleInput opticalFlow;
    discrete Avionics.NavigationEstimateOutput estimate(
      valid(start = false, fixed = true),
      timestamp_s(start = 0.0, fixed = true),
      positionWorldEnu_m(each start = 0.0, each fixed = true),
      velocityWorldEnu_m_s(each start = 0.0, each fixed = true),
      accelerationWorldEnu_m_s2(each start = 0.0, each fixed = true),
      quaternionWorldBody(
        start = {1.0, 0.0, 0.0, 0.0}, each fixed = true),
      rotationWorldBody(
        start = [1.0, 0.0, 0.0;
                 0.0, 1.0, 0.0;
                 0.0, 0.0, 1.0], each fixed = true),
      eulerRpy_rad(each start = 0.0, each fixed = true),
      angularVelocityBodyFlu_rad_s(each start = 0.0, each fixed = true),
      angularVelocityWorldEnu_rad_s(each start = 0.0, each fixed = true));
    discrete Avionics.EstimatorStatusOutput status(
      initialized(start = false, fixed = true),
      predictionAccepted(start = false, fixed = true),
      mocapCorrectionAccepted(start = false, fixed = true),
      gpsPositionCorrectionAccepted(start = false, fixed = true),
      gpsVelocityCorrectionAccepted(start = false, fixed = true),
      opticalFlowCorrectionAccepted(start = false, fixed = true),
      consecutiveRejectedCorrections(start = 0, fixed = true),
      covarianceReinitialized(start = false, fixed = true),
      innovationGateRejected(start = false, fixed = true));
  end PartialNavigationEstimator;

  annotation(Documentation(info = "<html>
    <p>These records are the dependency boundary between transport drivers,
    estimators, and controllers. They intentionally contain no FlatBuffers or
    Synapse types. A driver maps a transport message into these physical,
    frame-explicit records; every estimator implements
    <code>PartialNavigationEstimator</code>; every controller consumes
    <code>NavigationEstimate</code>.</p>
    <p>The canonical world/body convention is ENU/FLU. Redundant attitude
    representations are deliberate at this boundary: the estimator guarantees
    their consistency so consumers never depend on its internal state
    representation. Filter covariance, bias states, and tangent ordering remain
    private because they have no unambiguous meaning across algorithms.</p>
  </html>"));
end Avionics;
