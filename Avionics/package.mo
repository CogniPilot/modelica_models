within;

package Avionics
  "Pure-Modelica sensor and navigation contracts"

  record ImuSample
    "Calibrated inertial packet in body Forward-Left-Up axes"
    Boolean valid "True when the sample can be used";
    Boolean fresh "True for one estimator tick per new sample";
    Real timestamp_s(unit = "s")
      "End timestamp of the integration interval";
    Real angularVelocityBodyFlu_rad_s[3](each unit = "rad/s");
    Real specificForceBodyFlu_m_s2[3](each unit = "m/s2")
      "Accelerometer specific force; stationary level value is +g on body z";
    Real deltaAngleBodyFlu_rad[3](each unit = "rad")
      "Coning-corrected angle increment over integrationTime_s";
    Real deltaVelocityBodyFlu_m_s[3](each unit = "m/s")
      "Sculling-corrected specific-force increment, expressed in the body frame at the start of integration";
    Real deltaPositionBodyFlu_m[3](each unit = "m")
      "Closed-form position increment expressed in the body frame at the start of integration";
    Real deltaQuaternionBodyFlu[4](each unit = "1")
      "Scalar-first relative rotation from the start body frame to the end body frame";
    Real integrationTime_s(unit = "s")
      "Accumulation interval for delta angle and delta velocity";
    Real gyroscopeBiasLinearizationBodyFlu_rad_s[3](each unit = "rad/s")
      "Gyroscope-bias anchor used to form this preintegral";
    Real accelerometerBiasLinearizationBodyFlu_m_s2[3](each unit = "m/s2")
      "Accelerometer-bias anchor used to form this preintegral";
    Real deltaRotationGyroscopeBiasJacobian_s[3, 3](each unit = "s")
      "Right-local rotation-increment sensitivity to gyroscope bias";
    Real deltaVelocityGyroscopeBiasJacobian_m[3, 3](each unit = "m")
      "Velocity-increment sensitivity to gyroscope bias";
    Real deltaVelocityAccelerometerBiasJacobian_s[3, 3](each unit = "s")
      "Velocity-increment sensitivity to accelerometer bias";
    Real deltaPositionGyroscopeBiasJacobian_m_s[3, 3](each unit = "m.s")
      "Position-increment sensitivity to gyroscope bias";
    Real deltaPositionAccelerometerBiasJacobian_s2[3, 3](each unit = "s2")
      "Position-increment sensitivity to accelerometer bias";
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

  record MagnetometerSample
    "Calibrated three-axis magnetic field in body FLU axes"
    Boolean valid;
    Boolean fresh;
    Real timestamp_s(unit = "s");
    Real magneticFieldBodyFlu_T[3](each unit = "T");
    Real covarianceBody_T2[3, 3](each unit = "T2");
  end MagnetometerSample;

  record BarometerSample
    "Pressure-derived altitude measurement in the local ENU frame"
    Boolean valid;
    Boolean fresh;
    Real timestamp_s(unit = "s");
    Real altitudeWorldEnu_m(unit = "m")
      "Calibrated pressure altitude relative to the local-frame origin";
    Real variance_m2(unit = "m2");
  end BarometerSample;

  record OpticalFlowSample
    "Raw integrated optical flow with co-timed gyro and downward range"
    Boolean valid;
    Boolean fresh;
    Real timestamp_s(unit = "s");
    Real integratedLineOfSight_rad[2](each unit = "rad")
      "Right-handed scene rotations about camera/body FLU x and y over integrationTime_s. For a nadir camera, forward motion produces +y and left motion produces -x after rotation compensation";
    Real integratedLineOfSightCovariance_rad2[2, 2](each unit = "rad2");
    Real integratedGyroscopeBodyFlu_rad[3](each unit = "rad")
      "Actual FLU body-angle integral over the same exposure interval; add x/y to line-of-sight flow to remove camera rotation";
    Real integratedGyroscopeCovariance_rad2[3, 3](each unit = "rad2");
    Real integrationTime_s(unit = "s");
    Real groundDistance_m(unit = "m");
    Real groundDistanceVariance_m2(unit = "m2")
      "Variance of the range along the camera boresight";
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
    Boolean magnetometerCorrectionAccepted;
    Boolean barometerCorrectionAccepted;
    Boolean terrainCorrectionAccepted
      "A fresh downward-range sample updated the terrain-altitude filter";
    Boolean opticalFlowCorrectionAccepted;
    Integer consecutiveRejectedCorrections
      "Consecutive attempted aiding corrections rejected by conditioning
       or innovation gating; zero after any accepted correction";
    Real rejectionElapsed_s(unit = "s")
      "Wall-clock time the anchor source has held without moving the
       state. Advanced on ESTIMATOR TICKS, not on aiding attempts and not
       on the sensors` valid duty cycle, so it means the same elapsed time
       whatever rate the block is wired at and whatever waveform the
       drivers put on `valid`. Cleared when the anchor accepts, when the
       anchor changes, or when there is no anchor at all";
    Integer mocapConsecutiveRejections
      "Consecutive rejections of the mocap source alone";
    Integer gpsConsecutiveRejections
      "Consecutive rejections of the GPS source alone";
    Integer opticalFlowConsecutiveRejections
      "Consecutive rejections of the optical-flow source alone. The
       per-source counters exist because the aggregate above cannot show
       a failed sensor while a healthy one keeps resetting it: a source
       rejected on every one of its own samples is visible here from the
       first sample, whatever the other sources are doing";
    Integer correctionOutcome
      "Outcome of this tick's attempted aiding correction: 0 none
       attempted, 1 accepted, 2 rejected as not finite, 3 rejected by the
       innovation gate, 4 rejected because the innovation covariance did
       not factor, 5 rejected because the sensor-reported measurement
       covariance was not finite or claimed non-positive noise. Every
       rejection cause is named rather than inferred from the absence of
       other flags";
    Integer acceptedCorrectionCount
      "Monotonic count of SHIFTED FUSION INSTANTS since the last reset: the
       number of estimator ticks on which at least one aiding correction was
       accepted. AT MOST ONE PER ESTIMATOR TICK, and that is a contract on
       this field rather than a property of any particular filter.

       It counts instants and not measurements deliberately. The consumer this
       field exists for is an output predictor, which must recompose its
       buffered window once per tick on which the state it composes onto
       MOVED; two measurements fused into the same tick move it once, and
       counting them twice would ask for a second recomposition that has
       nothing to recompose.

       Both shipped filters satisfy the contract by construction rather than
       by arithmetic, because each fuses at most one source per tick from a
       priority chain, so their per-tick outcome is a single value. A filter
       that fused several sources in one tick would have to coalesce them here
       and must not increment per measurement: at a delayed fusion horizon
       every ripe measurement is an accepted correction, and the aiding set of
       a small multirotor offers well over a hundred a second against a
       recomposition budget of single digits.

       correctionOutcome is a LEVEL: it stands for the whole estimator
       tick, which at a 100 Hz filter behind an 800 Hz consumer is
       eight consumer ticks. A consumer that must act once per
       correction cannot edge-detect that level -- back-to-back
       accepted corrections hold it true across the boundary and the
       second one is invisible -- so the boundary carries the count and
       the consumer compares it with the one it last saw. That is the
       only well-defined edge across a rate change, which is why it is
       here and not reconstructed downstream";
    Integer correctionSource
      "Aiding source the outcome above refers to: 0 none, 1 mocap, 2 GPS,
       3 optical flow, 4 magnetometer, 5 barometer";
    Real normalizedInnovationSquared
      "NIS of this tick's attempted aiding correction; zero if none was attempted";
    Integer recoveryStage
      "Automatic recovery ladder: 0 nominal, 1 covariance ramping toward
       the mission envelope with the state untouched, 2 anchor still
       divergent at the envelope.

       THIS IS THE DEGRADED-MODE SIGNAL. It is the field a consumer reads
       to decide that the navigation solution is no longer trustworthy
       enough for position-holding flight, and it is deliberately separate
       from NavigationEstimate.valid, which says only that the published
       numbers are numbers. The separation is not cosmetic: on the
       deployed stack the rate loop takes body rates from the estimator,
       so clearing valid clears RatesValid, zeroes the motors and latches
       the fault in every mode including ACRO. A degraded navigation
       solution must be able to demote the flight mode without stopping
       the vehicle flying, so stage 2 raises this field and leaves valid
       alone.

       The estimator never re-seeds its own state from an aiding stream it
       is rejecting; a deliberate commanded reset is the only path that
       re-seeds";
    Boolean imuPayloadHeld
      "True when this tick`s published IMU-derived outputs -- angular
       velocity, world acceleration and the estimate timestamp -- were held
       from an earlier sample because the current inertial reading was not
       finite.

       Those three fields are computed from the raw sample rather than from
       the filter state, so they bypass the state finiteness guard entirely;
       holding them is what stops a NaN reaching a consumer. This flag, not
       NavigationEstimate.valid, is the signal that it happened: dropping
       valid clears RatesValid and latches a motors-zero fault in every
       mode, so using it to report a transient inertial glitch would turn a
       recoverable sample dropout into a crash";
    Integer anchorSource
      "Source currently anchoring the solution, by the same Source* codes
       as correctionSource. The recovery ladder is timed on this source
       alone, and a change of anchor restarts that timing";
  end EstimatorStatus;

  connector ImuSampleInput = input Avionics.ImuSample;
  connector ImuSampleOutput = output Avionics.ImuSample;
  connector MocapSampleInput = input Avionics.MocapSample;
  connector MocapSampleOutput = output Avionics.MocapSample;
  connector GpsSampleInput = input Avionics.GpsSample;
  connector GpsSampleOutput = output Avionics.GpsSample;
  connector MagnetometerSampleInput = input Avionics.MagnetometerSample;
  connector MagnetometerSampleOutput = output Avionics.MagnetometerSample;
  connector BarometerSampleInput = input Avionics.BarometerSample;
  connector BarometerSampleOutput = output Avionics.BarometerSample;
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
    Avionics.MagnetometerSampleInput magnetometer;
    Avionics.BarometerSampleInput barometer;
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
      magnetometerCorrectionAccepted(start = false, fixed = true),
      barometerCorrectionAccepted(start = false, fixed = true),
      terrainCorrectionAccepted(start = false, fixed = true),
      opticalFlowCorrectionAccepted(start = false, fixed = true),
      consecutiveRejectedCorrections(start = 0, fixed = true),
      rejectionElapsed_s(start = 0.0, fixed = true),
      mocapConsecutiveRejections(start = 0, fixed = true),
      gpsConsecutiveRejections(start = 0, fixed = true),
      opticalFlowConsecutiveRejections(start = 0, fixed = true),
      correctionOutcome(start = 0, fixed = true),
      acceptedCorrectionCount(start = 0, fixed = true),
      correctionSource(start = 0, fixed = true),
      recoveryStage(start = 0, fixed = true),
      anchorSource(start = 0, fixed = true),
      imuPayloadHeld(start = false, fixed = true));
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
