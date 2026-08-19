within Estimation;

package StrapdownINS
  "Estimators for strapdown inertial navigation with external aiding"
  type ProcessNoiseCovariance = Real[12, 12]
    "Continuous {gyro,accelerometer,gyro-bias walk,accelerometer-bias walk} noise covariance";

  // Algorithm-independent correction outcome and source codes exposed by
  // Avionics.EstimatorStatus. Keeping them at the common problem level lets
  // every PartialEstimator implementation report identical lifecycle data.
  constant Integer CorrectionNotAttempted = 0;
  constant Integer CorrectionAccepted = 1;
  constant Integer CorrectionRejectedNotFinite = 2;
  constant Integer CorrectionRejectedGate = 3;
  constant Integer CorrectionRejectedFactorization = 4;
  constant Integer CorrectionRejectedCovarianceUnusable = 5;
  constant Integer SourceNone = 0;
  constant Integer SourceMocap = 1;
  constant Integer SourceGps = 2;
  constant Integer SourceOpticalFlow = 3;
  constant Integer RecoveryNominal = 0;
  constant Integer RecoveryCovarianceInflated = 1;
  constant Integer RecoveryAidingDivergent = 2;
  constant Integer RecoveryMisconfigured = 3;

  annotation(Documentation(info = "<html>
    <p>This namespace groups alternative estimators for the same navigation
    problem: attitude, velocity, position, gyroscope bias, and accelerometer
    bias from an IMU plus external aiding. Algorithm names are nested below
    the problem namespace so implementations can be compared without claiming
    that every filter has the same mathematical structure.</p>
  </html>"));
end StrapdownINS;
