within Estimation.StrapdownINS;

package ESKF
  "15-state aided strapdown inertial-navigation error-state Kalman filter"
  constant Integer TangentLength = 15;
  constant Integer ProcessNoiseLength = 12;

  constant Real RecoveryEnvelope_m = 100.0;

  // Magnitude above which a Real is treated as not finite. Exactly
  // representable in binary32 (max ~3.4e38) and binary64, so the same
  // source means the same thing in simulation and in generated flight
  // code, and 1e30 is ~20 orders of magnitude beyond any residual,
  // position, or velocity this filter can legitimately see. The test
  // `abs(x) < FiniteMagnitudeLimit` is false for NaN and for +/-Inf
  // alike, so one comparison per element covers both.
  constant Real FiniteMagnitudeLimit = 1.0e30;

  constant Real MinimumSeedQuaternionNorm = 1.0e-3;

  constant Real MaxAttitudeCorrection_rad = 0.15;

  type TangentVector = Real[15]
    "{position,velocity,attitude,gyro bias,accelerometer bias} error";
  type Covariance = Real[15, 15];
  annotation(Documentation(info = "<html>
    <p>The nominal extended pose is propagated by
    <code>LieGroups.SE23.Quat.exp_mixed</code>. Covariance lives in a
    body-local right-perturbation error ordered as position, velocity,
    attitude, gyroscope bias, and accelerometer bias.</p>
    <p>Over one IMU interval the corrected IMU input is held constant. The
    locally linearized error transition is discretized with a third-order matrix
    exponential polynomial; the process-noise integral is evaluated by
    Simpson quadrature. Both operations retain full cross-axis covariance and
    use matrix expressions rather than three scalar filters.</p>
    <p>This is an aided strapdown INS ESKF, not a strict invariant Kalman
    filter. The additive bias states are not part of a group-affine, exactly
    log-linear augmented system. The local pose error and manifold retraction
    are geometric implementation choices inside an otherwise ordinary
    error-state EKF.</p>
    <p>The convention and inertial error dynamics follow T. D. Barfoot,
    <a href=\"https://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser24.pdf\">
    <i>State Estimation for Robotics</i>, 2nd-edition draft</a>, Section 9.4:
    the nominal state is right-perturbed, so the local error is
    <code>T_hat^-1 T</code>. In the tangent ordering used here, Barfoot's
    position, velocity, attitude, gyro-bias, and accelerometer-bias blocks map
    directly to <code>continuousTransition</code> and
    <code>noiseInputMatrix</code>. Prediction implements
    <code>P+ = Phi P Phi' + integral(Phi G Q G' Phi')</code>. Correction uses
    an arbitrary-gain Joseph update followed by the right-Jacobian reset that
    transports covariance to the newly injected nominal-state tangent.</p>
  </html>"));
end ESKF;
