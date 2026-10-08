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
  constant Integer CorrectionRejectedTimestamp = 6;
  // Reported on the tick the automatic recovery ladder re-seeds the state
  // from a fresh anchor sample after a sustained rejection window. It is a
  // deliberate replacement of the state, not an ordinary gated fusion, so it
  // is named rather than reported as CorrectionAccepted.
  constant Integer CorrectionReseeded = 7;
  constant Integer SourceNone = 0;
  constant Integer SourceMocap = 1;
  constant Integer SourceGps = 2;
  constant Integer SourceOpticalFlow = 3;
  constant Integer SourceMagnetometer = 4;
  constant Integer SourceBarometer = 5;
  constant Integer SourcePseudoPosition = 6
    "Synthetic hold-position measurement fused while no anchor source is
     live, so an unaided filter cannot integrate a tilt error into an
     unbounded velocity. Never an anchor and never counted as an accepted
     aiding correction";
  constant Integer SourceZeroVelocity = 7
    "Synthetic zero-velocity measurement fused while the IMU is quasi-static
     and no anchor source is live. Same standing as SourcePseudoPosition";
  constant Integer AlignmentNone = 0
    "No initial attitude alignment has been performed yet";
  constant Integer AlignmentMocap = 1
    "Initial attitude taken from a motion-capture quaternion";
  constant Integer AlignmentAccelerometerMagnetometer = 2
    "Initial attitude leveled from the specific force with heading from the
     magnetometer";
  constant Integer AlignmentAccelerometer = 3
    "Initial attitude leveled from the specific force with heading left at
     zero, because no magnetometer sample was usable";
  constant Integer AlignmentFallback = 4
    "Initial attitude taken from the configured initial quaternion because no
     usable sample was available";
  constant Integer RecoveryNominal = 0;
  constant Integer RecoveryCovarianceInflated = 1;
  constant Integer RecoveryAidingDivergent = 2;
  constant Integer RecoveryMisconfigured = 3;

  annotation(Documentation(info = "<html>
    <h4>Choose a filter</h4>
    <p><a href=\"modelica://Estimation.StrapdownINS.ESKF.Estimator\">ESKF.Estimator</a>
    implements the 15-state right-error filter;
    <a href=\"modelica://Estimation.StrapdownINS.UKF.Estimator\">UKF.Estimator</a>
    implements a manifold UKF. Both use
    <a href=\"modelica://Estimation.StrapdownINS.PartialEstimator\">PartialEstimator</a>
    and publish <a href=\"modelica://Avionics.NavigationEstimate\">NavigationEstimate</a>.</p>
    <h4>Deliver sensor samples</h4>
    <p>Use world ENU and body FLU coordinates. Set sensor timestamps to capture
    time in the estimator clock; hold <code>valid</code> while a sample is usable
    and pulse <code>fresh</code> for one estimator tick per new arrival. Supply
    covariance matching the actual sensor noise. A held sample must not be
    fused repeatedly as independent data.</p>
    <h4>Delayed aiding</h4>
    <p>The direct ESKF retrodicts with a held IMU input; see
    <a href=\"modelica://Estimation.StrapdownINS.ESKF.correctGps\">correctGps</a>.
    <a href=\"modelica://Estimation.FusionHorizon.HorizonEstimator\">HorizonEstimator</a>
    instead buffers measurements, filters in the past and predicts a separate
    current-time output. Pair covariance diagnostics with state and truth at
    the same epoch and in the same tangent convention.</p>
    <p>This namespace groups alternative estimators for the same navigation
    problem: attitude, velocity, position, gyroscope bias, and accelerometer
    bias from an IMU plus external aiding. Algorithm names are nested below
    the problem namespace so implementations can be compared without claiming
    that every filter has the same mathematical structure.</p>

    <h4>References</h4>
    <p>Published sources for the preintegration mathematics implemented in
    this package. Each site names the result it uses.</p>
    <ol>
    <li>L.-Y. Lin, K. A. Pant, B. Perseghetti, and J. Goppert, \"On Closed-Form
    Preintegration for a Class of Mixed-Invariant Systems in SE_n(3),\"
    <i>IEEE Control Systems Letters</i>, 2025 (Purdue e-Pubs, School of
    Aeronautics and Astronautics Faculty Publications, Paper 61,
    <a href=\"https://docs.lib.purdue.edu/aaepubs/61\">
    https://docs.lib.purdue.edu/aaepubs/61</a>). The zero-order-hold closed
    form that <code>LieGroups.SE23.Quat.exp_mixed</code> evaluates and that
    <code>preintegrateImuStep</code> composes.</li>
    <li>H. Reynolds, M. K. Condie, B. Perseghetti, and J. Goppert,
    \"First-Order-Hold Magnus Preintegration on SE_n(3) with Computable
    Flow-Error Bounds,\" ACC 2027 manuscript, 2026. The
    first-order-hold results this package implements: the FOH preintegration
    theorem (the third-order truncated Magnus exponent and its O(T^5) residual,
    with no T^4 term), the bracket-decomposition proposition (the single Lie
    bracket splits into the classical coning, sculling, and scrolling
    corrections), and the exponent bias sensitivities. Physical-increment
    Jacobians apply the chain rule through the closed-form mixed exponential
    and through composition. The current implementation does not evaluate the
    manuscript's flow-error certificate.</li>
    <li>W. Magnus, \"On the exponential solution of differential equations for
    a linear operator,\" <i>Communications on Pure and Applied Mathematics</i>,
    vol. 7, no. 4, pp. 649-673, 1954. The expansion the first-order-hold
    increment truncates. Survey: S. Blanes, F. Casas, J. A. Oteo, and J. Ros,
    <i>Physics Reports</i>, vol. 470, pp. 151-238, 2009.</li>
    <li>C. Forster, L. Carlone, F. Dellaert, and D. Scaramuzza, \"On-Manifold
    Preintegration for Real-Time Visual-Inertial Odometry,\" <i>IEEE
    Transactions on Robotics</i>, vol. 33, no. 1, pp. 1-21, 2017,
    doi:10.1109/TRO.2016.2597321. The bias-anchor formulation: a preintegral
    is stored at a linearization bias and moved to an estimated bias by its
    first-order Jacobians, which is what <code>correctPreintegratedImu</code>
    does. Predecessor: T. Lupton and S. Sukkarieh, <i>IEEE Transactions on
    Robotics</i>, vol. 28, no. 1, pp. 61-76, 2012.</li>
    <li>J. E. Bortz, \"A new mathematical formulation for strapdown inertial
    navigation,\" <i>IEEE Transactions on Aerospace and Electronic Systems</i>,
    vol. AES-7, no. 1, pp. 61-66, 1971; P. G. Savage, \"Strapdown inertial
    navigation integration algorithm design,\" parts 1 and 2, <i>Journal of
    Guidance, Control, and Dynamics</i>, vol. 21, nos. 1-2, 1998. The classical
    coning and sculling corrections that the bracket reproduces.</li>
    <li>T. D. Barfoot, <i>State Estimation for Robotics</i>, Cambridge
    University Press, 2017, Section 9.4. The right-perturbation error
    convention and inertial error dynamics used by
    <code>StrapdownINS.ESKF</code>.</li>
    </ol>
  </html>"));
end StrapdownINS;
