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
  constant Integer SourceNone = 0;
  constant Integer SourceMocap = 1;
  constant Integer SourceGps = 2;
  constant Integer SourceOpticalFlow = 3;
  constant Integer SourceMagnetometer = 4;
  constant Integer SourceBarometer = 5;
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
    <li>L.-Y. Lin, K. A. Pant, B. Perseghetti, and J. Goppert, \"An Exact Error
    Theory for Mixed-Invariant Preintegration on SE_2(3): Proved Truncation
    Residuals, a Sufficiency Bound for Deployed Coning Algorithms, and a
    Delayed-Fusion Error-State Filter,\" manuscript in preparation, 2026. The
    first-order-hold results this package implements: the FOH preintegration
    theorem (the third-order truncated Magnus exponent and its O(T^5) residual,
    with no T^4 term), the bracket-decomposition proposition (the single Lie
    bracket splits into the classical coning, sculling, and scrolling
    corrections), and the FOH bias-sensitivity proposition (one dt^2/12 cross
    term appended per Jacobian channel).</li>
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
