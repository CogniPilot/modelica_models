Sampled estimator error bounds using gnc_lean
8 October 2026

The delivered 42 theorems establish conditional error bounds and the precise
comparison obligations for ESKF, EKF2 and EKF3. They do not yet establish a
native-filter superiority result. No estimator tuning or default changes are
made by this proof package. The formal sources are GNC modules; copies here
retain the exact sources reviewed alongside the estimator experiments.

SampledErrorBound: GPS fusion, finite outage, recovery and publication

A regional update certificate has the form
  norm(F_k(e)) <= a_k*r + b_k*r^2 + c_k*r^3 + d_k, r = norm(e),
where all four coefficients are nonnegative. Starting from an initial error
bound, the polynomial recurrence produces a time-varying error envelope.
certified_tube proves trajectory containment by induction, provided the
computed envelope stays inside the declared validity region. Containment is
not assumed of the actual trajectory. contextual_tube makes the admissibility
of covariance, bias, sensor-policy state, buffers and inputs explicit. Gates
require valid bounds for accepted and rejected updates.

On a radius R, the polynomial has an affine majorant q*r+d, with
  q = a+b*R+c*R^2.
invariant_tube proves containment when q*R+d <= R. A recovery envelope has the
form u+q^n*(r-u), with d=(1-q)*u. When 0 <= q < 1, its limit is u. This is a
deterministic disturbance envelope, not a Gaussian MSE or Riccati theorem.
The contraction may need to be established for a complete aiding window;
individual IMU ticks need not contract a navigation error norm.

For an N-update GPS outage, growth may exceed one. The affine envelope is
  g^N*r+d*(g^N-1)/(g-1), g != 1,
and r+N*d for g=1. gps_outage_recovery connects a finite outage to returning
GPS. publication_bound charges the forward predictor's gain and disturbance
after fusion-horizon correction, so fusion-time bounds cannot be reported as
current-time accuracy without accounting for prediction.

sampled_map_from_derivatives derives a polynomial bound from actual first and
second derivatives along the entire error segment using GNC's Taylor theorem.
The required curvature bound is uniform along that segment. A Jacobian or
Hessian sampled only at a nominal point is insufficient. Discontinuous gate
and reset branches require separate certificates.

LieErrorEnvelope: exact SE_2(3) propagation and physical reconstruction

GNC's right error is truth*estimate^-1. Modelica ESKF injects a local correction
as estimate*Exp(local). local_to_right_error proves the conjugation identity
  Exp(Ad_estimate local) = estimate*Exp(local)*estimate^-1.
adjoint_envelope includes translation/attitude coupling. The conversion is
not an isometry of an arbitrary scaled navigation norm.

With matched physical bias and a valid logarithm chart, GNC's exact navigation
log equation reduces to
  position_dot = velocity, velocity_dot = gravity cross attitude, attitude_dot=0.
The exact finite-time solution and its uniqueness are proved. Its Euclidean
component bounds are
  position <= P+t*V+t^2*G*A/2,
  velocity <= V+t*G*A, attitude <= A.
These are navigation log components in metres, metres/second and radians.
local_physical_translation_bound and right_physical_translation_bound also
bound the actual Cartesian position/velocity differences. Reconstruction
uses the SO(3) Jacobian's nonexpansiveness in the valid chart; right-error
reconstruction retains the estimated-pose lever-arm terms.

The matched-bias special case is not used as a bound for the augmented biased
filter. biased_navigation_envelope retains the exact forcing
  tangentBias(error, Ad_estimate(trueBias-estimatedBias)).
Given uniform Euclidean component forcing bounds Wp, Wv, Wa over [0,T], it
proves the conservative finite-horizon envelope
  A_T = A+T*Wa,
  V_T = V+T*(G*A_T+Wv),
  P_T = P+T*(V_T+Wp).
Forcing bounds, chart validity, actual input/noise representation and reset
transitions remain certification obligations. Changing bias coordinates does
not make those terms disappear. The exact continuous dynamics do not by
themselves certify a Modelica discretization or generated floating-point code.

GpsDeniedGeometry: what the requested denied sensors can observe

For flat ground and a spatially uniform magnetic/gravity field, compensated
flow, nadir ray range, barometer and magnetic/inertial observations are
invariant under a constant horizontal position shift. The proof retains
tilted range, pressure datum and body-frame velocity. Translation is preserved
by inertial propagation, and entire ideal sensor histories are identical.
For two such indistinguishable states separated by horizontal distance D,
any common-history estimator has worst-case axis error at least D/2. This
requires initial horizontal uncertainty in the outage certificate. It does
not say velocity is unobservable or that finite-outage position is unbounded
when initial uncertainty and forcing are bounded. Terrain maps, landmarks,
spatial magnetic maps and a prior GPS history can supply additional information
and are outside the indistinguishable-state construction.

CorrectionErrorBound: deriving the sampled correction certificate

The correction is represented in a common physical norm as
  e_next = e - K(e)*(H*e + observationDefect(e) + noise(e)) + resetDefect(e).
The gain may depend on error/context, so a fixed nominal gain is not assumed.
Uniform bounds on ||e-K(e)*H*e|| <= a*||e||, ||K(e)|| <= k,
||observationDefect(e)|| <= b*||e||^2 and ||noise(e)|| <= nu,
with reset defect <= br*||e||^2+cr*||e||^3+epsilon, give StepBound
  linear=a, quadratic=k*b+br, cubic=cr, disturbance=k*nu+epsilon.
correction_map_bound proves this algebraic/norm bridge to the sampled tubes.
The bounds and the error-map representation remain implementation obligations;
covariance optimality, a fitted gain or pointwise finite differences do not
supply them. Noise may be error-correlated inside the uniformly bounded event.
A state-dependent gain constraint, delayed residual or Lie reset must be
represented and charged in the certified map rather than silently omitted.

An unobserved nonzero direction H*e=0 is preserved by every linear correction
I-K*H, so its norm cannot contract by a factor less than one. This gives a
specific check against assigning fictitious full-state contraction to each
flow or other partial measurement update. A complete observable aiding window
may admit a contraction certificate; it must be derived separately.

bounded_implementation_defect transfers a reference StepBound to an actual
map only if a uniform numerical/implementation error epsilon is supplied.
Its disturbance becomes d+epsilon. This makes IEEE754/compiler error an
explicit pending obligation rather than treating exact-real Lean results as
a generated-code refinement proof.

What would constitute superiority

ordered_envelopes proves ordering of certified upper envelopes. That alone
does not order actual estimator errors. strict_local_remainder_advantage
proves the quadratic-versus-cubic bound advantage only with common linear and
disturbance terms and c*r < b. Neither theorem supplies ESKF or native-filter
constants or claims an inherent advantage from renaming coordinates.

uniform_upper_vs_rival_witness provides a sufficient strict worst-case test:
ESKF must have a uniform upper bound U on a common admissible set, and a
competitor must have a certified witness error at least L on that same set,
with U < L. The result is a worst-case separation, not pointwise superiority
on each common input trace. A pointwise claim would need a stronger argument.
Separate comparisons are needed for EKF2 and EKF3, each mission phase and
each physical metric. Strict comparisons at zero error cannot be expected.

Remaining implementation obligations

1. Declare physical scales, initial state/bias/terrain uncertainty, flight
   envelope, bounded noise event, sensor schedule and finite outage duration.
   Gaussian noise has unbounded support; a deterministic bounded-noise claim
   must name its event. No probability coverage is proved here.
2. Certify the actual Modelica/generated ESKF map over that region, including
   FOH input approximation, bias Jacobian truncation, shared-sample noise,
   covariance/reset/trust-limit branches, sensor gates, delayed measurements,
   stationary activation and publication prediction. The source-level error
   convention is inspected; no compiler-refinement theorem is supplied.
3. Certify equivalent hypotheses from the pinned native EKF2 and EKF3 cores.
   Their priors, effective Q/R, auxiliary states and height/magnetic policies
   differ in the current experiments. Uniform conditions must include those
   differences rather than grant ESKF information denied to the references.
4. Derive native witness lower bounds or a valid common-model stochastic
   comparison. Larger competitor upper bounds are insufficient.
5. Resolve remaining empirical losses and validate on new independent noise
   and initial-error draws. The stationary-validation evidence in the sibling
   directory records four GPS-denied horizontal losses to each native filter,
   an 8/8 denied vertical split against PX4 and six denied yaw losses to EKF3.
   Those results concern the frozen stationary joint-horizon candidate, not a
   universal impossibility result for future ESKF changes.

Verification and reproduction

check_bounded_error.py rebuilds five direct GNC dependencies and all four
delivered modules into a fresh owned overlay, then audits all 42 public
theorems transitively for axioms. Only propext, Classical.choice and Quot.sound
are accepted. Missing/duplicate audits and sorryAx are rejected. Other GNC and
mathlib dependencies reuse the pinned checked cache. This is a targeted source
check, not a full GNC release build. theory.json records source, checker,
compiled artifact and audit hashes, exact commands and the GNC source revision.

python tools/estimator_comparison/check_bounded_error.py \
  --gnc /path/to/gnc_lean --cache /path/to/gnc_lean/.lake \
  --lean /path/to/lean-4.29.1 \
  --build "$HOME/scratch/modelica_models/bounded-error-check" \
  --output "$HOME/scratch/modelica_models/bounded-error-check.json"

Use fresh build and output paths. Source checkouts and dependency caches are
not modified. All large generated proof artifacts stay under the owned build
path. Proof sources and review receipts are retained in this repository.
