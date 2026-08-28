within Estimation;

package FusionHorizon
  "Estimator-agnostic delayed fusion horizon and SE_2(3) output predictor"
  constant Integer DeltaLength = 56
    "Flat storage width of one Estimation.FusionHorizon.Delta:
     3 position, 3 velocity, 4 quaternion, 1 span, and five 3x3 bias
     Jacobians stored row-major.";

  annotation(Documentation(info = "<html>
    <p>The estimator fuses at a delayed horizon <code>t - D</code>, where every
    aiding measurement has already arrived, and the state control needs at
    <code>t</code> is recovered by composing the SE_2(3) preintegrals that were
    buffered while waiting. This package owns everything on the buffer side of
    that split: the fixed-size ring of per-tick deltas, the composition algebra,
    the output predictor, and the re-base that follows a horizon correction.</p>

    <p><b>Nothing here knows what a filter is.</b> There is no covariance, no
    sigma point, no error state, and no tangent ordering in this package. The
    estimator boundary is the one that already exists:
    <code>Avionics.ImuSample</code> carries the accumulated delta and its bias
    Jacobians into the filter, <code>Avionics.NavigationEstimate</code> plus the
    published gyroscope and accelerometer bias carry the corrected horizon state
    back out, and <code>Avionics.EstimatorStatus.correctionOutcome</code> is the
    state-shifted signal that triggers a re-base. Any block extending
    <code>Estimation.StrapdownINS.PartialEstimator</code> plugs in unchanged;
    <code>HorizonEstimator</code> does exactly that through a
    <code>replaceable</code> slot, and the ESKF and the manifold UKF are both
    exercised through it.</p>

    <p><b>Why the buffer is exact.</b> Under the mixed-invariant flow
    <code>Xdot = M X + X N(t)</code> with constant <code>M</code>, the flow over
    an interval factors as <code>X(T) = L X(0) R</code> with <code>L</code> and
    <code>R</code> independent of the initial state. A horizon correction
    therefore reapplies the same precomputed right factors exactly, and adjacent
    right factors multiply, so composing N buffered deltas and integrating the
    same N samples in one pass are the same group element. See the FOH paper,
    Lemma 3, Theorem 6 (Sec. VI, exact reapplication), and Lemma 5 (Sec. IV-C,
    error composition).</p>

    <p><b>Anti-aliasing is an assumption, not code.</b> The buffer integrates one
    sample per tick and claims first-order-hold accuracy for it. That claim holds
    only for a stream band-limited below half the sampling rate. It is, in
    silicon, before sampling: the deployed ICM-45686 runs its gyroscope at
    1600 Hz output data rate with the on-die low-pass at ODR/8 = 200 Hz and the
    accelerometer at ODR/16 = 100 Hz, so the raw 32 kHz mechanical bandwidth
    never reaches this code. No filter appears in this package because its input
    is defined to be the already-filtered stream. A change to the hardware filter
    configuration invalidates the hold-order error budget, and nothing in this
    model would detect it.</p>

    <p><b>Bias anchoring.</b> Every buffered delta is integrated at one anchor
    bias, fixed at initialization. The estimator supplies a bias estimate; the
    horizon moves the whole composed window to it through the accumulated
    Jacobians, which is the same first-order move
    <code>Estimation.StrapdownINS.correctPreintegratedImu</code> performs on a
    single packet. The remainder is second order and bounded by the horizon
    length rather than by mission length: at a 200 ms window and a 0.05 rad/s
    bias offset it is about 1e-4 rad. The horizon never asks the estimator for an
    error-state injection, only for a bias value, so the same path serves an
    additive-bias ESKF and a manifold UKF without either being privileged.</p>
  </html>"));
end FusionHorizon;
