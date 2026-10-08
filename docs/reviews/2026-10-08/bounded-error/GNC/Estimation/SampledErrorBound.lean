import GNC.Analysis.TaylorCertificate
import GNC.Estimation.RemainderStability
import Mathlib.Analysis.SpecificLimits.Basic

/-! Sampled nonlinear estimator error certificates.

The error map must close over navigation/bias error and an explicit context
containing covariance, delayed samples, inputs and sensor-policy state. Norms
refer to a declared common physical scaling. Regional certificates must be
uniform over the admissible contexts, disturbances and policy branches.
No firmware implementation is assumed to satisfy these hypotheses.
-/
noncomputable section
open scoped BigOperators
namespace GNC.Estimation.SampledErrorBound

structure StepBound where
  linear : ℝ
  quadratic : ℝ
  cubic : ℝ
  disturbance : ℝ
  linear_nonneg : 0 ≤ linear
  quadratic_nonneg : 0 ≤ quadratic
  cubic_nonneg : 0 ≤ cubic
  disturbance_nonneg : 0 ≤ disturbance

def StepBound.eval (b : StepBound) (r : ℝ) : ℝ :=
  b.linear*r + b.quadratic*r^2 + b.cubic*r^3 + b.disturbance

theorem StepBound.eval_nonneg (b : StepBound) {r : ℝ} (hr : 0 ≤ r) :
    0 ≤ b.eval r := by
  unfold StepBound.eval
  positivity [b.linear_nonneg, b.quadratic_nonneg, b.cubic_nonneg, b.disturbance_nonneg]

theorem StepBound.eval_mono (b : StepBound) {r s : ℝ} (hr : 0 ≤ r) (hrs : r ≤ s) :
    b.eval r ≤ b.eval s := by
  unfold StepBound.eval
  gcongr
  · exact b.linear_nonneg
  · exact b.quadratic_nonneg
  · exact b.cubic_nonneg

def envelope (bounds : ℕ → StepBound) (initial : ℝ) : ℕ → ℝ
  | 0 => initial
  | n+1 => (bounds n).eval (envelope bounds initial n)

theorem envelope_nonneg (bounds : ℕ → StepBound) {initial : ℝ} (hi : 0 ≤ initial) :
    ∀ n, 0 ≤ envelope bounds initial n := by
  intro n
  induction n with
  | zero => exact hi
  | succ n ih => exact (bounds n).eval_nonneg ih

variable {E : Type*} [NormedAddCommGroup E]

theorem certified_tube (bounds : ℕ → StepBound) (domain : ℕ → ℝ)
    (update : ℕ → E → E) (error : ℕ → E) {initial : ℝ}
    (hi : ‖error 0‖ ≤ initial)
    (hstep : ∀ n, error (n+1) = update n (error n))
    (hmap : ∀ n y, ‖y‖ ≤ domain n → ‖update n y‖ ≤ (bounds n).eval ‖y‖)
    (hregion : ∀ n, envelope bounds initial n ≤ domain n) :
    ∀ n, ‖error n‖ ≤ envelope bounds initial n := by
  intro n
  induction n with
  | zero => exact hi
  | succ n ih =>
    rw [hstep, envelope]
    exact (hmap n (error n) (ih.trans (hregion n))).trans
      ((bounds n).eval_mono (norm_nonneg _) ih)

theorem invariant_tube (bound : StepBound) (update : ℕ → E → E)
    (error : ℕ → E) {radius : ℝ} (hi : ‖error 0‖ ≤ radius)
    (hstep : ∀ n, error (n+1) = update n (error n))
    (hmap : ∀ n y, ‖y‖ ≤ radius → ‖update n y‖ ≤ bound.eval ‖y‖)
    (hinward : bound.eval radius ≤ radius) :
    ∀ n, ‖error n‖ ≤ radius := by
  intro n
  induction n with
  | zero => exact hi
  | succ n ih =>
    rw [hstep]
    exact (hmap n (error n) ih).trans
      ((bound.eval_mono (norm_nonneg _) ih).trans hinward)

theorem contextual_tube {C : Type*} (bounds : ℕ → StepBound) (domain : ℕ → ℝ)
    (update : ℕ → C → E → E) (context : ℕ → C) (admissible : ℕ → C → Prop)
    (error : ℕ → E) {initial : ℝ} (hi : ‖error 0‖ ≤ initial)
    (hcontext : ∀ n, admissible n (context n))
    (hstep : ∀ n, error (n+1) = update n (context n) (error n))
    (hmap : ∀ n c, admissible n c → ∀ y, ‖y‖ ≤ domain n →
      ‖update n c y‖ ≤ (bounds n).eval ‖y‖)
    (hregion : ∀ n, envelope bounds initial n ≤ domain n) :
    ∀ n, ‖error n‖ ≤ envelope bounds initial n :=
  certified_tube bounds domain (fun n => update n (context n)) error hi hstep
    (fun n y hy => hmap n (context n) (hcontext n) y hy) hregion

theorem gated_map_bound (accepted rejected : E → E) (gate : E → Bool)
    (bound : StepBound) {radius : ℝ}
    (ha : ∀ y, ‖y‖ ≤ radius → ‖accepted y‖ ≤ bound.eval ‖y‖)
    (hr : ∀ y, ‖y‖ ≤ radius → ‖rejected y‖ ≤ bound.eval ‖y‖) :
    ∀ y, ‖y‖ ≤ radius →
      ‖if gate y then accepted y else rejected y‖ ≤ bound.eval ‖y‖ := by
  intro y hy
  cases gate y <;> simp only [Bool.false_eq_true, ↓reduceIte]
  · exact hr y hy
  · exact ha y hy

theorem ordered_envelopes (ours other : ℕ → StepBound) {r s : ℝ}
    (hr : 0 ≤ r) (hrs : r ≤ s)
    (horder : ∀ n x, 0 ≤ x → (ours n).eval x ≤ (other n).eval x) :
    ∀ n, envelope ours r n ≤ envelope other s n := by
  intro n
  induction n with
  | zero => exact hrs
  | succ n ih =>
    exact (horder n _ (envelope_nonneg ours hr n)).trans
      ((other n).eval_mono (envelope_nonneg ours hr n) ih)

theorem regional_affine_majorant (bound : StepBound) {r radius : ℝ}
    (hr : 0 ≤ r) (hregion : r ≤ radius) :
    bound.eval r ≤
      (bound.linear + bound.quadratic*radius + bound.cubic*radius^2)*r +
        bound.disturbance := by
  unfold StepBound.eval
  have hsq : r^2 ≤ radius^2 := by nlinarith
  have hq := mul_le_mul_of_nonneg_left hregion bound.quadratic_nonneg
  have hc := mul_le_mul_of_nonneg_left hsq bound.cubic_nonneg
  have hm := mul_le_mul_of_nonneg_right (add_le_add hq hc) hr
  nlinarith

def affineEnvelope (factor noise initial : ℝ) : ℕ → ℝ
  | 0 => initial
  | n+1 => factor * affineEnvelope factor noise initial n + noise

theorem affine_closed_form {factor noise initial : ℝ} (hf : factor ≠ 1) :
    ∀ n, affineEnvelope factor noise initial n =
      factor^n*initial + noise*(factor^n-1)/(factor-1) := by
  intro n
  induction n with
  | zero => simp [affineEnvelope]
  | succ n ih =>
    rw [affineEnvelope, ih, pow_succ]
    field_simp [sub_ne_zero.mpr hf]
    ring

theorem unit_factor_outage (noise initial : ℝ) :
    ∀ n, affineEnvelope 1 noise initial n = initial + n*noise := by
  intro n
  induction n with
  | zero => simp [affineEnvelope]
  | succ n ih => simp only [affineEnvelope, ih, Nat.cast_add, Nat.cast_one]; ring

theorem affine_segment_bound (error : ℕ → E) {factor noise initial : ℝ}
    (hf : 0 ≤ factor) (start : ℕ) (hi : ‖error start‖ ≤ initial)
    (hstep : ∀ n, ‖error (start+n+1)‖ ≤ factor*‖error (start+n)‖+noise) :
    ∀ n, ‖error (start+n)‖ ≤ affineEnvelope factor noise initial n := by
  intro n
  induction n with
  | zero => simpa [affineEnvelope] using hi
  | succ n ih =>
    exact (hstep n).trans (by
      simpa [affineEnvelope] using
        add_le_add_right (mul_le_mul_of_nonneg_left ih hf) noise)

theorem recovery_decay (factor ultimate initial : ℝ) :
    ∀ n, affineEnvelope factor ((1-factor)*ultimate) initial n =
      ultimate + factor^n*(initial-ultimate) := by
  intro n
  induction n with
  | zero => simp [affineEnvelope]
  | succ n ih => rw [affineEnvelope, ih, pow_succ]; ring

theorem recovery_envelope_limit {factor ultimate initial : ℝ}
    (hf : 0 ≤ factor) (hc : factor < 1) :
    Filter.Tendsto (affineEnvelope factor ((1-factor)*ultimate) initial)
      Filter.atTop (nhds ultimate) := by
  have h := (tendsto_pow_atTop_nhds_zero_of_lt_one hf hc).mul_const (initial-ultimate)
  have he : affineEnvelope factor ((1-factor)*ultimate) initial =
      fun n => ultimate + factor^n*(initial-ultimate) :=
    funext (recovery_decay factor ultimate initial)
  rw [he]
  simpa using h.const_add ultimate

theorem gps_outage_recovery (error : ℕ → E) (loss outage : ℕ)
    {growth drift contraction ultimate initial : ℝ}
    (hg : 0 ≤ growth) (hc : 0 ≤ contraction)
    (hi : ‖error loss‖ ≤ initial)
    (hdenied : ∀ n, n < outage →
      ‖error (loss+n+1)‖ ≤ growth*‖error (loss+n)‖+drift)
    (hreturn : ∀ n, ‖error (loss+outage+n+1)‖ ≤
      contraction*‖error (loss+outage+n)‖+(1-contraction)*ultimate) :
    ∀ n, ‖error (loss+outage+n)‖ ≤
      ultimate + contraction^n*(affineEnvelope growth drift initial outage-ultimate) := by
  have hfinite : ∀ n, n ≤ outage →
      ‖error (loss+n)‖ ≤ affineEnvelope growth drift initial n := by
    intro n
    induction n with
    | zero => intro _; simpa [affineEnvelope] using hi
    | succ n ih =>
      intro hn
      exact (hdenied n (by omega)).trans (by
        simpa [affineEnvelope] using add_le_add_right
          (mul_le_mul_of_nonneg_left (ih (by omega)) hg) drift)
  intro n
  simpa only [recovery_decay] using
    affine_segment_bound error hc (loss+outage) (hfinite outage le_rfl) hreturn n

theorem publication_bound {fusion published : E} {radius gain offset : ℝ}
    (hg : 0 ≤ gain) (hf : ‖fusion‖ ≤ radius)
    (hp : ‖published‖ ≤ gain*‖fusion‖+offset) :
    ‖published‖ ≤ gain*radius+offset :=
  hp.trans (add_le_add (mul_le_mul_of_nonneg_left hf hg) le_rfl)

theorem strict_local_remainder_advantage {linear quadratic cubic noise radius : ℝ}
    (hr : 0 < radius) (hgain : cubic*radius < quadratic) :
    linear*radius+cubic*radius^3+noise < linear*radius+quadratic*radius^2+noise := by
  have h := mul_lt_mul_of_pos_right hgain (sq_pos_of_pos hr)
  nlinarith

theorem uniform_upper_vs_rival_witness {I : Type*} (ours rival : I → ℝ)
    {upper lower : ℝ} (hupper : ∀ i, ours i ≤ upper)
    (witness : I) (hlower : lower ≤ rival witness) (hgap : upper < lower) :
    (∀ i, ours i < rival witness) ∧ ¬(∀ i, rival i ≤ upper) := by
  constructor
  · intro i; exact (hupper i).trans_lt (hgap.trans_le hlower)
  · intro hall; linarith [hall witness]

theorem indistinguishable_state_lower_bound (first second estimate : E) :
    ‖first-second‖/2 ≤ max ‖first-estimate‖ ‖second-estimate‖ := by
  have h := norm_sub_le (first-estimate) (second-estimate)
  have he : first-estimate-(second-estimate) = first-second := by abel
  rw [he] at h
  nlinarith [le_max_left ‖first-estimate‖ ‖second-estimate‖,
    le_max_right ‖first-estimate‖ ‖second-estimate‖]

theorem common_history_lower_bound {H : Type*} (estimator : H → E)
    (first second : E) {history₁ history₂ : H} (same : history₁ = history₂) :
    ‖first-second‖/2 ≤
      max ‖first-estimator history₁‖ ‖second-estimator history₂‖ := by
  rw [same]
  exact indistinguishable_state_lower_bound first second (estimator history₂)

section Taylor
variable [NormedSpace ℝ E] [CompleteSpace E]

theorem sampled_map_from_derivatives (path derivative curvature : ℝ → E)
    {linear quadratic cubic noise radius : ℝ}
    (hfirst : ∀ s ∈ Set.Icc (0:ℝ) 1, HasDerivAt path (derivative s) s)
    (hsecond : ∀ s ∈ Set.Icc (0:ℝ) 1, HasDerivAt derivative (curvature s) s)
    (hcontinuous : ContinuousOn curvature (Set.Icc (0:ℝ) 1))
    (hcurvature : ∀ s ∈ Set.Icc (0:ℝ) 1,
      ‖curvature s‖ ≤ 2*(quadratic+cubic*radius)*radius^2)
    (hzero : ‖path 0‖ ≤ noise) (hlinear : ‖derivative 0‖ ≤ linear*radius) :
    ‖path 1‖ ≤ linear*radius+quadratic*radius^2+cubic*radius^3+noise := by
  have hrem := GNC.TaylorCertificate.remainder_bound path derivative curvature
    hfirst hsecond hcontinuous hcurvature
  have hdecomp : path 1 = (path 1-path 0-derivative 0)+path 0+derivative 0 := by abel
  calc
    ‖path 1‖ ≤ ‖path 1-path 0-derivative 0‖+‖path 0‖+‖derivative 0‖ := by
      conv_lhs => rw [hdecomp]
      exact (norm_add_le _ _).trans (add_le_add (norm_add_le _ _) le_rfl)
    _ ≤ (2*(quadratic+cubic*radius)*radius^2)/2+noise+linear*radius := by gcongr
    _ = _ := by ring

end Taylor
end GNC.Estimation.SampledErrorBound
