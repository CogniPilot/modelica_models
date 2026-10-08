import GNC.Estimation.SampledErrorBound

/-! Error envelopes for an adaptive-gain correction in a common physical
metric. The observation, reset and numerical defects must be certified for
the actual implementation. These hypotheses are not supplied by a covariance
matrix or by the Kalman gain's least-squares optimality. -/
noncomputable section
namespace GNC.Estimation.CorrectionErrorBound
open GNC.Estimation.SampledErrorBound

variable {E F : Type*} [NormedAddCommGroup E] [NormedSpace ℝ E]
    [NormedAddCommGroup F] [NormedSpace ℝ F]

def correctedError (H : E →L[ℝ] F) (gain : E → F →L[ℝ] E)
    (observationDefect noise : E → F) (resetDefect : E → E) (error : E) : E :=
  error - gain error (H error + observationDefect error + noise error) + resetDefect error

def correctionBound (linear gain observationQuadratic resetQuadratic resetCubic
    noise numerical : ℝ) (hl : 0 ≤ linear) (hg : 0 ≤ gain)
    (ho : 0 ≤ observationQuadratic) (hr : 0 ≤ resetQuadratic)
    (hc : 0 ≤ resetCubic) (hn : 0 ≤ noise) (he : 0 ≤ numerical) : StepBound where
  linear := linear
  quadratic := gain * observationQuadratic + resetQuadratic
  cubic := resetCubic
  disturbance := gain * noise + numerical
  linear_nonneg := hl
  quadratic_nonneg := by positivity
  cubic_nonneg := hc
  disturbance_nonneg := by positivity

theorem correction_map_bound (H : E →L[ℝ] F) (gain : E → F →L[ℝ] E)
    (observationDefect noise : E → F) (resetDefect : E → E)
    {linear gainBound observationQuadratic resetQuadratic resetCubic
      noiseBound numerical radius : ℝ}
    (hl : 0 ≤ linear) (hg : 0 ≤ gainBound) (ho : 0 ≤ observationQuadratic)
    (hr : 0 ≤ resetQuadratic) (hc : 0 ≤ resetCubic)
    (hn : 0 ≤ noiseBound) (he : 0 ≤ numerical)
    (hlinear : ∀ y, ‖y‖ ≤ radius → ‖y - gain y (H y)‖ ≤ linear * ‖y‖)
    (hgain : ∀ y, ‖y‖ ≤ radius → ‖gain y‖ ≤ gainBound)
    (hobservation : ∀ y, ‖y‖ ≤ radius →
      ‖observationDefect y‖ ≤ observationQuadratic * ‖y‖^2)
    (hnoise : ∀ y, ‖y‖ ≤ radius → ‖noise y‖ ≤ noiseBound)
    (hreset : ∀ y, ‖y‖ ≤ radius →
      ‖resetDefect y‖ ≤ resetQuadratic * ‖y‖^2 + resetCubic * ‖y‖^3 + numerical) :
    ∀ y, ‖y‖ ≤ radius →
      ‖correctedError H gain observationDefect noise resetDefect y‖ ≤
        (correctionBound linear gainBound observationQuadratic resetQuadratic resetCubic
          noiseBound numerical hl hg ho hr hc hn he).eval ‖y‖ := by
  intro y hy
  have hdefect : ‖observationDefect y + noise y‖ ≤
      observationQuadratic * ‖y‖^2 + noiseBound :=
    (norm_add_le _ _).trans (add_le_add (hobservation y hy) (hnoise y hy))
  have hscaled : ‖gain y (observationDefect y + noise y)‖ ≤
      gainBound * (observationQuadratic * ‖y‖^2 + noiseBound) :=
    ((gain y).le_opNorm _).trans (mul_le_mul (hgain y hy) hdefect
      (norm_nonneg _) hg)
  have hid : correctedError H gain observationDefect noise resetDefect y =
      (y - gain y (H y)) - gain y (observationDefect y + noise y) + resetDefect y := by
    simp only [correctedError, map_add]
    abel
  rw [hid]
  have hsum := (norm_add_le _ _).trans (add_le_add
    ((norm_sub_le _ _).trans (add_le_add (hlinear y hy) hscaled)) (hreset y hy))
  exact hsum.trans_eq (by simp only [StepBound.eval, correctionBound]; ring)

theorem unobserved_direction_preserved (H : E →L[ℝ] F) (gain : F →L[ℝ] E)
    {error : E} (hunobserved : H error = 0) : error - gain (H error) = error := by
  simp [hunobserved]

theorem unobserved_direction_no_contraction (H : E →L[ℝ] F)
    (gain : F →L[ℝ] E) {error : E} {factor : ℝ}
    (hnonzero : error ≠ 0) (hunobserved : H error = 0) (hfactor : factor < 1) :
    ¬ ‖error - gain (H error)‖ ≤ factor * ‖error‖ := by
  rw [unobserved_direction_preserved H gain hunobserved]
  have hpositive : 0 < ‖error‖ := norm_pos_iff.mpr hnonzero
  have hstrict := mul_lt_mul_of_pos_right hfactor hpositive
  simpa only [one_mul] using not_le.mpr hstrict

def withImplementationDefect (bound : StepBound) (numerical : ℝ)
    (hn : 0 ≤ numerical) : StepBound where
  linear := bound.linear
  quadratic := bound.quadratic
  cubic := bound.cubic
  disturbance := bound.disturbance + numerical
  linear_nonneg := bound.linear_nonneg
  quadratic_nonneg := bound.quadratic_nonneg
  cubic_nonneg := bound.cubic_nonneg
  disturbance_nonneg := add_nonneg bound.disturbance_nonneg hn

omit [NormedSpace ℝ E] in
theorem bounded_implementation_defect (reference actual : E → E) (bound : StepBound)
    {radius numerical : ℝ} (hn : 0 ≤ numerical)
    (hreference : ∀ y, ‖y‖ ≤ radius → ‖reference y‖ ≤ bound.eval ‖y‖)
    (himplementation : ∀ y, ‖y‖ ≤ radius → ‖actual y - reference y‖ ≤ numerical) :
    ∀ y, ‖y‖ ≤ radius → ‖actual y‖ ≤
      (withImplementationDefect bound numerical hn).eval ‖y‖ := by
  intro y hy
  have hid : actual y = reference y + (actual y - reference y) := by abel
  have h := (norm_add_le (reference y) (actual y - reference y)).trans
    (add_le_add (hreference y hy) (himplementation y hy))
  calc
    ‖actual y‖ ≤ bound.eval ‖y‖ + numerical := by simpa only [← hid] using h
    _ = (withImplementationDefect bound numerical hn).eval ‖y‖ := by
      simp only [StepBound.eval, withImplementationDefect, add_assoc]

end GNC.Estimation.CorrectionErrorBound
