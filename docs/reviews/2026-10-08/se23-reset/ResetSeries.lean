import GNC.Analysis.TrigonometricPolynomial
import Mathlib.Tactic

noncomputable section
open Real Set
namespace GNC.ResetSeries

def sine7 (x : ℝ) : ℝ := x - x^3/6 + x^5/120 - x^7/5040
def sine9 (x : ℝ) : ℝ := sine7 x + x^9/362880
def cosine8 (x : ℝ) : ℝ := 1 - x^2/2 + x^4/24 - x^6/720 + x^8/40320
def cosine10 (x : ℝ) : ℝ := cosine8 x - x^10/3628800

def p1 (s : ℝ) : ℝ := 1/6 - s/120 + s*s/5040
def p2 (s : ℝ) : ℝ := 1/24 - s/720 + s*s/40320
def p3 (s : ℝ) : ℝ := 1/120 - s/2520 + s*s/120960
def p4 (s : ℝ) : ℝ := 1/720 - s/20160 + s*s/1209600
def p5 (s : ℝ) : ℝ := 1/24 - s/360 + s*s/13440

def c1 (x : ℝ) : ℝ := (x - sin x) / x^3
def c2 (x : ℝ) : ℝ := (x^2 + 2*cos x - 2) / (2*x^4)
def c3 (x : ℝ) : ℝ := (x*cos x + 2*x - 3*sin x) / (2*x^5)
def c4 (x : ℝ) : ℝ := (x^2 + x*sin x + 4*cos x - 4) / (2*x^6)
def c5 (x : ℝ) : ℝ := (2 - 2*cos x - x*sin x) / (2*x^4)

private theorem sine7_taylor (x : ℝ) : taylorWithinEval sin 8 univ 0 x = sine7 x := by
  norm_num [sine7,taylorWithinEval_succ,iteratedDerivWithin_univ,iteratedDeriv_succ,
    Real.deriv_sin,deriv_cos',deriv.neg',deriv_neg]
  ring

private theorem sine9_taylor (x : ℝ) : taylorWithinEval sin 10 univ 0 x = sine9 x := by
  norm_num [sine7,sine9,taylorWithinEval_succ,iteratedDerivWithin_univ,iteratedDeriv_succ,
    Real.deriv_sin,deriv_cos',deriv.neg',deriv_neg]
  ring

private theorem cosine8_taylor (x : ℝ) : taylorWithinEval cos 9 univ 0 x = cosine8 x := by
  norm_num [cosine8,taylorWithinEval_succ,iteratedDerivWithin_univ,iteratedDeriv_succ,
    Real.deriv_sin,deriv_cos',deriv.neg',deriv_neg]
  ring

private theorem cosine10_taylor (x : ℝ) : taylorWithinEval cos 11 univ 0 x = cosine10 x := by
  norm_num [cosine8,cosine10,taylorWithinEval_succ,iteratedDerivWithin_univ,iteratedDeriv_succ,
    Real.deriv_sin,deriv_cos',deriv.neg',deriv_neg]
  ring

private theorem sine7_bound {x : ℝ} (hx : 0 < x) : |sin x - sine7 x| ≤ x^9/362880 := by
  have h := TrigonometricPolynomial.taylor_bound_pos sin contDiff_sin 8
    (abs_iteratedDeriv_sin_le_one 9) hx
  rw [sine7_taylor] at h
  norm_num at h
  exact h

private theorem sine9_bound {x : ℝ} (hx : 0 < x) : |sin x - sine9 x| ≤ x^11/39916800 := by
  have h := TrigonometricPolynomial.taylor_bound_pos sin contDiff_sin 10
    (abs_iteratedDeriv_sin_le_one 11) hx
  rw [sine9_taylor] at h
  norm_num at h
  exact h

private theorem cosine8_bound {x : ℝ} (hx : 0 < x) : |cos x - cosine8 x| ≤ x^10/3628800 := by
  have h := TrigonometricPolynomial.taylor_bound_pos cos contDiff_cos 9
    (abs_iteratedDeriv_cos_le_one 10) hx
  rw [cosine8_taylor] at h
  norm_num at h
  exact h

private theorem cosine10_bound {x : ℝ} (hx : 0 < x) : |cos x - cosine10 x| ≤ x^12/479001600 := by
  have h := TrigonometricPolynomial.taylor_bound_pos cos contDiff_cos 11
    (abs_iteratedDeriv_cos_le_one 12) hx
  rw [cosine10_taylor] at h
  norm_num at h
  exact h

private theorem weighted_error {a b u v U V : ℝ} (ha : 0 ≤ a) (hb : 0 ≤ b)
    (hu : |u| ≤ U) (hv : |v| ≤ V) : |a*u + b*v| ≤ a*U + b*V := by
  apply (abs_add_le _ _).trans
  simp only [abs_mul,abs_of_nonneg ha,abs_of_nonneg hb]
  exact add_le_add (mul_le_mul_of_nonneg_left hu ha) (mul_le_mul_of_nonneg_left hv hb)

theorem first_bound {x : ℝ} (hx : 0 < x) : |c1 x - p1 (x^2)| ≤ x^6/362880 := by
  have he : c1 x - p1 (x^2) = (sine7 x - sin x) / x^3 := by
    dsimp [c1,p1,sine7]; field_simp <;> ring
  rw [he,abs_div,abs_of_pos (pow_pos hx 3)]
  have h := sine7_bound hx
  rw [abs_sub_comm] at h
  calc
    |sine7 x - sin x| / x^3 ≤ (x^9/362880) / x^3 :=
      div_le_div_of_nonneg_right h (pow_nonneg hx.le 3)
    _ = x^6/362880 := by field_simp <;> ring

theorem second_bound {x : ℝ} (hx : 0 < x) : |c2 x - p2 (x^2)| ≤ x^6/3628800 := by
  have he : c2 x - p2 (x^2) = (cos x - cosine8 x) / x^4 := by
    dsimp [c2,p2,cosine8]; field_simp <;> ring
  rw [he,abs_div,abs_of_pos (pow_pos hx 4)]
  calc
    |cos x - cosine8 x| / x^4 ≤ (x^10/3628800) / x^4 :=
      div_le_div_of_nonneg_right (cosine8_bound hx) (pow_nonneg hx.le 4)
    _ = x^6/3628800 := by field_simp <;> ring

theorem third_bound {x : ℝ} (hx : 0 < x) : |c3 x - p3 (x^2)| ≤ x^6/5702400 := by
  have he : c3 x - p3 (x^2) =
      (x*(cos x-cosine8 x) + 3*(sine9 x-sin x)) / (2*x^5) := by
    dsimp [c3,p3,sine9,sine7,cosine8]; field_simp <;> ring
  have hs := sine9_bound hx
  rw [abs_sub_comm] at hs
  have hn := weighted_error hx.le (by norm_num : (0:ℝ) ≤ 3) (cosine8_bound hx) hs
  rw [he,abs_div,abs_of_pos (show 0 < 2*x^5 by positivity)]
  calc
    _ ≤ (x*(x^10/3628800)+3*(x^11/39916800))/(2*x^5) :=
      div_le_div_of_nonneg_right hn (by positivity)
    _ = x^6/5702400 := by field_simp <;> ring

theorem fourth_bound {x : ℝ} (hx : 0 < x) : |c4 x - p4 (x^2)| ≤ x^6/59875200 := by
  have he : c4 x - p4 (x^2) =
      (x*(sin x-sine9 x) + 4*(cos x-cosine10 x)) / (2*x^6) := by
    dsimp [c4,p4,sine9,sine7,cosine10,cosine8]; field_simp <;> ring
  have hn := weighted_error hx.le (by norm_num : (0:ℝ) ≤ 4) (sine9_bound hx) (cosine10_bound hx)
  rw [he,abs_div,abs_of_pos (show 0 < 2*x^6 by positivity)]
  calc
    _ ≤ (x*(x^11/39916800)+4*(x^12/479001600))/(2*x^6) :=
      div_le_div_of_nonneg_right hn (by positivity)
    _ = x^6/59875200 := by field_simp <;> ring

theorem fifth_bound {x : ℝ} (hx : 0 < x) : |c5 x - p5 (x^2)| ≤ x^6/604800 := by
  have he : c5 x - p5 (x^2) =
      (2*(cosine8 x-cos x) + x*(sine7 x-sin x)) / (2*x^4) := by
    dsimp [c5,p5,sine7,cosine8]; field_simp <;> ring
  have hc := cosine8_bound hx
  have hs := sine7_bound hx
  rw [abs_sub_comm] at hc hs
  have hn := weighted_error (by norm_num : (0:ℝ) ≤ 2) hx.le hc hs
  rw [he,abs_div,abs_of_pos (show 0 < 2*x^4 by positivity)]
  calc
    _ ≤ (2*(x^10/3628800)+x*(x^9/362880))/(2*x^4) :=
      div_le_div_of_nonneg_right hn (by positivity)
    _ = x^6/604800 := by field_simp <;> ring

theorem coefficients_in_series_range {x : ℝ} (hx : 0 < x) (hupper : x ≤ 1/5) :
    |c1 x - p1 (x^2)| ≤ 1/5000000000 ∧
    |c2 x - p2 (x^2)| ≤ 1/5000000000 ∧
    |c3 x - p3 (x^2)| ≤ 1/5000000000 ∧
    |c4 x - p4 (x^2)| ≤ 1/5000000000 ∧
    |c5 x - p5 (x^2)| ≤ 1/5000000000 := by
  have hpow : x^6 ≤ (1/5 : ℝ)^6 := by gcongr
  norm_num at hpow
  refine ⟨(first_bound hx).trans ?_, (second_bound hx).trans ?_,
    (third_bound hx).trans ?_, (fourth_bound hx).trans ?_, (fifth_bound hx).trans ?_⟩
  all_goals nlinarith

theorem eskf_correction_uses_series {x : ℝ} (hx : 0 ≤ x) (hlimit : x ≤ 3/20) :
    x^2 < (1/25 : ℝ) := by nlinarith

end GNC.ResetSeries
#print axioms GNC.ResetSeries.first_bound
#print axioms GNC.ResetSeries.second_bound
#print axioms GNC.ResetSeries.third_bound
#print axioms GNC.ResetSeries.fourth_bound
#print axioms GNC.ResetSeries.fifth_bound
#print axioms GNC.ResetSeries.eskf_correction_uses_series

#print axioms GNC.ResetSeries.coefficients_in_series_range
