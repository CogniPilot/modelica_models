import GNC.Lie.Euclidean

noncomputable section
open Matrix
namespace GNC.MagneticGauge

variable {m n : Type*} [Fintype n]

def projectedJacobian (H : Matrix m n ℝ) (p : n → ℝ) : Matrix m n ℝ :=
  fun row column => H row column - ((H row ⬝ᵥ p) / (p ⬝ᵥ p)) * p column

theorem projected_action (H : Matrix m n ℝ) (p v : n → ℝ) (row : m) :
    (projectedJacobian H p).mulVec v row =
      H.mulVec v row - ((H row ⬝ᵥ p) / (p ⬝ᵥ p)) * (p ⬝ᵥ v) := by
  simp only [projectedJacobian, mulVec, dotProduct, sub_mul,
    Finset.sum_sub_distrib, mul_assoc, Finset.mul_sum]

theorem projected_nullspace (H : Matrix m n ℝ) (p : n → ℝ)
    (hp : p ⬝ᵥ p ≠ 0) : (projectedJacobian H p).mulVec p = 0 := by
  ext row
  rw [projected_action]
  simp [div_mul_cancel₀ _ hp, mulVec]

theorem projected_preserves_transverse_action (H : Matrix m n ℝ)
    (p v : n → ℝ) (hv : p ⬝ᵥ v = 0) :
    (projectedJacobian H p).mulVec v = H.mulVec v := by
  ext row
  rw [projected_action, hv]
  simp


theorem projected_preserves_secant (H : Matrix m n ℝ) (p v : n → ℝ)
    (residual : m → ℝ) (hv : p ⬝ᵥ v = 0) (hr : H.mulVec v = residual) :
    (projectedJacobian H p).mulVec v = residual := by
  rw [projected_preserves_transverse_action H p v hv, hr]

theorem projected_zero_direction_information [Fintype m]
    (H : Matrix m n ℝ) (W : Matrix m m ℝ) (p : n → ℝ)
    (hp : p ⬝ᵥ p ≠ 0) :
    ((projectedJacobian H p).mulVec p) ⬝ᵥ
      (W.mulVec ((projectedJacobian H p).mulVec p)) = 0 := by
  rw [projected_nullspace H p hp]
  simp

def stationaryLinearOutput (field gravity angle bias : Vec3) : Vec3 × Vec3 :=
  (skew field *ᵥ angle, skew gravity *ᵥ angle + bias)

theorem stationary_attitude_bias_ambiguity (field gravity : Vec3) (a : ℝ) :
    stationaryLinearOutput field gravity (a • field)
      (-(skew gravity *ᵥ (a • field))) = (0, 0) := by
  simp [stationaryLinearOutput, skew_mulVec]


def shortestCayley (p y : Vec3) : Vec3 :=
  (2 / (1 + p ⬝ᵥ y)) • (y ⨯₃ p)

theorem shortest_cayley_transverse (p y : Vec3) :
    p ⬝ᵥ shortestCayley p y = 0 := by
  simp [shortestCayley]

theorem symmetric_cayley_secant (p y : Vec3)
    (hp : p ⬝ᵥ p = 1) (hy : y ⬝ᵥ y = 1)
    (hd : 1 + p ⬝ᵥ y ≠ 0) :
    skew ((1 / 2 : ℝ) • (p + y)) *ᵥ shortestCayley p y = y - p := by
  rw [skew_mulVec]
  simp only [shortestCayley, map_smul, LinearMap.smul_apply, smul_smul,
    cross_cross_eq_smul_sub_smul', add_dotProduct, dotProduct_add,
    hp, hy, dotProduct_comm y p]
  ext axis
  simp only [Pi.smul_apply, Pi.sub_apply, smul_eq_mul]
  field_simp [hd]
  ring


theorem projected_cayley_secant (p y : Vec3)
    (hp : p ⬝ᵥ p = 1) (hy : y ⬝ᵥ y = 1)
    (hd : 1 + p ⬝ᵥ y ≠ 0) :
    (projectedJacobian (skew ((1 / 2 : ℝ) • (p + y))) p).mulVec
      (shortestCayley p y) = y - p := by
  exact projected_preserves_secant _ _ _ _
    (shortest_cayley_transverse p y) (symmetric_cayley_secant p y hp hy hd)

end GNC.MagneticGauge
#print axioms GNC.MagneticGauge.projected_action
#print axioms GNC.MagneticGauge.projected_nullspace
#print axioms GNC.MagneticGauge.projected_preserves_transverse_action

#print axioms GNC.MagneticGauge.projected_preserves_secant

#print axioms GNC.MagneticGauge.projected_zero_direction_information

#print axioms GNC.MagneticGauge.stationary_attitude_bias_ambiguity

#print axioms GNC.MagneticGauge.shortest_cayley_transverse
#print axioms GNC.MagneticGauge.symmetric_cayley_secant

#print axioms GNC.MagneticGauge.projected_cayley_secant
