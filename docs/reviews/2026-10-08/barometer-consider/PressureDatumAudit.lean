import GNC.Estimation.KalmanCorrection

open Matrix
noncomputable section

def pressureConsiderGain (K : Matrix (Fin 15) (Fin 1) ℝ) :
    Matrix (Fin 16) (Fin 1) ℝ :=
  fun i j => if h : i.val < 15 then K ⟨i.val, h⟩ j else 0

example (K : Matrix (Fin 15) (Fin 1) ℝ) (j : Fin 1) :
    pressureConsiderGain K 15 j = 0 := by
  simp [pressureConsiderGain]

example (L : Matrix (Fin 16) (Fin 16) ℝ)
    (U : Matrix (Fin 16) (Fin 1) ℝ) (K : Matrix (Fin 15) (Fin 1) ℝ)
    (V : Matrix (Fin 1) (Fin 1) ℝ) (H : Matrix (Fin 1) (Fin 16) ℝ) :
    let gain := pressureConsiderGain K
    let F := 1 - gain * H
    (F * (L * Lᵀ) * Fᵀ + gain * (Uᵀ * U + V * Vᵀ) * gainᵀ
      - F * (L * U) * gainᵀ - gain * (L * U)ᵀ * Fᵀ).PosSemidef := by
  exact GNC.Estimation.KalmanCorrection.conditional_factor_positive_semidefinite
    L U (pressureConsiderGain K) V H

#print axioms GNC.Estimation.KalmanCorrection.conditional_factor_positive_semidefinite
#print axioms GNC.Estimation.KalmanCorrection.gain_gap_positive_semidefinite
