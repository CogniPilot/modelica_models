import GNC.Estimation.KalmanCorrection

open Matrix
noncomputable section

def pressureConsiderGain (K : Matrix (Fin 15) (Fin 1) ℝ) :
    Matrix (Fin 16) (Fin 1) ℝ :=
  fun i j => if h : i.val < 15 then K ⟨i.val, h⟩ j else 0

example (P : Matrix (Fin 16) (Fin 16) ℝ)
    (B Kopt : Matrix (Fin 16) (Fin 1) ℝ)
    (K : Matrix (Fin 15) (Fin 1) ℝ)
    (S : Matrix (Fin 1) (Fin 1) ℝ)
    (J : Matrix (Fin 16) (Fin 16) ℝ)
    (hS : S.PosSemidef) (hopt : Kopt * S = B) :
    let gap := GNC.Estimation.KalmanCorrection.posterior P B
      (pressureConsiderGain K) S -
      GNC.Estimation.KalmanCorrection.posterior P B Kopt S
    (J * gap * Jᵀ).PosSemidef := by
  exact GNC.Estimation.CovariancePropagation.positive_semidefinite J
    (GNC.Estimation.KalmanCorrection.gain_gap_positive_semidefinite
      P B (pressureConsiderGain K) Kopt S hS hopt)

example (L : Matrix (Fin 16) (Fin 16) ℝ)
    (U K : Matrix (Fin 16) (Fin 1) ℝ)
    (V : Matrix (Fin 1) (Fin 1) ℝ) (H : Matrix (Fin 1) (Fin 16) ℝ) :
    let F := 1 - K * H
    (F * (L * Lᵀ) * Fᵀ + K * (Uᵀ * U + V * Vᵀ) * Kᵀ
      - F * (L * U) * Kᵀ - K * (L * U)ᵀ * Fᵀ).PosSemidef := by
  exact GNC.Estimation.KalmanCorrection.conditional_factor_positive_semidefinite
    L U K V H

#print axioms GNC.Estimation.KalmanCorrection.gain_gap_positive_semidefinite
#print axioms GNC.Estimation.KalmanCorrection.conditional_factor_positive_semidefinite
