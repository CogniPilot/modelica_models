import GNC.Estimation.SampledErrorBound
import GNC.Lie.Euclidean

/-! Ideal flat-ground, spatially uniform-field sensor geometry.

These expressions follow the horizontal-position dependence of the Modelica
range, compensated-flow, barometer, magnetic and inertial observation maps.
This is a mathematical sensor model, not a compiler-refinement proof.
It excludes terrain maps, visual landmarks and spatial magnetic maps.
-/
noncomputable section
namespace GNC.Estimation.GpsDeniedGeometry
open Matrix SampledErrorBound

abbrev SensorOutput := (Fin 2 → ℝ) × ℝ × ℝ × Vec3 × Vec3 × Vec3

def outputs (position velocity field angularRate specificForce : Vec3)
    (rotation : Matrix (Fin 3) (Fin 3) ℝ) (ground pressureBias : ℝ) : SensorOutput :=
  let distance := (position 2-ground)/rotation 2 2
  let bodyVelocity := rotationᵀ *ᵥ velocity
  (![-bodyVelocity 1/distance, bodyVelocity 0/distance],
    distance, position 2+pressureBias, rotationᵀ *ᵥ field, angularRate, specificForce)

theorem horizontal_translation_invariant (position velocity field angularRate specificForce shift : Vec3)
    (rotation : Matrix (Fin 3) (Fin 3) ℝ) (ground pressureBias : ℝ)
    (horizontal : shift 2 = 0) :
    outputs (position+shift) velocity field angularRate specificForce rotation ground pressureBias =
      outputs position velocity field angularRate specificForce rotation ground pressureBias := by
  simp [outputs, Pi.add_apply, horizontal]

theorem propagation_preserves_translation (position velocity acceleration shift : Vec3)
    (step : ℝ) :
    position+shift+step • velocity+(step^2/2) • acceleration =
      (position+step • velocity+(step^2/2) • acceleration)+shift := by
  module

def history (position velocity field angularRate specificForce : ℕ → Vec3)
    (rotation : ℕ → Matrix (Fin 3) (Fin 3) ℝ) (ground pressureBias : ℕ → ℝ) :
    ℕ → SensorOutput :=
  fun n => outputs (position n) (velocity n) (field n) (angularRate n) (specificForce n)
    (rotation n) (ground n) (pressureBias n)

theorem identical_denied_histories (position velocity field angularRate specificForce : ℕ → Vec3)
    (rotation : ℕ → Matrix (Fin 3) (Fin 3) ℝ) (ground pressureBias : ℕ → ℝ)
    (shift : Vec3) (horizontal : shift 2 = 0) :
    history (fun n => position n+shift) velocity field angularRate specificForce rotation ground pressureBias =
      history position velocity field angularRate specificForce rotation ground pressureBias := by
  funext n
  exact horizontal_translation_invariant _ _ _ _ _ _ _ _ _ horizontal

theorem horizontal_minimax_lower_bound
    (position velocity field angularRate specificForce : ℕ → Vec3)
    (rotation : ℕ → Matrix (Fin 3) (Fin 3) ℝ) (ground pressureBias : ℕ → ℝ)
    (shift : Vec3) (horizontal : shift 2 = 0)
    (estimator : (ℕ → SensorOutput) → ℝ) (time : ℕ) :
    |shift 0|/2 ≤ max
      |position time 0 - estimator
        (history position velocity field angularRate specificForce rotation ground pressureBias)|
      |position time 0 + shift 0 - estimator
        (history (fun n => position n+shift) velocity field angularRate specificForce rotation
          ground pressureBias)| := by
  have h := common_history_lower_bound estimator
    (position time 0) (position time 0+shift 0)
    (identical_denied_histories position velocity field angularRate specificForce rotation
      ground pressureBias shift horizontal).symm
  simpa only [Real.norm_eq_abs, sub_add_cancel_left, abs_neg] using h

end GNC.Estimation.GpsDeniedGeometry
