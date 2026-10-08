import Mathlib.Data.Matrix.Basic
import Mathlib.Data.Matrix.Mul
import Mathlib.Data.Real.Basic

/-! Exact linearized information identities. Interpreting these matrices as a
Gaussian update additionally requires valid covariances, a common linearization
and correctly represented nuisance-state correlations. No nonlinear SLAM or
floating-point refinement follows from matrix associativity. -/
namespace GNC.Estimation.VisualCompression

variable {Measurements Pose Navigation : Type*}
  [Fintype Measurements] [Fintype Pose] [Fintype Navigation]

def rawInformation (camera : Matrix Measurements Pose ℝ)
    (selector : Matrix Pose Navigation ℝ)
    (weight : Matrix Measurements Measurements ℝ) : Matrix Navigation Navigation ℝ :=
  (camera * selector).transpose * weight * (camera * selector)

def compressedInformation (camera : Matrix Measurements Pose ℝ)
    (selector : Matrix Pose Navigation ℝ)
    (weight : Matrix Measurements Measurements ℝ) : Matrix Navigation Navigation ℝ :=
  selector.transpose * (camera.transpose * weight * camera) * selector

def rawScore (camera : Matrix Measurements Pose ℝ)
    (selector : Matrix Pose Navigation ℝ)
    (weight : Matrix Measurements Measurements ℝ)
    (residual : Matrix Measurements Unit ℝ) : Matrix Navigation Unit ℝ :=
  (camera * selector).transpose * weight * residual

def compressedScore (camera : Matrix Measurements Pose ℝ)
    (selector : Matrix Pose Navigation ℝ)
    (weight : Matrix Measurements Measurements ℝ)
    (residual : Matrix Measurements Unit ℝ) : Matrix Navigation Unit ℝ :=
  selector.transpose * (camera.transpose * weight * residual)

omit [Fintype Navigation] in
theorem information_preserved (camera : Matrix Measurements Pose ℝ)
    (selector : Matrix Pose Navigation ℝ)
    (weight : Matrix Measurements Measurements ℝ) :
    rawInformation camera selector weight = compressedInformation camera selector weight := by
  simp only [rawInformation, compressedInformation, Matrix.transpose_mul, Matrix.mul_assoc]

omit [Fintype Navigation] in
theorem score_preserved (camera : Matrix Measurements Pose ℝ)
    (selector : Matrix Pose Navigation ℝ)
    (weight : Matrix Measurements Measurements ℝ)
    (residual : Matrix Measurements Unit ℝ) :
    rawScore camera selector weight residual = compressedScore camera selector weight residual := by
  simp only [rawScore, compressedScore, Matrix.transpose_mul, Matrix.mul_assoc]

omit [Fintype Navigation] in
theorem posterior_information_preserved (prior : Matrix Navigation Navigation ℝ)
    (camera : Matrix Measurements Pose ℝ) (selector : Matrix Pose Navigation ℝ)
    (weight : Matrix Measurements Measurements ℝ) :
    prior + rawInformation camera selector weight =
      prior + compressedInformation camera selector weight := by
  rw [information_preserved]

theorem common_posterior_correction_preserved (posterior : Matrix Navigation Navigation ℝ)
    (camera : Matrix Measurements Pose ℝ) (selector : Matrix Pose Navigation ℝ)
    (weight : Matrix Measurements Measurements ℝ)
    (residual : Matrix Measurements Unit ℝ) :
    posterior * rawScore camera selector weight residual =
      posterior * compressedScore camera selector weight residual := by
  rw [score_preserved]

omit [Fintype Navigation] in
theorem pose_statistic_preserves_score (camera : Matrix Measurements Pose ℝ)
    (selector : Matrix Pose Navigation ℝ) (weight : Matrix Measurements Measurements ℝ)
    (residual : Matrix Measurements Unit ℝ) (pose : Matrix Pose Unit ℝ)
    (hpose : (camera.transpose * weight * camera) * pose = camera.transpose * weight * residual) :
    (selector.transpose * (camera.transpose * weight * camera)) * pose =
      rawScore camera selector weight residual := by
  rw [Matrix.mul_assoc, hpose, score_preserved]
  rfl

omit [Fintype Measurements] in
theorem unobserved_direction_preserved (camera : Matrix Measurements Pose ℝ)
    (selector : Matrix Pose Navigation ℝ) (direction : Matrix Navigation Unit ℝ)
    (hunobserved : selector * direction = 0) : (camera * selector) * direction = 0 := by
  rw [Matrix.mul_assoc, hunobserved, Matrix.mul_zero]

end GNC.Estimation.VisualCompression
