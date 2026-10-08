import GNC.Estimation.InertialBias
import GNC.Analysis.LinearODE
import GNC.Dynamics.LieErrorReconstruction

/-! A finite-horizon SE_2(3) navigation-log envelope.

Bias matching is explicit. The augmented biased estimator is not silently
replaced by this special case. Physical error components have separate units;
the bounds are not an unscaled sum of metres, metres/second and radians.
-/
noncomputable section
namespace GNC.Estimation.LieErrorEnvelope
open Matrix InertialBias

theorem local_to_right_error (estimate : SE23) (localError : LogState) :
    groupExp (adjoint estimate localError) = estimate*groupExp localError*estimate⁻¹ := by
  apply SE23.toMatrix_injective
  rw [groupExp_toMatrix, hat_adjoint]
  have h := Matrix.exp_units_conj (SE23.matrixUnits estimate) (hat localError)
  change NormedSpace.exp
      (SE23.toMatrix estimate*hat localError*SE23.toMatrix estimate⁻¹) =
    SE23.toMatrix estimate*NormedSpace.exp (hat localError)*SE23.toMatrix estimate⁻¹ at h
  simpa only [SE23.toMatrix_mul, groupExp_toMatrix] using h

theorem adjoint_envelope (estimate : SE23) (localError : LogState)
    {position velocity attitude : ℝ} (hp : enorm (localError 0) ≤ position)
    (hv : enorm (localError 1) ≤ velocity) (ha : enorm (localError 2) ≤ attitude) :
    enorm (adjoint estimate localError 0) ≤ position+enorm estimate.pos*attitude ∧
      enorm (adjoint estimate localError 1) ≤ velocity+enorm estimate.vel*attitude ∧
      enorm (adjoint estimate localError 2) ≤ attitude := by
  have hr i := rotate_enorm estimate.rot (localError i)
  have hposition := (cross_enorm_le estimate.pos (rotate estimate.rot (localError 2))).trans
    (mul_le_mul_of_nonneg_left ((hr 2).le.trans ha) (enorm_nonneg estimate.pos))
  have hvelocity := (cross_enorm_le estimate.vel (rotate estimate.rot (localError 2))).trans
    (mul_le_mul_of_nonneg_left ((hr 2).le.trans ha) (enorm_nonneg estimate.vel))
  refine ⟨?_, ?_, ?_⟩
  · change enorm (rotate estimate.rot (localError 0)+
      estimate.pos ⨯₃ rotate estimate.rot (localError 2)) ≤ _
    exact (enorm_add_le _ _).trans (add_le_add ((hr 0).le.trans hp) hposition)
  · change enorm (rotate estimate.rot (localError 1)+
      estimate.vel ⨯₃ rotate estimate.rot (localError 2)) ≤ _
    exact (enorm_add_le _ _).trans (add_le_add ((hr 1).le.trans hv) hvelocity)
  · exact (hr 2).le.trans ha

theorem local_physical_translation_bound (truth estimate : SE23) (localError : LogState)
    {position velocity : ℝ} (hstate : truth = estimate*groupExp localError)
    (hchart : enorm (localError 2) < 2*Real.pi)
    (hp : enorm (localError 0) ≤ position) (hv : enorm (localError 1) ≤ velocity) :
    enorm (truth.pos-estimate.pos) ≤ position ∧
      enorm (truth.vel-estimate.vel) ≤ velocity := by
  rw [hstate]
  constructor
  · simpa [SE23.mul_pos, groupExp] using
      ((rotate_enorm estimate.rot (Jacobian.leftAt (localError 2) (localError 0))).le.trans
        ((leftAt_nonexpansive _ _ hchart).trans hp))
  · simpa [SE23.mul_vel, groupExp] using
      ((rotate_enorm estimate.rot (Jacobian.leftAt (localError 2) (localError 1))).le.trans
        ((leftAt_nonexpansive _ _ hchart).trans hv))

theorem right_physical_translation_bound (truth estimate : SE23) (rightError : LogState)
    {position velocity attitude : ℝ} (hstate : truth*estimate⁻¹ = groupExp rightError)
    (hchart : enorm (rightError 2) < 2*Real.pi)
    (hp : enorm (rightError 0) ≤ position) (hv : enorm (rightError 1) ≤ velocity)
    (ha : enorm (rightError 2) ≤ attitude) :
    enorm (truth.pos-estimate.pos) ≤ position+enorm estimate⁻¹.pos*attitude ∧
      enorm (truth.vel-estimate.vel) ≤ velocity+enorm estimate⁻¹.vel*attitude := by
  have hlocal : truth = estimate*groupExp (adjoint estimate⁻¹ rightError) := by
    calc
      truth = (truth*estimate⁻¹)*estimate := by group
      _ = groupExp rightError*estimate := by rw [hstate]
      _ = estimate*groupExp (adjoint estimate⁻¹ rightError) := by
        rw [local_to_right_error]
        group
  have hangle : enorm (adjoint estimate⁻¹ rightError 2) = enorm (rightError 2) := by
    exact rotate_enorm estimate⁻¹.rot (rightError 2)
  have bounds := adjoint_envelope estimate⁻¹ rightError hp hv ha
  exact local_physical_translation_bound truth estimate _ hlocal (hangle.trans_lt hchart)
    bounds.1 bounds.2.1

def navigationFlow (gravity : Vec3) (initial : LogState) (time : ℝ) : LogState :=
  ![initial 0+time • initial 1+(time^2/2) • (gravity ⨯₃ initial 2),
    initial 1+time • (gravity ⨯₃ initial 2), initial 2]

theorem matched_bias_log_derivative {X H : ℝ → SE23} {error : ℝ → LogState}
    {rate input bias : LogState} {gravity : Vec3} {time : ℝ}
    (hX : HasDerivAt (fun s => SE23.toMatrix (X s))
      (spacecraftDerivative (X time) (input-bias) gravity) time)
    (hH : HasDerivAt (fun s => SE23.toMatrix (H s))
      (spacecraftDerivative (H time) (input-bias) gravity) time)
    (he : HasDerivAt error rate time)
    (hlog : ∀ s, X s*(H s)⁻¹ = groupExp (error s))
    (hchart : enorm (error time 2) < 2*Real.pi) :
    rate = navigationDrift gravity (error time) := by
  have h := navigation_log_equation hX hH he hlog hchart
  simpa [tangentBias, adjoint, Jacobian.blockRightInverse, Jacobian.blockInverse,
    Jacobian.inverseAt, Jacobian.Q] using h

theorem navigationFlow_derivative (gravity : Vec3) (initial : LogState) (time : ℝ) :
    HasDerivAt (navigationFlow gravity initial)
      (navigationDrift gravity (navigationFlow gravity initial time)) time := by
  have hv := (hasDerivAt_id time).smul_const (initial 1)
  have ha := (((hasDerivAt_id time).pow 2).div_const 2).smul_const
    (gravity ⨯₃ initial 2)
  have hg := (hasDerivAt_id time).smul_const (gravity ⨯₃ initial 2)
  apply hasDerivAt_pi.mpr
  intro i
  fin_cases i
  · change HasDerivAt
      (fun s => initial 0+s • initial 1+(s^2/2) • (gravity ⨯₃ initial 2))
      (initial 1+time • (gravity ⨯₃ initial 2)) time
    convert ((hasDerivAt_const time (initial 0)).add hv).add ha using 1
    simp
  · change HasDerivAt (fun s => initial 1+s • (gravity ⨯₃ initial 2))
      (gravity ⨯₃ initial 2) time
    simpa only [Pi.add_apply, zero_add, one_smul] using
      (hasDerivAt_const time (initial 1)).add hg
  · change HasDerivAt (fun _ : ℝ => initial 2) 0 time
    exact hasDerivAt_const time (initial 2)

theorem navigationFlow_initial (gravity : Vec3) (initial : LogState) :
    navigationFlow gravity initial 0 = initial := by
  ext i j
  fin_cases i <;> simp [navigationFlow]

def navigationOperator (gravity : Vec3) : LogState →L[ℝ] LogState :=
  LinearMap.toContinuousLinearMap {
    toFun := navigationDrift gravity
    map_add' := by
      intro x y
      ext i j
      fin_cases i <;> simp [navigationDrift, Pi.add_apply, map_add]
    map_smul' := by
      intro c x
      ext i j
      fin_cases i <;> simp [navigationDrift, Pi.smul_apply, map_smul] }

theorem navigation_trajectory_eq (error : ℝ → LogState) (gravity : Vec3)
    {left right : ℝ} (hzero : (0:ℝ) ∈ Set.Ioo left right)
    (hode : ∀ s ∈ Set.Ioo left right,
      HasDerivAt error (navigationDrift gravity (error s)) s) :
    Set.EqOn error (navigationFlow gravity (error 0)) (Set.Ioo left right) := by
  exact GNC.LinearODE.unique_continuous (fun _ => navigationOperator gravity)
    continuous_const hode (fun s _ => navigationFlow_derivative gravity (error 0) s)
    hzero (navigationFlow_initial gravity (error 0)).symm

theorem navigationFlow_envelope (gravity : Vec3) (initial : LogState)
    {time position velocity attitude gravityBound : ℝ} (ht : 0 ≤ time)
    (hp : enorm (initial 0) ≤ position) (hv : enorm (initial 1) ≤ velocity)
    (ha : enorm (initial 2) ≤ attitude) (hg : enorm gravity ≤ gravityBound) :
    enorm (navigationFlow gravity initial time 0) ≤
        position+time*velocity+time^2/2*gravityBound*attitude ∧
      enorm (navigationFlow gravity initial time 1) ≤
        velocity+time*gravityBound*attitude ∧
      enorm (navigationFlow gravity initial time 2) ≤ attitude := by
  have hG := (enorm_nonneg gravity).trans hg
  have hcross := (cross_enorm_le gravity (initial 2)).trans
    (mul_le_mul hg ha (enorm_nonneg _) hG)
  have htv := mul_le_mul_of_nonneg_left hv ht
  have hta := mul_le_mul_of_nonneg_left hcross ht
  have htp := mul_le_mul_of_nonneg_left hcross (by positivity : 0 ≤ time^2/2)
  have habs : |time| = time := abs_of_nonneg ht
  have hsquare : |time^2/2| = time^2/2 := abs_of_nonneg (by positivity)
  refine ⟨?_, ?_, ?_⟩
  · change enorm (initial 0+time • initial 1+(time^2/2) • (gravity ⨯₃ initial 2)) ≤ _
    have h := (enorm_add_le (initial 0+time • initial 1)
      ((time^2/2) • (gravity ⨯₃ initial 2))).trans
        (add_le_add (enorm_add_le _ _) le_rfl)
    rw [enorm_smul, enorm_smul, habs, hsquare] at h
    nlinarith
  · change enorm (initial 1+time • (gravity ⨯₃ initial 2)) ≤ _
    have h := enorm_add_le (initial 1) (time • (gravity ⨯₃ initial 2))
    rw [enorm_smul, habs] at h
    nlinarith
  · simpa [navigationFlow] using ha

theorem navigation_trajectory_envelope (error : ℝ → LogState) (gravity : Vec3)
    {left right time position velocity attitude gravityBound : ℝ}
    (hzero : (0:ℝ) ∈ Set.Ioo left right)
    (hode : ∀ s ∈ Set.Ioo left right,
      HasDerivAt error (navigationDrift gravity (error s)) s)
    (ht : 0 ≤ time) (hinterval : time ∈ Set.Ioo left right)
    (hp : enorm (error 0 0) ≤ position) (hv : enorm (error 0 1) ≤ velocity)
    (ha : enorm (error 0 2) ≤ attitude) (hg : enorm gravity ≤ gravityBound) :
    enorm (error time 0) ≤ position+time*velocity+time^2/2*gravityBound*attitude ∧
      enorm (error time 1) ≤ velocity+time*gravityBound*attitude ∧
      enorm (error time 2) ≤ attitude := by
  have heq := navigation_trajectory_eq error gravity hzero hode hinterval
  rw [heq]
  exact navigationFlow_envelope gravity (error 0) ht hp hv ha hg

theorem enorm_growth (path rate : ℝ → Vec3) {horizon bound : ℝ}
    (hderivative : ∀ s ∈ Set.Icc (0:ℝ) horizon, HasDerivAt path (rate s) s)
    (hbound : ∀ s ∈ Set.Icc (0:ℝ) horizon, enorm (rate s) ≤ bound) :
    ∀ s ∈ Set.Icc (0:ℝ) horizon, enorm (path s) ≤ enorm (path 0)+bound*s := by
  let embedding := (PiLp.continuousLinearEquiv 2 ℝ (fun _ : Fin 3 => ℝ)).symm
  have hd s hs := embedding.toContinuousLinearMap.hasFDerivAt.comp_hasDerivAt s
    (hderivative s hs)
  have hm := norm_image_sub_le_of_norm_deriv_le_segment'
    (fun s hs => (hd s hs).hasDerivWithinAt)
    (fun s hs => hbound s ⟨hs.1,hs.2.le⟩)
  intro s hs
  have h := hm s hs
  change enorm (path s-path 0) ≤ bound*(s-0) at h
  have hn := enorm_add_le (path s-path 0) (path 0)
  rw [sub_add_cancel] at hn
  linarith

theorem forced_navigation_envelope (error forcing : ℝ → LogState) (gravity : Vec3)
    {horizon position velocity attitude gravityBound forcingPosition forcingVelocity
      forcingAttitude : ℝ}
    (ht : 0 ≤ horizon)
    (hode : ∀ s ∈ Set.Icc (0:ℝ) horizon,
      HasDerivAt error (navigationDrift gravity (error s)+forcing s) s)
    (hp : enorm (error 0 0) ≤ position) (hv : enorm (error 0 1) ≤ velocity)
    (ha : enorm (error 0 2) ≤ attitude) (hg : enorm gravity ≤ gravityBound)
    (hwp : ∀ s ∈ Set.Icc (0:ℝ) horizon, enorm (forcing s 0) ≤ forcingPosition)
    (hwv : ∀ s ∈ Set.Icc (0:ℝ) horizon, enorm (forcing s 1) ≤ forcingVelocity)
    (hwa : ∀ s ∈ Set.Icc (0:ℝ) horizon, enorm (forcing s 2) ≤ forcingAttitude) :
    let attitudeBound := attitude+horizon*forcingAttitude
    let velocityBound := velocity+horizon*(gravityBound*attitudeBound+forcingVelocity)
    enorm (error horizon 0) ≤ position+horizon*(velocityBound+forcingPosition) ∧
      enorm (error horizon 1) ≤ velocityBound ∧
      enorm (error horizon 2) ≤ attitudeBound := by
  have hG := (enorm_nonneg gravity).trans hg
  have hwA : 0 ≤ forcingAttitude :=
    (enorm_nonneg (forcing 0 2)).trans (hwa 0 ⟨le_rfl,ht⟩)
  have hd (i : Fin 3) s hs := hasDerivAt_pi.mp (hode s hs) i
  have hangle := enorm_growth (fun s => error s 2) (fun s => forcing s 2)
    (fun s hs => by simpa [navigationDrift] using hd 2 s hs) hwa
  have hangleUniform : ∀ s ∈ Set.Icc (0:ℝ) horizon,
      enorm (error s 2) ≤ attitude+horizon*forcingAttitude := by
    intro s hs
    have h := hangle s hs
    have hm := mul_le_mul_of_nonneg_right hs.2 hwA
    nlinarith
  have hvelocityRate : ∀ s ∈ Set.Icc (0:ℝ) horizon,
      enorm (gravity ⨯₃ error s 2+forcing s 1) ≤
        gravityBound*(attitude+horizon*forcingAttitude)+forcingVelocity := by
    intro s hs
    exact (enorm_add_le _ _).trans (add_le_add
      ((cross_enorm_le _ _).trans (mul_le_mul hg (hangleUniform s hs)
        (enorm_nonneg _) hG)) (hwv s hs))
  have hvelocity := enorm_growth (fun s => error s 1)
    (fun s => gravity ⨯₃ error s 2+forcing s 1)
    (fun s hs => by simpa [navigationDrift] using hd 1 s hs) hvelocityRate
  have hVRate : 0 ≤ gravityBound*(attitude+horizon*forcingAttitude)+forcingVelocity :=
    (enorm_nonneg _).trans (hvelocityRate 0 ⟨le_rfl,ht⟩)
  have hvelocityUniform : ∀ s ∈ Set.Icc (0:ℝ) horizon,
      enorm (error s 1) ≤
        velocity+horizon*(gravityBound*(attitude+horizon*forcingAttitude)+forcingVelocity) := by
    intro s hs
    have h := hvelocity s hs
    have hm := mul_le_mul_of_nonneg_right hs.2 hVRate
    nlinarith
  have hpositionRate : ∀ s ∈ Set.Icc (0:ℝ) horizon,
      enorm (error s 1+forcing s 0) ≤
        velocity+horizon*(gravityBound*(attitude+horizon*forcingAttitude)+forcingVelocity)+
          forcingPosition := by
    intro s hs
    exact (enorm_add_le _ _).trans (add_le_add (hvelocityUniform s hs) (hwp s hs))
  have hposition := enorm_growth (fun s => error s 0) (fun s => error s 1+forcing s 0)
    (fun s hs => by simpa [navigationDrift] using hd 0 s hs) hpositionRate
  have h := hposition horizon ⟨ht,le_rfl⟩
  refine ⟨?_, hvelocityUniform horizon ⟨ht,le_rfl⟩, hangleUniform horizon ⟨ht,le_rfl⟩⟩
  nlinarith

def biasForcing (error : ℝ → LogState) (estimate : ℝ → SE23)
    (bias estimatedBias : ℝ → LogState) (time : ℝ) : LogState :=
  tangentBias (error time) (adjoint (estimate time) (bias time-estimatedBias time))

theorem biased_navigation_envelope (truth estimate : ℝ → SE23)
    (error rate input bias estimatedBias : ℝ → LogState) (gravity : Vec3)
    {horizon position velocity attitude gravityBound forcingPosition forcingVelocity
      forcingAttitude : ℝ} (ht : 0 ≤ horizon)
    (htruth : ∀ s ∈ Set.Icc (0:ℝ) horizon,
      HasDerivAt (fun t => SE23.toMatrix (truth t))
        (spacecraftDerivative (truth s) (input s-bias s) gravity) s)
    (hestimate : ∀ s ∈ Set.Icc (0:ℝ) horizon,
      HasDerivAt (fun t => SE23.toMatrix (estimate t))
        (spacecraftDerivative (estimate s) (input s-estimatedBias s) gravity) s)
    (herror : ∀ s ∈ Set.Icc (0:ℝ) horizon, HasDerivAt error (rate s) s)
    (hlog : ∀ s, truth s*(estimate s)⁻¹ = groupExp (error s))
    (hchart : ∀ s ∈ Set.Icc (0:ℝ) horizon, enorm (error s 2) < 2*Real.pi)
    (hp : enorm (error 0 0) ≤ position) (hv : enorm (error 0 1) ≤ velocity)
    (ha : enorm (error 0 2) ≤ attitude) (hg : enorm gravity ≤ gravityBound)
    (hwp : ∀ s ∈ Set.Icc (0:ℝ) horizon,
      enorm (biasForcing error estimate bias estimatedBias s 0) ≤ forcingPosition)
    (hwv : ∀ s ∈ Set.Icc (0:ℝ) horizon,
      enorm (biasForcing error estimate bias estimatedBias s 1) ≤ forcingVelocity)
    (hwa : ∀ s ∈ Set.Icc (0:ℝ) horizon,
      enorm (biasForcing error estimate bias estimatedBias s 2) ≤ forcingAttitude) :
    let attitudeBound := attitude+horizon*forcingAttitude
    let velocityBound := velocity+horizon*(gravityBound*attitudeBound+forcingVelocity)
    enorm (error horizon 0) ≤ position+horizon*(velocityBound+forcingPosition) ∧
      enorm (error horizon 1) ≤ velocityBound ∧
      enorm (error horizon 2) ≤ attitudeBound := by
  apply forced_navigation_envelope error (biasForcing error estimate bias estimatedBias)
    gravity ht ?_ hp hv ha hg hwp hwv hwa
  intro s hs
  have heq := navigation_log_equation (htruth s hs) (hestimate s hs)
    (herror s hs) hlog (hchart s hs)
  simpa only [heq, biasForcing] using herror s hs

end GNC.Estimation.LieErrorEnvelope
