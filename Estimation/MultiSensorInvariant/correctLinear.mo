within Estimation.MultiSensorInvariant;

function correctLinear
  "Vectorized Joseph-form correction in the invariant tangent space"
  input Estimation.MultiSensorInvariant.State predicted;
  input Real residual[:];
  input Real H[size(residual, 1), TangentLength];
  input Real measurementCovariance[size(residual, 1), size(residual, 1)];
  input Real innovationGate = 0.0
    "Reject when NIS exceeds innovationGate * size(residual, 1);
     non-positive disables the gate";
  output Estimation.MultiSensorInvariant.State corrected;
  output Boolean accepted;
  output Integer rejectionReason
    "Estimation.MultiSensorInvariant.Correction* code; Accepted when the
     correction was applied, and a named rejection cause otherwise";
protected
  Integer measurementLength = size(residual, 1);
  Boolean residualFinite;
  Boolean measurementCovarianceUsable;
  Boolean gatePassed;
  Real attitudeCorrection_rad;
  Real trustScale;
  Real crossCovariance[TangentLength, size(residual, 1)];
  Real innovationCovariance[size(residual, 1), size(residual, 1)];
  Real augmentedRhs[size(residual, 1), TangentLength + 1];
  Real augmentedSolution[size(residual, 1), TangentLength + 1];
  Real gainTranspose[size(residual, 1), TangentLength];
  Real gain[TangentLength, size(residual, 1)];
  Real normalizedInnovationSquared;
  Boolean factorized;
  Estimation.MultiSensorInvariant.TangentVector correction;
  Real josephFactor[TangentLength, TangentLength];
  Real posteriorCovariance[TangentLength, TangentLength];
  Real resetRotationJacobian[9, 9];
  Estimation.MultiSensorInvariant.NominalState nominal;
  Estimation.MultiSensorInvariant.NominalState correctedNominal;
  Estimation.MultiSensorInvariant.Covariance correctedCovariance;
algorithm
  // AFFIRMATIVE INPUT ADMISSION, evaluated before anything downstream can
  // launder a bad value into a plausible-looking number.
  //
  // measurementLength is 2, 3, or 6, so the residual scan is at most six
  // comparisons and the covariance scan at most 42, against the
  // O(TangentLength^3) factorization below: unmeasurable. The
  // factorization itself stays unconditional so this function's
  // worst-case execution time is unchanged -- a data-dependent early-out
  // could only shorten it, and a flight schedule is sized on the worst
  // case, so shortening buys nothing and costs a branch.
  residualFinite := true;
  for i in 1:measurementLength loop
    residualFinite := residualFinite
      and abs(residual[i]) < FiniteMagnitudeLimit;
  end for;

  // The sensor-reported measurement covariance R is an INPUT from a
  // driver, so it is checked like any other measurement rather than
  // trusted. Two failure modes were witnessed reaching the Cholesky: a
  // NaN entry, and a non-positive diagonal. A non-positive R is not
  // merely unusual, it is unphysical -- it claims negative measurement
  // noise -- and it subtracts from the innovation covariance, which can
  // leave S positive definite (so it still factors) while driving the
  // gain above unity, so the correction overshoots the measurement it is
  // supposed to be pulled toward. Requiring every entry finite and every
  // diagonal entry strictly positive rejects both affirmatively.
  measurementCovarianceUsable := true;
  for i in 1:measurementLength loop
    measurementCovarianceUsable := measurementCovarianceUsable
      and measurementCovariance[i, i] > 0.0
      and measurementCovariance[i, i] < FiniteMagnitudeLimit;
    for j in 1:measurementLength loop
      measurementCovarianceUsable := measurementCovarianceUsable
        and abs(measurementCovariance[i, j]) < FiniteMagnitudeLimit;
    end for;
  end for;

  crossCovariance := predicted.covariance * transpose(H);
  innovationCovariance := LinearAlgebra.symmetrize(
    H * crossCovariance + measurementCovariance);
  // One factorization serves both the gain solve S*K' = (P*H')' and the
  // whitened residual S^-1 * r needed for the innovation gate: append the
  // residual as one extra right-hand side.
  augmentedRhs := zeros(measurementLength, TangentLength + 1);
  augmentedRhs[:, 1:TangentLength] := transpose(crossCovariance);
  augmentedRhs[:, TangentLength + 1] := residual;
  (augmentedSolution, factorized) := LinearAlgebra.solveSPD(
    innovationCovariance, augmentedRhs);
  // Scalar normalized innovation squared (NIS) chi-square gate:
  // r' * S^-1 * r is chi-square distributed with size(residual, 1)
  // degrees of freedom when the filter is consistent, so a residual
  // grossly inconsistent with the predicted innovation covariance is
  // rejected even though S itself factors fine. The threshold is
  // expressed per degree of freedom so one configurable number covers
  // the 2-, 3-, and 6-dimensional corrections used here. The whitened
  // residual is all zeros when the factorization failed, so the gate
  // terms below are well defined on every path.
  normalizedInnovationSquared :=
    residual * augmentedSolution[:, TangentLength + 1];
  //
  // AFFIRMATIVE acceptance. The previous form computed a REJECTION
  // predicate -- `gateRejected := factorized and gate > 0 and NIS >
  // gate * m` -- and accepted everything it had not explicitly rejected.
  // Every comparison against a NaN is false, so a NaN NIS made
  // `gateRejected` false and `accepted := factorized and not gateRejected`
  // TRUE: a non-finite measurement was applied to the state, and because
  // acceptance also resets the rejection clock, the resulting all-NaN
  // state could never trigger the automatic recovery below either.
  // Stating the condition a measurement must PROVE before it is allowed
  // to move the state makes every non-finite input fail INTO rejection.
  //
  // The NIS must be finite AND non-negative, not merely non-negative:
  // r' * S^-1 * r is non-negative for any positive definite S and any
  // finite residual, so `>= 0.0` costs one comparison and is false for a
  // NaN -- but it is true for +Inf, and with `innovationGate <= 0` (a
  // documented, supported configuration that disables the chi-square
  // test) nothing else would have bounded it. Both terms sit outside the
  // gate guard so that disabling the gate cannot re-open a non-finite
  // entry path.
  gatePassed := normalizedInnovationSquared >= 0.0
    and normalizedInnovationSquared < FiniteMagnitudeLimit
    and (innovationGate <= 0.0
      or normalizedInnovationSquared <= innovationGate * measurementLength);
  accepted := factorized and residualFinite and measurementCovarianceUsable
    and gatePassed;
  // Every rejection carries a named cause. Ordered by depth: an unusable
  // measurement covariance or a non-finite residual makes the
  // factorization and the gate result meaningless, so those are reported
  // first and the deeper verdicts are not attributed.
  rejectionReason := if accepted then CorrectionAccepted
    elseif not residualFinite then CorrectionRejectedNotFinite
    elseif not measurementCovarianceUsable then
      CorrectionRejectedCovarianceUnusable
    elseif not factorized then CorrectionRejectedFactorization
    else CorrectionRejectedGate;
  if accepted then
    gainTranspose := augmentedSolution[:, 1:TangentLength];
    gain := transpose(gainTranspose);
    correction := gain * residual;

    // TRUST REGION ON THE ROTATION ONLY.
    //
    // A correction whose attitude part exceeds the rotation this
    // linearization can represent has that part limited; every other
    // block -- position, velocity, and both biases -- is applied IN FULL.
    //
    // Limiting the whole 15-vector instead was a ratchet, and a bad one.
    // Scaling everything by the attitude factor starves the states the
    // measurement actually determines: they move a few percent, so the
    // residual barely changes, so the next tick requests the same large
    // rotation again, and the attitude walks a fixed step per tick along
    // a direction that is only a descent direction when taken whole.
    // Measured on truthful flow-only acquisition: a clean monotone
    // 0.1, 0.2, ... 1.43 rad climb, position error -1142 m at 4 m/s with
    // estimate.valid still 1, and unbounded velocity divergence at the
    // deployed 1 kHz aiding rate -- while the unmodified filter tracked
    // to +0.062 m. It was rate-dependent, harmless at 10 Hz and
    // catastrophic at 1 kHz, which is exactly the wiring that ships.
    //
    // Letting the well-observed blocks converge is what breaks the loop:
    // position and velocity reach the measurement on the first step, the
    // residual collapses, and the next tick therefore asks for a SMALL
    // rotation rather than the same large one. The rotation still arrives
    // in installments, so where attitude is genuinely observed -- mocap --
    // it converges in exact installments as before.
    //
    // The covariance is blended toward the prior by the same factor. It
    // contracts less than a full update would, which is deliberately
    // conservative: having applied only part of the requested rotation,
    // claiming the full reduction in uncertainty would leave the filter
    // more confident than the state it actually holds, and that
    // overconfidence is what closes the gate against the samples that
    // would finish the job. A convex combination of two positive
    // semi-definite matrices is positive semi-definite.
    //
    // The guard is an `if` rather than an unconditional scale by 1.0 so
    // that the nominal path -- where the clamp never binds -- executes
    // the identical arithmetic it did before this existed, and the
    // emitted flight code is bit-identical on every healthy tick.
    attitudeCorrection_rad := sqrt(correction[7] * correction[7]
      + correction[8] * correction[8] + correction[9] * correction[9]);
    trustScale := 1.0;
    if attitudeCorrection_rad > MaxAttitudeCorrection_rad then
      trustScale := MaxAttitudeCorrection_rad / attitudeCorrection_rad;
      correction[7] := trustScale * correction[7];
      correction[8] := trustScale * correction[8];
      correction[9] := trustScale * correction[9];
    end if;

    nominal := NominalState(
      positionWorldEnu_m=predicted.positionWorldEnu_m,
      velocityWorldEnu_m_s=predicted.velocityWorldEnu_m_s,
      quaternionWorldBody=predicted.quaternionWorldBody,
      gyroscopeBiasBodyFlu_rad_s=predicted.gyroscopeBiasBodyFlu_rad_s,
      accelerometerBiasBodyFlu_m_s2=predicted.accelerometerBiasBodyFlu_m_s2);
    correctedNominal := inject(nominal, correction);
    josephFactor := identity(TangentLength) - gain * H;
    posteriorCovariance := LinearAlgebra.symmetrize(
      LinearAlgebra.josephUpdate(
        josephFactor,
        predicted.covariance,
        gain,
        measurementCovariance));
    // The reset Jacobian is the block diagonal diag(J, identity(6)) with
    // literal zeros off the diagonal, so the conjugation is done blockwise in
    // conjugateReset rather than by forming the 15x15 matrix and multiplying
    // it through: 54,000 -> 2,431 multiplies per call, same products, same
    // order. See conjugateReset for the exactness argument, the two
    // exceptional-value changes, and why it has to be a separate function.
    resetRotationJacobian := LieGroups.SE23.Quat.right_jacobian(correction[1:9]);
    correctedCovariance := LinearAlgebra.symmetrize(
      Estimation.MultiSensorInvariant.conjugateReset(
        resetRotationJacobian, posteriorCovariance));
    if trustScale < 1.0 then
      correctedCovariance := LinearAlgebra.symmetrize(
        trustScale * correctedCovariance
          + (1.0 - trustScale) * predicted.covariance);
    end if;
  else
    // A rejected correction (factorization failure or gate) never
    // modifies the state.
    correctedNominal := Estimation.MultiSensorInvariant.NominalState(
      positionWorldEnu_m=predicted.positionWorldEnu_m,
      velocityWorldEnu_m_s=predicted.velocityWorldEnu_m_s,
      quaternionWorldBody=predicted.quaternionWorldBody,
      gyroscopeBiasBodyFlu_rad_s=predicted.gyroscopeBiasBodyFlu_rad_s,
      accelerometerBiasBodyFlu_m_s2=
        predicted.accelerometerBiasBodyFlu_m_s2);
    correctedCovariance := predicted.covariance;
  end if;
  corrected := Estimation.MultiSensorInvariant.State(
    positionWorldEnu_m=correctedNominal.positionWorldEnu_m,
    velocityWorldEnu_m_s=correctedNominal.velocityWorldEnu_m_s,
    quaternionWorldBody=correctedNominal.quaternionWorldBody,
    gyroscopeBiasBodyFlu_rad_s=
      correctedNominal.gyroscopeBiasBodyFlu_rad_s,
    accelerometerBiasBodyFlu_m_s2=
      correctedNominal.accelerometerBiasBodyFlu_m_s2,
    covariance=correctedCovariance);
end correctLinear;
