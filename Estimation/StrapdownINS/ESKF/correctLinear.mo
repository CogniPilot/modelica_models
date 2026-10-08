within Estimation.StrapdownINS.ESKF;

function correctLinear
  "Vectorized Joseph-form correction in the local right-error tangent space"
  input State predicted;
  input Real residual[:];
  input Real H[size(residual, 1), TangentLength];
  input Real measurementCovariance[size(residual, 1), size(residual, 1)];
  input Real innovationGate = 0.0
    "Reject when NIS exceeds innovationGate * size(residual, 1);
     non-positive disables the gate";
  input Real attitudeGainAxis[3] = zeros(3)
    "Zero, the default, leaves the optimal gain alone. A UNIT body axis
     instead projects the three rotation rows of the gain onto that axis,
     so this measurement may rotate the state about it and about nothing
     else, while the rotation directions it is excluded from still carry
     their full sensitivity into the innovation covariance and into the
     Joseph update. That is a Schmidt, or consider, treatment: the excluded
     directions keep their uncertainty, the included one inherits the
     correlation the measurement really has with them, and the posterior
     stays consistent because the Joseph form below is valid for an
     ARBITRARY gain. It exists for a measurement whose Jacobian is honestly
     non-zero on a state the sensor must not be trusted to correct.";
  input Real measurementStateCrossCovariance[
    TangentLength, size(residual, 1)] =
    zeros(TangentLength, size(residual, 1))
    "Cov(error before correction, observation noise); zero for independent noise";
  input Boolean headingOnly = false
    "With a unit attitudeGainAxis, update only heading and gyro bias along that axis";
  input Boolean useSemiDirectBias = false;
  output State corrected;
  output Boolean accepted;
  output Integer rejectionReason
    "Estimation.StrapdownINS.Correction* code; Accepted when the
     correction was applied, and a named rejection cause otherwise";
  output Real normalizedInnovationSquared
    "Innovation residual squared after whitening by its predicted covariance";
protected
  Integer measurementLength = size(residual, 1);
  Boolean residualFinite;
  Boolean measurementCovarianceUsable;
  Boolean correlatedMeasurement;
  Boolean gatePassed;
  Real crossCovariance[TangentLength, size(residual, 1)];
  Real innovationCovariance[size(residual, 1), size(residual, 1)];
  Real augmentedRhs[size(residual, 1), TangentLength + 1];
  Real augmentedSolution[size(residual, 1), TangentLength + 1];
  Real gain[TangentLength, size(residual, 1)];
  Boolean factorized;
  TangentVector correction;
  Real josephFactor[TangentLength, TangentLength];
  Real posteriorCovariance[TangentLength, TangentLength];
  Real resetRotationJacobian[9, 9];
  Real resetJacobian[TangentLength, TangentLength];
  NominalState nominal;
  NominalState correctedNominal;
  Covariance correctedCovariance;
  Covariance correctedRoot;
  Real noiseStateRoot[TangentLength, size(residual, 1)];
  Real measurementColumns[size(residual, 1), TangentLength];
  Real measurementRoot[size(residual, 1), size(residual, 1)];
  Real innovationRoot[size(residual, 1), size(residual, 1)];
  Covariance priorRoot;
  Real whitened[size(residual, 1), TangentLength + 1];
  Boolean crossFactorized;
  Boolean noiseFactorized;
algorithm
  residualFinite := true;
  for row in 1:measurementLength loop
    residualFinite := residualFinite
      and abs(residual[row]) < FiniteMagnitudeLimit;
  end for;

  measurementCovarianceUsable := true;
  for row in 1:measurementLength loop
    measurementCovarianceUsable := measurementCovarianceUsable
      and measurementCovariance[row, row] > 0.0
      and measurementCovariance[row, row] < FiniteMagnitudeLimit;
    for column in 1:measurementLength loop
      measurementCovarianceUsable := measurementCovarianceUsable
        and abs(measurementCovariance[row, column]) < FiniteMagnitudeLimit;
    end for;
  end for;

  correlatedMeasurement := false;
  for row in 1:TangentLength loop
    for column in 1:measurementLength loop
      measurementCovarianceUsable := measurementCovarianceUsable
        and abs(measurementStateCrossCovariance[row, column]) < FiniteMagnitudeLimit;
      correlatedMeasurement := correlatedMeasurement
        or measurementStateCrossCovariance[row, column] <> 0.0;
    end for;
  end for;

  correctedRoot := predicted.covarianceRoot;
  measurementColumns := zeros(measurementLength, TangentLength);
  measurementRoot := zeros(measurementLength, measurementLength);
  if predicted.useSquareRootCovariance then
    (noiseStateRoot, crossFactorized) := LinearAlgebra.solveLower(
      predicted.covarianceRoot, measurementStateCrossCovariance);
    (measurementRoot, noiseFactorized) := LinearAlgebra.factorPSD(
      measurementCovariance - transpose(noiseStateRoot) * noiseStateRoot);
    measurementColumns := H * predicted.covarianceRoot
      + transpose(noiseStateRoot);
    crossCovariance := predicted.covarianceRoot * transpose(measurementColumns);
    innovationRoot := LinearAlgebra.covarianceRoot(
      cat(2, measurementColumns, measurementRoot));
    augmentedRhs := zeros(measurementLength, TangentLength + 1);
    augmentedRhs[:, 1:TangentLength] := transpose(crossCovariance);
    augmentedRhs[:, TangentLength + 1] := residual;
    (augmentedSolution, whitened, factorized) :=
      LinearAlgebra.solveCovarianceRoot(innovationRoot, augmentedRhs);
    factorized := factorized and crossFactorized and noiseFactorized;
    normalizedInnovationSquared := whitened[:, TangentLength + 1]
      * whitened[:, TangentLength + 1];
  else
    crossFactorized := true;
    noiseStateRoot := zeros(TangentLength, measurementLength);
    if correlatedMeasurement then
      (priorRoot, crossFactorized) := LinearAlgebra.factorPSD(predicted.covariance);
      (noiseStateRoot, noiseFactorized) := LinearAlgebra.solveLower(
        priorRoot, measurementStateCrossCovariance);
      crossFactorized := crossFactorized and noiseFactorized;
    end if;
    (measurementRoot, noiseFactorized) := LinearAlgebra.factorPSD(
      measurementCovariance
        - transpose(noiseStateRoot) * noiseStateRoot);
    measurementCovarianceUsable := measurementCovarianceUsable
      and crossFactorized and noiseFactorized;
    crossCovariance := predicted.covariance * transpose(H)
      + measurementStateCrossCovariance;
    innovationCovariance := LinearAlgebra.symmetrize(
      H * crossCovariance
        + transpose(measurementStateCrossCovariance) * transpose(H)
        + measurementCovariance);
    // One factorization serves both the gain solve S*K' = (P*H')' and the
    // whitened residual S^-1 * r needed for the innovation gate: append the
    // residual as one extra right-hand side.
    augmentedRhs := zeros(measurementLength, TangentLength + 1);
    augmentedRhs[:, 1:TangentLength] := transpose(crossCovariance);
    augmentedRhs[:, TangentLength + 1] := residual;
    (augmentedSolution, factorized) := LinearAlgebra.solveSPD(
      innovationCovariance, augmentedRhs);
    normalizedInnovationSquared :=
      residual * augmentedSolution[:, TangentLength + 1];
  end if;
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
    (gain, correction) := constrainGain(
      transpose(augmentedSolution[:, 1:TangentLength]), residual,
      attitudeGainAxis, headingOnly);

    nominal := NominalState(
      positionWorldEnu_m=predicted.positionWorldEnu_m,
      velocityWorldEnu_m_s=predicted.velocityWorldEnu_m_s,
      quaternionWorldBody=predicted.quaternionWorldBody,
      gyroscopeBiasBodyFlu_rad_s=predicted.gyroscopeBiasBodyFlu_rad_s,
      accelerometerBiasBodyFlu_m_s2=predicted.accelerometerBiasBodyFlu_m_s2);
    if useSemiDirectBias then
      correctedNominal := SemiDirectBias.inject(nominal, correction);
    else
      correctedNominal := inject(nominal, correction);
    end if;
    if predicted.useSquareRootCovariance then
      correctedRoot := LinearAlgebra.covarianceRoot(cat(2,
        predicted.covarianceRoot - gain * measurementColumns,
        gain * measurementRoot));
      resetJacobian := identity(TangentLength);
      if useSemiDirectBias then
        resetJacobian := SemiDirectBias.resetJacobian(correction);
      else
        resetRotationJacobian :=
          LieGroups.SE23.Quat.right_jacobian(correction[1:9]);
        resetJacobian := cat(1,
          cat(2, resetRotationJacobian, zeros(9, 6)),
          cat(2, zeros(6, 9), identity(6)));
      end if;
      correctedRoot := LinearAlgebra.covarianceRoot(resetJacobian * correctedRoot);
      correctedCovariance := LinearAlgebra.symmetrize(
        correctedRoot * transpose(correctedRoot));
    else
      josephFactor := identity(TangentLength) - gain * H;
      posteriorCovariance := LinearAlgebra.symmetrize(
        LinearAlgebra.josephUpdate(
          josephFactor,
          predicted.covariance,
          gain,
          measurementCovariance));
      if correlatedMeasurement then
        // Cov((I-KH)e - K v) includes both cross terms when Cov(e,v) != 0.
        // This generalized Joseph form remains valid for the effective gain
        // after the attitude trust limit and consider projection above.
        posteriorCovariance := LinearAlgebra.symmetrize(posteriorCovariance
          - josephFactor * measurementStateCrossCovariance * transpose(gain)
          - gain * transpose(measurementStateCrossCovariance)
            * transpose(josephFactor));
      end if;
      if useSemiDirectBias then
        resetJacobian := SemiDirectBias.resetJacobian(correction);
        correctedCovariance := LinearAlgebra.symmetrize(
          LinearAlgebra.transformCovariance(resetJacobian, posteriorCovariance));
      else
        // The reset Jacobian is the block diagonal diag(J, identity(6)) with
        // literal zeros off the diagonal, so the conjugation is done blockwise in
        // conjugateReset rather than by forming the 15x15 matrix and multiplying
        // it through: 54,000 -> 2,431 multiplies per call, same products, same
        // order. See conjugateReset for the exactness argument, the two
        // exceptional-value changes, and why it has to be a separate function.
        resetRotationJacobian := LieGroups.SE23.Quat.right_jacobian(correction[1:9]);
        correctedCovariance := LinearAlgebra.symmetrize(
          conjugateReset(
            resetRotationJacobian, posteriorCovariance));
      end if;
    end if;
  else
    // A rejected correction (factorization failure or gate) never
    // modifies the state.
    correctedNominal := NominalState(
      positionWorldEnu_m=predicted.positionWorldEnu_m,
      velocityWorldEnu_m_s=predicted.velocityWorldEnu_m_s,
      quaternionWorldBody=predicted.quaternionWorldBody,
      gyroscopeBiasBodyFlu_rad_s=predicted.gyroscopeBiasBodyFlu_rad_s,
      accelerometerBiasBodyFlu_m_s2=
        predicted.accelerometerBiasBodyFlu_m_s2);
    correctedCovariance := predicted.covariance;
  end if;
  corrected := State(
    positionWorldEnu_m=correctedNominal.positionWorldEnu_m,
    velocityWorldEnu_m_s=correctedNominal.velocityWorldEnu_m_s,
    quaternionWorldBody=correctedNominal.quaternionWorldBody,
    gyroscopeBiasBodyFlu_rad_s=
      correctedNominal.gyroscopeBiasBodyFlu_rad_s,
    accelerometerBiasBodyFlu_m_s2=
      correctedNominal.accelerometerBiasBodyFlu_m_s2,
    covariance=correctedCovariance,
    useSquareRootCovariance=predicted.useSquareRootCovariance,
    covarianceRoot=correctedRoot);
end correctLinear;
