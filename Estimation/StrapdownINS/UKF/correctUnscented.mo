within Estimation.StrapdownINS.UKF;

function correctUnscented
  "Unscented measurement correction in local navigation coordinates"
  input Estimation.StrapdownINS.UKF.State predicted;
  input Real sigmaMeasurement[:, SigmaCount]
    "Measurement function evaluated at sigma states from predicted covariance";
  input Real measurement[size(sigmaMeasurement, 1)];
  input Real measurementCovariance[size(sigmaMeasurement, 1),
    size(sigmaMeasurement, 1)];
  input Real innovationGate = 0.0;
  output Estimation.StrapdownINS.UKF.State corrected;
  output Boolean accepted;
  output Integer rejectionReason;
  output Real normalizedInnovationSquared;
protected
  Integer measurementLength = size(sigmaMeasurement, 1);
  Real nominal[16];
  Real sigmaState[16, SigmaCount];
  Real correctedNominal[16];
  Real sigma[TangentLength, SigmaCount];
  Boolean sigmaFactorized;
  Boolean innovationFactorized;
  Boolean inputsUsable;
  Real measurementMean[measurementLength];
  Real measurementDeviation[measurementLength];
  Real stateDeviation[TangentLength];
  Real residual[measurementLength];
  Real innovationCovariance[measurementLength, measurementLength];
  Real crossCovariance[TangentLength, measurementLength];
  Real augmentedRhs[measurementLength, TangentLength + 1];
  Real augmentedSolution[measurementLength, TangentLength + 1];
  Real gain[TangentLength, measurementLength];
  Real correction[TangentLength];
  Real attitudeCorrection;
  Real trustScale;
  Real posteriorCovariance[TangentLength, TangentLength];
  Real correctedCovariance[TangentLength, TangentLength];
algorithm
  nominal := stateVector(predicted);
  (sigma, sigmaFactorized) := sigmaTangents(predicted.covariance);
  for index in 1:SigmaCount loop
    sigmaState[:, index] := injectVector(
      nominal, sigma[:, index]);
  end for;

  measurementMean := zeros(measurementLength);
  for index in 2:SigmaCount loop
    measurementMean := measurementMean
      + SigmaWeight * sigmaMeasurement[:, index];
  end for;
  residual := measurement - measurementMean;
  measurementDeviation := sigmaMeasurement[:, 1] - measurementMean;
  innovationCovariance := measurementCovariance;
  for row in 1:measurementLength loop
    for column in 1:measurementLength loop
      innovationCovariance[row, column] :=
        innovationCovariance[row, column]
        + CentralCovarianceWeight * measurementDeviation[row]
          * measurementDeviation[column];
    end for;
  end for;
  crossCovariance := zeros(TangentLength, measurementLength);
  for index in 2:SigmaCount loop
    stateDeviation := localErrorVector(nominal, sigmaState[:, index]);
    measurementDeviation := sigmaMeasurement[:, index] - measurementMean;
    for row in 1:TangentLength loop
      for column in 1:measurementLength loop
        crossCovariance[row, column] := crossCovariance[row, column]
          + SigmaWeight * stateDeviation[row]
            * measurementDeviation[column];
      end for;
    end for;
    for row in 1:measurementLength loop
      for column in 1:measurementLength loop
        innovationCovariance[row, column] :=
          innovationCovariance[row, column]
          + SigmaWeight * measurementDeviation[row]
            * measurementDeviation[column];
      end for;
    end for;
  end for;
  innovationCovariance := LinearAlgebra.symmetrize(innovationCovariance);
  augmentedRhs := zeros(measurementLength, TangentLength + 1);
  augmentedRhs[:, 1:TangentLength] := transpose(crossCovariance);
  augmentedRhs[:, TangentLength + 1] := residual;
  (augmentedSolution, innovationFactorized) := LinearAlgebra.solveSPD(
    innovationCovariance, augmentedRhs);
  normalizedInnovationSquared := residual
    * augmentedSolution[:, TangentLength + 1];

  inputsUsable := true;
  for row in 1:measurementLength loop
    inputsUsable := inputsUsable
      and abs(residual[row]) < Estimation.StrapdownINS.ESKF.FiniteMagnitudeLimit
      and measurementCovariance[row, row] > 0.0;
    for column in 1:measurementLength loop
      inputsUsable := inputsUsable
        and abs(measurementCovariance[row, column])
          < Estimation.StrapdownINS.ESKF.FiniteMagnitudeLimit;
    end for;
  end for;
  accepted := sigmaFactorized and innovationFactorized and inputsUsable
    and normalizedInnovationSquared >= 0.0
    and normalizedInnovationSquared
      < Estimation.StrapdownINS.ESKF.FiniteMagnitudeLimit
    and (innovationGate <= 0.0 or normalizedInnovationSquared
      <= innovationGate * measurementLength);
  rejectionReason := if accepted then
      Estimation.StrapdownINS.CorrectionAccepted
    elseif not inputsUsable then
      Estimation.StrapdownINS.CorrectionRejectedCovarianceUnusable
    elseif not sigmaFactorized or not innovationFactorized then
      Estimation.StrapdownINS.CorrectionRejectedFactorization
    else Estimation.StrapdownINS.CorrectionRejectedGate;

  if accepted then
    gain := transpose(augmentedSolution[:, 1:TangentLength]);
    correction := gain * residual;
    attitudeCorrection := sqrt(correction[7] * correction[7]
      + correction[8] * correction[8] + correction[9] * correction[9]);
    trustScale := if attitudeCorrection
        > Estimation.StrapdownINS.ESKF.MaxAttitudeCorrection_rad then
      Estimation.StrapdownINS.ESKF.MaxAttitudeCorrection_rad
        / attitudeCorrection else 1.0;
    gain := cat(1, gain[1:6, :], trustScale * gain[7:9, :],
      gain[10:TangentLength, :]);
    correction := gain * residual;
    correctedNominal := injectVector(
      nominal, correction);
    // The attitude trust region row-scales the optimal unscented gain. Once
    // modified it is an arbitrary gain, so P-K*S*K' is no longer the valid
    // covariance update. Use the general form driven by the same gain that
    // generated the applied state correction.
    posteriorCovariance := LinearAlgebra.symmetrize(
      predicted.covariance
        - gain * transpose(crossCovariance)
        - crossCovariance * transpose(gain)
        + gain * innovationCovariance * transpose(gain));
    correctedCovariance := LinearAlgebra.symmetrize(
      Estimation.StrapdownINS.ESKF.conjugateReset(
        LieGroups.SE23.Quat.right_jacobian(correction[1:9]),
        posteriorCovariance));
  else
    correctedNominal := nominal;
    correctedCovariance := predicted.covariance;
  end if;
  corrected := Estimation.StrapdownINS.UKF.State(
    positionWorldEnu_m=correctedNominal[1:3],
    velocityWorldEnu_m_s=correctedNominal[4:6],
    quaternionWorldBody=correctedNominal[7:10],
    gyroscopeBiasBodyFlu_rad_s=correctedNominal[11:13],
    accelerometerBiasBodyFlu_m_s2=correctedNominal[14:16],
    covariance=correctedCovariance);
end correctUnscented;
