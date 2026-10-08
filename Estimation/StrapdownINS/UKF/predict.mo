within Estimation.StrapdownINS.UKF;

function predict
  "Unscented propagation through the strapdown nominal mechanization"
  input State previous;
  input Real angularVelocityMeasuredBodyFlu_rad_s[3];
  input Real specificForceMeasuredBodyFlu_m_s2[3];
  input Real gravityWorldEnu_m_s2[3];
  input Real dt(unit = "s");
  input Estimation.StrapdownINS.ProcessNoise processNoise;
  output State predicted;
  output Boolean success;
protected
  Real previousNominal[16];
  Real sigmaState[16, SigmaCount];
  Real propagated[16, SigmaCount];
  Real predictedMean[16];
  Real sigma[TangentLength, SigmaCount];
  Real meanCorrection[TangentLength];
  Real deviation[TangentLength];
  Real covariance[TangentLength, TangentLength];
  Real correctedAngularVelocity[3];
  Real correctedSpecificForce[3];
  Real A[TangentLength, TangentLength];
  Real G[TangentLength, 12];
  Real continuousNoise[12, 12];
  Real discreteNoise[TangentLength, TangentLength];
algorithm
  previousNominal := stateVector(previous);
  (sigma, success) := sigmaTangents(previous.covariance);
  for sigmaIndex in 1:SigmaCount loop
    sigmaState[:, sigmaIndex] := injectVector(
      previousNominal, sigma[:, sigmaIndex]);
    propagated[:, sigmaIndex] := predictNominalVector(
      sigmaState[:, sigmaIndex], angularVelocityMeasuredBodyFlu_rad_s,
      specificForceMeasuredBodyFlu_m_s2, gravityWorldEnu_m_s2, dt);
  end for;

  predictedMean := propagated[:, 1];
  for iteration in 1:4 loop
    meanCorrection := zeros(TangentLength);
    for sigmaIndex in 2:SigmaCount loop
      // Preserve the per-sigma call in Rumoca 0.10.2 GALEC reductions.
      deviation := localErrorVector(predictedMean, propagated[:, sigmaIndex]);
      meanCorrection := meanCorrection + SigmaWeight * deviation;
    end for;
    predictedMean := injectVector(predictedMean, meanCorrection);
  end for;

  deviation := localErrorVector(predictedMean, propagated[:, 1]);
  covariance := transpose({CentralCovarianceWeight * deviation})
    * {deviation};
  for sigmaIndex in 2:SigmaCount loop
    deviation := localErrorVector(predictedMean, propagated[:, sigmaIndex]);
    covariance := covariance
      + transpose({SigmaWeight * deviation}) * {deviation};
  end for;

  correctedAngularVelocity := angularVelocityMeasuredBodyFlu_rad_s
    - predictedMean[11:13];
  correctedSpecificForce := specificForceMeasuredBodyFlu_m_s2
    - predictedMean[14:16];
  A := Estimation.StrapdownINS.ESKF.continuousTransition(
    correctedAngularVelocity, correctedSpecificForce);
  G := Estimation.StrapdownINS.ESKF.noiseInputMatrix();
  continuousNoise :=
    Estimation.StrapdownINS.ESKF.processNoiseMatrix(processNoise);
  discreteNoise := Estimation.StrapdownINS.ESKF.discreteProcessCovariance(
    A, G, continuousNoise, dt);
  predicted := State(
    positionWorldEnu_m=predictedMean[1:3],
    velocityWorldEnu_m_s=predictedMean[4:6],
    quaternionWorldBody=predictedMean[7:10],
    gyroscopeBiasBodyFlu_rad_s=predictedMean[11:13],
    accelerometerBiasBodyFlu_m_s2=predictedMean[14:16],
    covariance=LinearAlgebra.symmetrize(covariance + discreteNoise));
end predict;
