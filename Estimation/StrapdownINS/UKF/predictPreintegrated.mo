within Estimation.StrapdownINS.UKF;

function predictPreintegrated
  "Unscented propagation through one closed-form IMU preintegration packet"
  input Estimation.StrapdownINS.UKF.State previous;
  input Avionics.ImuSample imu;
  input Real gravityWorldEnu_m_s2[3];
  input Estimation.StrapdownINS.ProcessNoise processNoise;
  output Estimation.StrapdownINS.UKF.State predicted;
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
  Real deltaPositionBodyFlu_m[3];
  Real deltaVelocityBodyFlu_m_s[3];
  Real deltaQuaternionBodyFlu[4];
  Real rotationIncrement_rad[3];
  Real equivalentAngularVelocity_rad_s[3];
  Real equivalentSpecificForce_m_s2[3];
  Real A[TangentLength, TangentLength];
  Real G[TangentLength, 12];
  Real continuousNoise[12, 12];
  Real discreteNoise[TangentLength, TangentLength];
algorithm
  previousNominal := stateVector(previous);
  (sigma, success) := sigmaTangents(previous.covariance);
  for index in 1:SigmaCount loop
    sigmaState[:, index] := injectVector(
      previousNominal, sigma[:, index]);
    propagated[:, index] := predictPreintegratedNominalVector(
      sigmaState[:, index], imu, gravityWorldEnu_m_s2);
  end for;

  predictedMean := propagated[:, 1];
  for iteration in 1:4 loop
    meanCorrection := zeros(TangentLength);
    for index in 2:SigmaCount loop
      meanCorrection := meanCorrection + SigmaWeight
        * localErrorVector(predictedMean, propagated[:, index]);
    end for;
    predictedMean := injectVector(predictedMean, meanCorrection);
  end for;

  deviation := localErrorVector(predictedMean, propagated[:, 1]);
  covariance := zeros(TangentLength, TangentLength);
  for row in 1:TangentLength loop
    for column in 1:TangentLength loop
      covariance[row, column] := CentralCovarianceWeight
        * deviation[row] * deviation[column];
    end for;
  end for;
  for index in 2:SigmaCount loop
    deviation := localErrorVector(predictedMean, propagated[:, index]);
    for row in 1:TangentLength loop
      for column in 1:TangentLength loop
        covariance[row, column] := covariance[row, column]
          + SigmaWeight * deviation[row] * deviation[column];
      end for;
    end for;
  end for;

  (deltaPositionBodyFlu_m,
   deltaVelocityBodyFlu_m_s,
   deltaQuaternionBodyFlu) :=
    Estimation.StrapdownINS.correctPreintegratedImu(
      imu, predictedMean[11:13], predictedMean[14:16]);
  rotationIncrement_rad :=
    LieGroups.SO3.Quat.log_map(deltaQuaternionBodyFlu);
  equivalentAngularVelocity_rad_s :=
    rotationIncrement_rad / imu.integrationTime_s;
  equivalentSpecificForce_m_s2 :=
    LieGroups.SO3.Quat.left_jacobian_inv(rotationIncrement_rad)
      * deltaVelocityBodyFlu_m_s / imu.integrationTime_s;
  A := Estimation.StrapdownINS.ESKF.continuousTransition(
    equivalentAngularVelocity_rad_s, equivalentSpecificForce_m_s2);
  G := Estimation.StrapdownINS.ESKF.noiseInputMatrix();
  continuousNoise :=
    Estimation.StrapdownINS.ESKF.processNoiseMatrix(processNoise);
  discreteNoise := Estimation.StrapdownINS.ESKF.discreteProcessCovariance(
    A, G, continuousNoise, imu.integrationTime_s);
  predicted := Estimation.StrapdownINS.UKF.State(
    positionWorldEnu_m=predictedMean[1:3],
    velocityWorldEnu_m_s=predictedMean[4:6],
    quaternionWorldBody=predictedMean[7:10],
    gyroscopeBiasBodyFlu_rad_s=predictedMean[11:13],
    accelerometerBiasBodyFlu_m_s2=predictedMean[14:16],
    covariance=LinearAlgebra.symmetrize(covariance + discreteNoise));
end predictPreintegrated;
