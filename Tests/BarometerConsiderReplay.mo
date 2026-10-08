within Tests;

block BarometerConsiderReplay
  "Deployment probe for a pressure datum retained as a consider state"
  input Real priorState[16];
  input Real priorCovariance[15, 15];
  input Real priorRoot[15, 15];
  input Real priorCrossCovariance[15];
  input Real measurementResidual[3];
  input Real observationMatrix[3, 15];
  input Real measurementCovariance[3, 3];
  input Real measurementStateCrossCovariance[15, 3];
  input Real measurementBarometerCrossCovariance[3];
  input Real attitudeAxis[3];
  input Real innovationGate;
  input Real biasVariance;
  input Real biasMean;
  input Real biasProcessNoise;
  input Real pressureAltitude;
  input Real pressureVariance;
  input Real measurementAge;
  input Real transition[15, 15];
  input Real varianceBounds[15];
  input Integer operation;
  input Boolean useSquareRoot;
  input Boolean useSemiDirectBias;
  input Boolean headingOnly;
  input Boolean useJointBarometerBias;
  output Real correctedState[16];
  output Real posteriorCovariance[15, 15];
  output Real posteriorCrossCovariance[15];
  output Real posteriorBiasMean;
  output Real posteriorBiasVariance;
  output Boolean accepted;
  output Integer reason;
  output Real nis;
protected
  Estimation.StrapdownINS.ESKF.State prior(
    positionWorldEnu_m=priorState[1:3], velocityWorldEnu_m_s=priorState[4:6],
    quaternionWorldBody=priorState[7:10], gyroscopeBiasBodyFlu_rad_s=priorState[11:13],
    accelerometerBiasBodyFlu_m_s2=priorState[14:16], covariance=priorCovariance,
    covarianceRoot=priorRoot, useSquareRootCovariance=useSquareRoot,
    barometerBiasCrossCovariance=priorCrossCovariance,
    barometerBias_m=biasMean, barometerBiasVariance_m2=biasVariance,
    useJointBarometerBias=useJointBarometerBias);
  Estimation.StrapdownINS.ESKF.State posterior;
  Estimation.StrapdownINS.ProcessNoise processNoise(
    gyroscope_rad2_s=0.01 * identity(3),
    accelerometer_m2_s3=0.02 * identity(3),
    gyroscopeBias_rad2_s3=0.03 * identity(3),
    accelerometerBias_m2_s5=0.04 * identity(3));
algorithm
  when sample(0.0, 0.01) then
    accepted := false;
    reason := 0;
    nis := 0.0;
    if operation == 0 then
      (posterior, accepted, reason, nis) := Estimation.StrapdownINS.ESKF.correctLinear(
        prior, measurementResidual, observationMatrix, measurementCovariance,
        innovationGate, attitudeAxis, measurementStateCrossCovariance,
        headingOnly, useSemiDirectBias, measurementBarometerCrossCovariance);
    elseif operation == 1 then
      (posterior, accepted, reason, nis) := Estimation.StrapdownINS.ESKF.correctBarometer(
        prior, Avionics.BarometerSample(valid=true, fresh=true, timestamp_s=0.0,
          altitudeWorldEnu_m=pressureAltitude, variance_m2=pressureVariance),
        0.0, biasVariance, innovationGate, measurementAge, zeros(3), zeros(3),
        zeros(3), 0.25, useSemiDirectBias, true, biasProcessNoise);
    elseif operation == 2 then
      posterior := Estimation.StrapdownINS.ESKF.predictCovariance(
        prior, transition, zeros(15, 15), processNoise, 0.02);
    elseif operation == 3 then
      posterior := Estimation.StrapdownINS.ESKF.predictStationary(prior, 0.02, processNoise);
    elseif operation == 4 then
      posterior := Estimation.StrapdownINS.ESKF.limitStateCovariance(prior,
        Estimation.StrapdownINS.ESKF.VarianceLimits(
          position_m2=varianceBounds[1:3], velocity_m2_s2=varianceBounds[4:6],
          attitude_rad2=varianceBounds[7:9], gyroscopeBias_rad2_s2=varianceBounds[10:12],
          accelerometerBias_m2_s4=varianceBounds[13:15]));
    else
      posterior := Estimation.StrapdownINS.ESKF.reseed(prior, zeros(3), zeros(3),
        Estimation.StrapdownINS.InitialVariances(
          position_m2=varianceBounds[1:3], velocity_m2_s2=varianceBounds[4:6],
          attitude_rad2=varianceBounds[7:9], gyroscopeBias_rad2_s2=varianceBounds[10:12],
          accelerometerBias_m2_s4=varianceBounds[13:15]));
    end if;
    correctedState := cat(1, posterior.positionWorldEnu_m,
      posterior.velocityWorldEnu_m_s, posterior.quaternionWorldBody,
      posterior.gyroscopeBiasBodyFlu_rad_s, posterior.accelerometerBiasBodyFlu_m_s2);
    posteriorCovariance := posterior.covariance;
    posteriorCrossCovariance := posterior.barometerBiasCrossCovariance;
    posteriorBiasMean := posterior.barometerBias_m;
    posteriorBiasVariance := posterior.barometerBiasVariance_m2;
  end when;
end BarometerConsiderReplay;
