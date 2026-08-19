within Estimation.StrapdownINS.UKF;

block Estimator
  "Sampled manifold UKF implementing the common strapdown estimator boundary"
  extends Estimation.StrapdownINS.PartialEstimator;

  parameter Real innovationGate = 6.0
    "NIS gate per measurement degree of freedom; non-positive disables";

protected
  discrete Real statePositionWorldEnu_m[3](each start=0.0, each fixed=true);
  discrete Real stateVelocityWorldEnu_m_s[3](each start=0.0, each fixed=true);
  discrete Real stateQuaternionWorldBody[4](
    start={1.0, 0.0, 0.0, 0.0}, each fixed=true);
  discrete Real stateGyroscopeBiasBodyFlu_rad_s[3](
    each start=0.0, each fixed=true);
  discrete Real stateAccelerometerBiasBodyFlu_m_s2[3](
    each start=0.0, each fixed=true);
  // Literal extent keeps this replaceable block evaluable in compilers that
  // resolve class constants after component array dimensions.
  discrete Real stateCovariance[15, 15](
    each start=0.0, each fixed=true);
  discrete Boolean initialized(start=false, fixed=true);
  discrete Boolean predictionSucceeded(start=false, fixed=true);
  discrete Boolean mocapCorrectionAccepted(start=false, fixed=true);
  discrete Boolean gpsCorrectionAccepted(start=false, fixed=true);
  discrete Boolean opticalFlowCorrectionAccepted(start=false, fixed=true);
  discrete Integer mocapRejections(start=0, fixed=true);
  discrete Integer gpsRejections(start=0, fixed=true);
  discrete Integer opticalFlowRejections(start=0, fixed=true);
  discrete Integer correctionReason(start=0, fixed=true);
  discrete Integer correctionSource(start=0, fixed=true);
  discrete Real correctionNis(start=0.0, fixed=true);

algorithm
  when sample(0.0, samplePeriod) then
    (statePositionWorldEnu_m,
     stateVelocityWorldEnu_m_s,
     stateQuaternionWorldBody,
     stateGyroscopeBiasBodyFlu_rad_s,
     stateAccelerometerBiasBodyFlu_m_s2,
     stateCovariance,
     initialized,
     predictionSucceeded,
     mocapCorrectionAccepted,
     gpsCorrectionAccepted,
     opticalFlowCorrectionAccepted,
     correctionReason,
     correctionSource,
     correctionNis,
     mocapRejections,
     gpsRejections,
     opticalFlowRejections) := Estimation.StrapdownINS.UKF.step(
       pre(initialized),
       Estimation.StrapdownINS.UKF.State(
         positionWorldEnu_m=pre(statePositionWorldEnu_m),
         velocityWorldEnu_m_s=pre(stateVelocityWorldEnu_m_s),
         quaternionWorldBody=pre(stateQuaternionWorldBody),
         gyroscopeBiasBodyFlu_rad_s=pre(stateGyroscopeBiasBodyFlu_rad_s),
         accelerometerBiasBodyFlu_m_s2=
           pre(stateAccelerometerBiasBodyFlu_m_s2),
         covariance=pre(stateCovariance)),
       reset, imu, mocap, gps, opticalFlow,
       gravityWorldEnu_m_s2, samplePeriod,
       initialPositionWorldEnu_m, initialVelocityWorldEnu_m_s,
       initialQuaternionWorldBody, initialGyroscopeBiasBodyFlu_rad_s,
       initialAccelerometerBiasBodyFlu_m_s2, initialVariances,
       processNoise, innovationGate,
       opticalFlowGroundNormalWorldEnu,
       opticalFlowGroundPlaneOffset_m,
       pre(mocapRejections), pre(gpsRejections),
       pre(opticalFlowRejections));

    (estimate.valid,
     estimate.timestamp_s,
     estimate.positionWorldEnu_m,
     estimate.velocityWorldEnu_m_s,
     estimate.accelerationWorldEnu_m_s2,
     estimate.quaternionWorldBody,
     estimate.rotationWorldBody,
     estimate.eulerRpy_rad,
     estimate.angularVelocityBodyFlu_rad_s,
     estimate.angularVelocityWorldEnu_rad_s) :=
      Estimation.StrapdownINS.ESKF.navigationEstimate(
        Estimation.StrapdownINS.ESKF.State(
          positionWorldEnu_m=statePositionWorldEnu_m,
          velocityWorldEnu_m_s=stateVelocityWorldEnu_m_s,
          quaternionWorldBody=stateQuaternionWorldBody,
          gyroscopeBiasBodyFlu_rad_s=stateGyroscopeBiasBodyFlu_rad_s,
          accelerometerBiasBodyFlu_m_s2=
            stateAccelerometerBiasBodyFlu_m_s2,
          covariance=stateCovariance),
        imu, gravityWorldEnu_m_s2, initialized);

    status.initialized := initialized;
    status.predictionAccepted := predictionSucceeded;
    status.mocapCorrectionAccepted := mocapCorrectionAccepted;
    status.gpsPositionCorrectionAccepted := gpsCorrectionAccepted;
    status.gpsVelocityCorrectionAccepted := gpsCorrectionAccepted;
    status.opticalFlowCorrectionAccepted := opticalFlowCorrectionAccepted;
    status.consecutiveRejectedCorrections :=
      if mocapRejections >= gpsRejections
          and mocapRejections >= opticalFlowRejections then
        mocapRejections
      elseif gpsRejections >= opticalFlowRejections then gpsRejections
      else opticalFlowRejections;
    status.rejectionElapsed_s := samplePeriod
      * status.consecutiveRejectedCorrections;
    status.mocapConsecutiveRejections := mocapRejections;
    status.gpsConsecutiveRejections := gpsRejections;
    status.opticalFlowConsecutiveRejections := opticalFlowRejections;
    status.correctionOutcome := correctionReason;
    status.correctionSource := correctionSource;
    status.normalizedInnovationSquared := correctionNis;
    status.recoveryStage := Estimation.StrapdownINS.RecoveryNominal;
    status.anchorSource := correctionSource;
    status.imuPayloadHeld := not (imu.valid and imu.fresh);
  end when;

equation
  navigationCovarianceLocal = stateCovariance[1:6, 1:6];
  gyroscopeBiasBodyFlu_rad_s = stateGyroscopeBiasBodyFlu_rad_s;
  accelerometerBiasBodyFlu_m_s2 = stateAccelerometerBiasBodyFlu_m_s2;
end Estimator;
