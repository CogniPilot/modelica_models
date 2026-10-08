within Tests;

block SquareRootCorrectionReplay
  "Deployment probe for correlated square-root corrections and constrained gains"
  input Real priorState[16];
  input Real priorCovariance[15, 15];
  input Real priorRoot[15, 15];
  input Real measurementResidual[3];
  input Real observationMatrix[3, 15];
  input Real measurementCovariance[3, 3];
  input Real measurementStateCrossCovariance[15, 3];
  input Real attitudeAxis[3];
  input Real innovationGate;
  input Boolean headingOnly;
  input Boolean useSemiDirectBias;
  output Real correctedState[16];
  output Real posteriorCovariance[15, 15];
  output Real posteriorRoot[15, 15];
  output Boolean accepted;
  output Integer reason;
  output Real nis;
protected
  Estimation.StrapdownINS.ESKF.State posterior;
algorithm
  when sample(0.0, 0.01) then
    (posterior, accepted, reason, nis) := Estimation.StrapdownINS.ESKF.correctLinear(
      Estimation.StrapdownINS.ESKF.State(
        positionWorldEnu_m=priorState[1:3], velocityWorldEnu_m_s=priorState[4:6],
        quaternionWorldBody=priorState[7:10], gyroscopeBiasBodyFlu_rad_s=priorState[11:13],
        accelerometerBiasBodyFlu_m_s2=priorState[14:16], covariance=priorCovariance,
        covarianceRoot=priorRoot, useSquareRootCovariance=true),
      measurementResidual, observationMatrix, measurementCovariance,
      innovationGate, attitudeAxis, measurementStateCrossCovariance,
      headingOnly, useSemiDirectBias);
    correctedState := cat(1, posterior.positionWorldEnu_m,
      posterior.velocityWorldEnu_m_s, posterior.quaternionWorldBody,
      posterior.gyroscopeBiasBodyFlu_rad_s, posterior.accelerometerBiasBodyFlu_m_s2);
    posteriorCovariance := posterior.covariance;
    posteriorRoot := posterior.covarianceRoot;
  end when;
end SquareRootCorrectionReplay;
