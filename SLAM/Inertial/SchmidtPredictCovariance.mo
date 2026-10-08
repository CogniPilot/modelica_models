within SLAM.Inertial;
function SchmidtPredictCovariance
  import SLAMCovariancePSDCheck = SLAM.Inertial.SLAMCovariancePSDCheck;
  import SLAMExactRealEqual = SLAM.Inertial.SLAMExactRealEqual;

  input Real covariance[15,15];
  input Real crossCovariance[15,6];
  input Real referenceCovariance[6,6];
  input Real transition[15,15];
  input Real processCovariance[15,15];
  input Real referenceAvailable;
  input Real predictionEnabled;
  output Real accepted;
  output Real nextCovariance[15,15];
  output Real nextCrossCovariance[15,6];
  output Real nextReferenceCovariance[6,6];
protected
  Real raw[15,15]; Real proposed[15,15]; Real proposedCross[15,6];
  Real priorJoint[21,21]; Real proposedJoint[21,21];
  Real currentValid; Real processValid; Real proposedCurrentValid;
  Real priorJointValid; Real proposedJointValid;
  Real finiteTransition[15,15];
algorithm
  raw := transition*covariance*transpose(transition)+processCovariance;
  proposed := 0.5*(raw+transpose(raw));
  proposedCross := transition*crossCovariance;
  priorJoint := cat(1,cat(2,covariance,crossCovariance),
    cat(2,transpose(crossCovariance),referenceCovariance));
  proposedJoint := cat(1,cat(2,proposed,proposedCross),
    cat(2,transpose(proposedCross),referenceCovariance));
  currentValid := SLAMCovariancePSDCheck(covariance,1e-12);
  processValid := SLAMCovariancePSDCheck(processCovariance,1e-12);
  proposedCurrentValid := SLAMCovariancePSDCheck(proposed,1e-12);
  priorJointValid := SLAMCovariancePSDCheck(priorJoint,1e-12);
  proposedJointValid := SLAMCovariancePSDCheck(proposedJoint,1e-12);
  for i in 1:15 loop
    for j in 1:15 loop
      finiteTransition[i,j] := if abs(transition[i,j]) <= 1e6 then 0.0 else 1.0;
    end for;
  end for;
  accepted := if SLAMExactRealEqual(predictionEnabled,1.0)
    and (SLAMExactRealEqual(referenceAvailable,0.0) or SLAMExactRealEqual(referenceAvailable,1.0))
    and currentValid > 0.5 and processValid > 0.5 and proposedCurrentValid > 0.5
    and sum(finiteTransition) < 0.5
    and (SLAMExactRealEqual(referenceAvailable,0.0) or (priorJointValid > 0.5 and proposedJointValid > 0.5))
    then 1.0 else 0.0;
  nextCovariance := if accepted > 0.5 then proposed else covariance;
  nextCrossCovariance := if accepted > 0.5 and SLAMExactRealEqual(referenceAvailable,1.0)
    then proposedCross else crossCovariance;
  nextReferenceCovariance := referenceCovariance;
end SchmidtPredictCovariance;
