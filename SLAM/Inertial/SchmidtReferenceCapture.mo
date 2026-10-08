within SLAM.Inertial;
model SchmidtReferenceCapture
  import RGBDProperRotation = SLAM.Localization.RGBDProperRotation;
  import SLAMCovariancePSDCheck = SLAM.Inertial.SLAMCovariancePSDCheck;
  import SLAMExactRealEqual = SLAM.Inertial.SLAMExactRealEqual;

  constant Integer currentDimension = 15;
  constant Integer referenceDimension = 6;
  input Real position[3] = zeros(3);
  input Real rotation[3,3] = identity(3);
  input Real covariance[currentDimension,currentDimension];
  input Real referencePosition[3] = zeros(3);
  input Real referenceRotation[3,3] = identity(3);
  input Real crossCovariance[currentDimension,referenceDimension];
  input Real referenceCovariance[referenceDimension,referenceDimension];
  input Real referenceAvailable = 0.0;
  input Real captureRequested = 0.0;
  input Real currentValid = 1.0;
  output Real accepted;
  output Real rejected;
  output Real nextReferenceAvailable;
  output Real nextReferencePosition[3];
  output Real nextReferenceRotation[3,3];
  output Real nextCovariance[currentDimension,currentDimension];
  output Real nextCrossCovariance[currentDimension,referenceDimension];
  output Real nextReferenceCovariance[referenceDimension,referenceDimension];
protected
  RGBDProperRotation currentRotationCheck(rotation=rotation);
  RGBDProperRotation referenceRotationCheck(rotation=referenceRotation);
  Real selection[referenceDimension,currentDimension];
  Real proposedCross[currentDimension,referenceDimension];
  Real proposedReference[referenceDimension,referenceDimension];
  Real priorJoint[21,21];
  Real proposedJoint[21,21];
  Real currentCovarianceValid;
  Real priorJointValid;
  Real proposedJointValid;
  Real currentPositionChecks[3];
  Real referencePositionChecks[3];
equation
  // Reference nominal equals current nominal at capture, so both right-local
  // attitude errors share exactly the same body tangent; no guessed heading.
  selection = cat(1,cat(2,identity(3),zeros(3,12)),
    cat(2,zeros(3,6),identity(3),zeros(3,6)));
  proposedCross = covariance*transpose(selection);
  proposedReference = selection*proposedCross;
  priorJoint = cat(1,cat(2,covariance,crossCovariance),
    cat(2,transpose(crossCovariance),referenceCovariance));
  proposedJoint = cat(1,cat(2,covariance,proposedCross),
    cat(2,transpose(proposedCross),proposedReference));
  currentCovarianceValid = SLAMCovariancePSDCheck(covariance,1e-12);
  priorJointValid = SLAMCovariancePSDCheck(priorJoint,1e-12);
  proposedJointValid = SLAMCovariancePSDCheck(proposedJoint,1e-12);
  for i in 1:3 loop
    currentPositionChecks[i] = if noEvent(abs(position[i]) <= 1e6) then 0.0 else 1.0;
    referencePositionChecks[i] = if noEvent(abs(referencePosition[i]) <= 1e6) then 0.0 else 1.0;
  end for;
  accepted = if noEvent(SLAMExactRealEqual(captureRequested,1.0) and SLAMExactRealEqual(currentValid,1.0)
    and (SLAMExactRealEqual(referenceAvailable,0.0) or SLAMExactRealEqual(referenceAvailable,1.0))
    and currentRotationCheck.valid > 0.5 and sum(currentPositionChecks) < 0.5
    and currentCovarianceValid > 0.5 and proposedJointValid > 0.5
    and (SLAMExactRealEqual(referenceAvailable,0.0) or (priorJointValid > 0.5
      and referenceRotationCheck.valid > 0.5 and sum(referencePositionChecks) < 0.5)))
    then 1.0 else 0.0;
  rejected = if noEvent(SLAMExactRealEqual(captureRequested,0.0) or accepted > 0.5) then 0.0 else 1.0;
  nextReferenceAvailable = if noEvent(accepted > 0.5) then 1.0 else referenceAvailable;
  nextReferencePosition = if noEvent(accepted > 0.5) then position else referencePosition;
  nextReferenceRotation = if noEvent(accepted > 0.5) then rotation else referenceRotation;
  nextCovariance = covariance;
  nextCrossCovariance = if noEvent(accepted > 0.5) then proposedCross else crossCovariance;
  nextReferenceCovariance = if noEvent(accepted > 0.5) then proposedReference else referenceCovariance;
end SchmidtReferenceCapture;
