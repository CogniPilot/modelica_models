within SLAM.Localization;
function RGBDRegistrationSandwich
  import RGBDOpticalPointCovariance = SLAM.Localization.RGBDOpticalPointCovariance;
  import RGBDUncertaintyInverse6 = SLAM.Localization.RGBDUncertaintyInverse6;
  import RGBDUncertaintyProper = SLAM.Localization.RGBDUncertaintyProper;
  import RGBDUncertaintySkew = SLAM.Localization.RGBDUncertaintySkew;

  input Real referencePoint[:,:]; input Real currentPoint[:,:];
  input Real pairEnabled[:]; input Real activeCount; input Real registrationAccepted;
  input Real currentFromReference[3,3]; input Real translation[3];
  input Real referenceBodyRotation[3,3]; input Real opticalToBody[3,3];
  input Real cameraOriginBody[3];
  input Real referenceRgbFocal[2]; input Real currentRgbFocal[2];
  input Real referenceNoiseFx; input Real currentNoiseFx; input Real baseline;
  input Real localizationSigma; input Real disparitySigma; input Real depthInflation;
  input Real coordinateLimit; input Real minimumPivot;
  output Real valid; output Real rejectionReason;
  output Real validCount; output Real invalidCount;
  output Real relativeCovariance[6,6]; output Real observationCovariance[6,6];
  output Real normalMatrix[6,6]; output Real noiseMatrix[6,6];
  output Real observationJacobian[6,6]; output Real minimumScaledPivot;
protected
  Boolean configuration; Boolean pose; Boolean pointValid; Boolean hValid; Boolean cValid;
  Real q[3]; Real J[3,6]; Real Sigma[3,3]; Real Cref[3,3]; Real Ccur[3,3];
  Real Hinv[6,6]; Real unusedInverse[6,6]; Real unusedPivot;
  Real cameraRotation[3,3]; Real arm[3]; Real skewArm[3,3]; Real rotationBlock[3,3];
  Real candidateRelative[6,6]; Real candidateObservation[6,6];
algorithm
  valid := 0.0; validCount := 0.0; invalidCount := 0.0;
  normalMatrix := zeros(6,6); noiseMatrix := zeros(6,6);
  relativeCovariance := zeros(6,6); observationCovariance := zeros(6,6);
  observationJacobian := zeros(6,6); minimumScaledPivot := 0.0;
  configuration := activeCount >= 0.0 and activeCount <= size(pairEnabled,1)
    and floor(activeCount) == activeCount and size(referencePoint,1) == size(pairEnabled,1)
    and size(currentPoint,1) == size(pairEnabled,1) and size(referencePoint,2) == 3 and size(currentPoint,2) == 3
    and baseline > 0.0 and baseline <= 1.0 and localizationSigma > 0.0 and localizationSigma <= 10.0
    and disparitySigma > 0.0 and disparitySigma <= 10.0 and depthInflation >= 1.0 and depthInflation <= 100.0
    and referenceNoiseFx > 0.0 and referenceNoiseFx <= 1e6 and currentNoiseFx > 0.0 and currentNoiseFx <= 1e6
    and coordinateLimit > 0.0 and coordinateLimit <= 1e4 and minimumPivot > 0.0 and minimumPivot < 1.0;
  for k in 1:2 loop
    configuration := configuration and referenceRgbFocal[k] > 0.0 and referenceRgbFocal[k] <= 1e6
      and currentRgbFocal[k] > 0.0 and currentRgbFocal[k] <= 1e6;
  end for;
  pose := registrationAccepted == 1.0 and RGBDUncertaintyProper(currentFromReference)
    and RGBDUncertaintyProper(referenceBodyRotation) and RGBDUncertaintyProper(opticalToBody);
  for k in 1:3 loop
    pose := pose and abs(translation[k]) <= coordinateLimit and abs(cameraOriginBody[k]) <= coordinateLimit;
  end for;
  for i in 1:size(pairEnabled,1) loop
    pointValid := configuration and i <= activeCount and pairEnabled[i] == 1.0;
    for k in 1:3 loop
      pointValid := pointValid and abs(referencePoint[i,k]) <= coordinateLimit and abs(currentPoint[i,k]) <= coordinateLimit;
    end for;
    pointValid := pointValid and referencePoint[i,3] > 0.0 and currentPoint[i,3] > 0.0;
    invalidCount := invalidCount+(if i <= activeCount and pairEnabled[i] <> 0.0 and not pointValid then 1.0 else 0.0);
    validCount := validCount+(if pointValid then 1.0 else 0.0);
    if pointValid and pose then
      q := currentFromReference*referencePoint[i,:];
      J := zeros(3,6);
      J[:,1:3] := identity(3); J[:,4:6] := -RGBDUncertaintySkew(q);
      Cref := RGBDOpticalPointCovariance(referencePoint[i,:],referenceRgbFocal[1],referenceRgbFocal[2],
        localizationSigma,disparitySigma,referenceNoiseFx,baseline,depthInflation);
      Ccur := RGBDOpticalPointCovariance(currentPoint[i,:],currentRgbFocal[1],currentRgbFocal[2],
        localizationSigma,disparitySigma,currentNoiseFx,baseline,depthInflation);
      Sigma := Ccur+currentFromReference*Cref*transpose(currentFromReference);
      normalMatrix := normalMatrix+transpose(J)*J;
      noiseMatrix := noiseMatrix+transpose(J)*Sigma*J;
    end if;
  end for;
  (Hinv,hValid,minimumScaledPivot) := RGBDUncertaintyInverse6(normalMatrix,minimumPivot);
  candidateRelative := Hinv*noiseMatrix*transpose(Hinv);
  cameraRotation := if pose then referenceBodyRotation*opticalToBody*transpose(currentFromReference) else identity(3);
  arm := if pose then translation+transpose(opticalToBody)*cameraOriginBody else zeros(3);
  skewArm := RGBDUncertaintySkew(arm); rotationBlock := -cameraRotation*skewArm;
  for a in 1:3 loop
    for b in 1:3 loop
      observationJacobian[a,b] := -cameraRotation[a,b];
      observationJacobian[a,b+3] := rotationBlock[a,b];
      observationJacobian[a+3,b+3] := if pose then -opticalToBody[a,b] else 0.0;
    end for;
  end for;
  candidateObservation := observationJacobian*candidateRelative*transpose(observationJacobian);
  (unusedInverse,cValid,unusedPivot) := RGBDUncertaintyInverse6(candidateObservation,minimumPivot);
  rejectionReason := if not configuration then 1.0 else if not pose then 2.0 else if invalidCount > 0.0 then 3.0
    else if validCount < 3.0 then 4.0 else if not hValid then 5.0 else if not cValid then 6.0 else 0.0;
  valid := if rejectionReason == 0.0 then 1.0 else 0.0;
  relativeCovariance := if valid > 0.5 then candidateRelative else zeros(6,6);
  observationCovariance := if valid > 0.5 then candidateObservation else zeros(6,6);
end RGBDRegistrationSandwich;
