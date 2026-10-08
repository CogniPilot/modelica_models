within SLAM.Localization;
// Ordered full raw-camera composition. The equation models above retain the
// unweighted baseline; robustRegistration=false selects that same fit here.
// The bounded consensus fallback uses calibrated per-pair residuals by default.
// calibratedRegistration=false retains the fixed metric gate for comparisons.
// imageEnabled retains its existing meaning
// (only description is bypassed, matching/registration still report refusals).
function ObserveRGBDRelativeFrame
  import DescribeRGBDFrame = Vision.Matching.DescribeRGBDFrame;
  import FitRigidPointPairs = Vision.Registration.FitRigidPointPairs;
  import FitRigidPointPairsRobust = Vision.Registration.FitRigidPointPairsRobust;
  import MatchRGBDDescriptors = Vision.Matching.MatchRGBDDescriptors;
  import RGBDOpticalPointCovariance = SLAM.Localization.RGBDOpticalPointCovariance;
  import RGBDRegistrationSandwich = SLAM.Localization.RGBDRegistrationSandwich;
  import RGBDRelativeBodyPose = SLAM.Localization.RGBDRelativeBodyPose;

  input Real rgb[:,:,:]; input Real depth[size(rgb,1),size(rgb,2)];
  input Real pixels[featureCapacity,2]; input Real activeCount;
  input Real rgbCalibration[4]; input Real depthCalibration[4]; input Real noiseReferenceFx;
  input Real referenceDescriptor[featureCapacity,descriptorSize]; input Real referencePoint[featureCapacity,3];
  input Real referenceEnabled[featureCapacity]; input Real referenceCount;
  input Real referenceRgbFocal[2]; input Real referenceNoiseReferenceFx;
  input Boolean imageEnabled = true;
  input Real disparityNoise = 0.08; input Real baseline = 0.05;
  input Real referenceBodyRotation[3,3] = identity(3);
  input Real referenceBodyPosition[3] = zeros(3);
  input Real opticalToBody[3,3] = [0.0,0.0,1.0;-1.0,0.0,0.0;0.0,-1.0,0.0];
  input Real cameraOriginBody[3] = {0.18,0.0,-0.04};
  input Real usePrediction = 0.0;
  input Real predictedRotation[3,3] = identity(3); input Real predictedTranslation[3] = zeros(3);
  input Real nearDepth = 0.28; input Real farDepth = 10.0; input Real minimumContrast = 1e-6;
  input Real ratio = 0.8; input Real maximumDescriptorDistance = 0.8; input Real maximumGeometricDistance = 0.5;
  input Real registrationCoordinateLimit = 1e6; input Real rankTolerance = 1e-8; input Real maximumRms = 0.02;
  input Real localizationSigma = 0.5; input Real depthInflation = 1.0;
  input Real uncertaintyCoordinateLimit = 100.0; input Real uncertaintyMinimumPivot = 1e-10;
  input Boolean robustRegistration = true;
  input Integer maximumRegistrationHypotheses = 64;
  input Real minimumRegistrationConsensusFraction = 0.5;
  output Real observedBodyRotation[3,3]; output Real observedBodyPosition[3];
  output Real valid; output Real matchCount; output Real registrationRms; output Real registrationRejectionReason;
  output Real currentIndex[featureCapacity]; output Real currentDescriptor[featureCapacity,descriptorSize];
  output Real currentPoint[featureCapacity,3]; output Real currentEnabled[featureCapacity];
  output Real currentFromReference[3,3]; output Real currentFromReferenceTranslation[3];
  output Real relativeCovariance[6,6] "Optical translation/left-angle runtime relative noise";
  output Real conditionalObservationCovariance[6,6] "Conditional on retained reference pose; not independent absolute noise";
  output Real relativeValid; output Real uncertaintyRejectionReason;
  output Real uncertaintyValidCount; output Real uncertaintyInvalidCount;
  output Real descriptionInvalidCount; output Real matchingConfigurationValid;
  output Real invalidReference; output Real invalidCurrent; output Real pairEnabled[featureCapacity];
  output Real registrationAccepted; output Real registrationValidCount; output Real registrationInvalidCount;
  output Real registrationRank; output Real uncertaintyValid;
  input Real depthUnits = 1.0 "Meters per depth sample; 1 for metric depth, SDK scale for Z16";
  input Boolean calibratedRegistration = true;
  input Real maximumNormalizedSquared = 9.0 "Squared whitened residual radius, not a metric RMS";
protected
  constant Integer featureCapacity = 350;
  constant Integer descriptorWidth = 7; constant Integer descriptorSize = descriptorWidth*descriptorWidth;
  Real sourcePoint[featureCapacity,3]; Real targetPoint[featureCapacity,3];
  Real nearestDistance[featureCapacity]; Real secondDistance[featureCapacity];
  Real candidatePairEnabled[featureCapacity]; Real registrationInlierMask[featureCapacity];
  Real registrationRejectedCount;
  Real registrationCost; Real eigenGap; Real sourceCentroid[3]; Real targetCentroid[3];
  Real sourceCovariance[featureCapacity,3,3]; Real targetCovariance[featureCapacity,3,3];
  Boolean noiseConfiguration;
  Real poseValid; Real normalMatrix[6,6]; Real noiseMatrix[6,6]; Real observationJacobian[6,6]; Real minimumScaledPivot;
algorithm
  (currentDescriptor,currentPoint,currentEnabled,descriptionInvalidCount) :=
    DescribeRGBDFrame(rgb,depth,pixels,activeCount,rgbCalibration,depthCalibration,
      disparityNoise,noiseReferenceFx,baseline,nearDepth,farDepth,minimumContrast,imageEnabled,depthUnits=depthUnits);
  (currentIndex,candidatePairEnabled,sourcePoint,targetPoint,matchCount,matchingConfigurationValid,
    invalidReference,invalidCurrent,nearestDistance,secondDistance) :=
    MatchRGBDDescriptors(referenceDescriptor,currentDescriptor,referencePoint,currentPoint,
      referenceEnabled,currentEnabled,referenceCount,activeCount,ratio,maximumDescriptorDistance,
      usePrediction,predictedRotation,predictedTranslation,maximumGeometricDistance);
  // Sparse masks span the complete reference domain, never the match prefix.
  if robustRegistration then
    sourceCovariance := zeros(featureCapacity,3,3); targetCovariance := zeros(featureCapacity,3,3);
    noiseConfiguration := referenceRgbFocal[1] > 0 and referenceRgbFocal[2] > 0
      and rgbCalibration[1] > 0 and rgbCalibration[2] > 0
      and referenceNoiseReferenceFx > 0 and noiseReferenceFx > 0 and baseline > 0
      and localizationSigma > 0 and disparityNoise > 0 and depthInflation >= 1;
    if calibratedRegistration and noiseConfiguration then
      for pair in 1:featureCapacity loop
        if candidatePairEnabled[pair] == 1 and sourcePoint[pair,3] > 0 and targetPoint[pair,3] > 0 then
          sourceCovariance[pair,:,:] := RGBDOpticalPointCovariance(sourcePoint[pair,:],
            referenceRgbFocal[1],referenceRgbFocal[2],localizationSigma,disparityNoise,
            referenceNoiseReferenceFx,baseline,depthInflation);
          targetCovariance[pair,:,:] := RGBDOpticalPointCovariance(targetPoint[pair,:],
            rgbCalibration[1],rgbCalibration[2],localizationSigma,disparityNoise,
            noiseReferenceFx,baseline,depthInflation);
        end if;
      end for;
    end if;
    (registrationAccepted,registrationRejectionReason,currentFromReference,currentFromReferenceTranslation,
      registrationValidCount,registrationInvalidCount,registrationRank,registrationCost,registrationRms,
      eigenGap,sourceCentroid,targetCentroid,registrationInlierMask,registrationRejectedCount) :=
      FitRigidPointPairsRobust(sourcePoint,targetPoint,candidatePairEnabled,
        featureCapacity,registrationCoordinateLimit,rankTolerance,maximumRms,
        maximumRegistrationHypotheses,minimumRegistrationConsensusFraction,
        useCovariance=calibratedRegistration,sourceCovariance=sourceCovariance,targetCovariance=targetCovariance,
        maximumNormalizedSquared=maximumNormalizedSquared,covarianceMinimumPivot=uncertaintyMinimumPivot);
  else
    (registrationAccepted,registrationRejectionReason,currentFromReference,currentFromReferenceTranslation,
      registrationValidCount,registrationInvalidCount,registrationRank,registrationCost,registrationRms,
      eigenGap,sourceCentroid,targetCentroid) := FitRigidPointPairs(sourcePoint,targetPoint,candidatePairEnabled,
        featureCapacity,registrationCoordinateLimit,rankTolerance,maximumRms);
    registrationInlierMask := candidatePairEnabled; registrationRejectedCount := 0.0;
  end if;
  // Descriptor candidates remain observable in matchCount. Accepted registration,
  // uncertainty and tracking consume the same certified sparse inlier inventory.
  // On refusal retain candidate diagnostics; valid=0 prevents a filter update.
  pairEnabled := if registrationAccepted > 0.5 then registrationInlierMask else candidatePairEnabled;
  if registrationAccepted > 0.5 then
    for pair in 1:featureCapacity loop
      if registrationInlierMask[pair] < 0.5 then currentIndex[pair] := 0.0; end if;
    end for;
  end if;
  (poseValid,observedBodyRotation,observedBodyPosition) := RGBDRelativeBodyPose(referenceBodyRotation,
    referenceBodyPosition,currentFromReference,currentFromReferenceTranslation,opticalToBody,
    cameraOriginBody,registrationAccepted);
  valid := if matchingConfigurationValid > 0.5 then poseValid else 0.0;
  (uncertaintyValid,uncertaintyRejectionReason,uncertaintyValidCount,uncertaintyInvalidCount,
    relativeCovariance,conditionalObservationCovariance,normalMatrix,noiseMatrix,observationJacobian,minimumScaledPivot) :=
    RGBDRegistrationSandwich(sourcePoint,targetPoint,pairEnabled,featureCapacity,valid,currentFromReference,
      currentFromReferenceTranslation,referenceBodyRotation,opticalToBody,cameraOriginBody,
      referenceRgbFocal,{rgbCalibration[1],rgbCalibration[2]},referenceNoiseReferenceFx,noiseReferenceFx,
      baseline,localizationSigma,disparityNoise,depthInflation,uncertaintyCoordinateLimit,uncertaintyMinimumPivot);
  relativeValid := if valid > 0.5 and uncertaintyValid > 0.5 then 1.0 else 0.0;
end ObserveRGBDRelativeFrame;
