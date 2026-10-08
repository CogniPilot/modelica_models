within SLAM.Localization;
// Ordered reusable full-domain step. Both original equation models above/in
// SchmidtReferenceState remain unchanged reference owners. Explicit covariance
// inputs carry tuned priors; these defaults match the existing public interface.
pure function AdvanceRGBDLocalization
  import ES15PredictHeldInterval = SLAM.Inertial.ES15PredictHeldInterval;
  import ObserveRGBDRelativeFrame = SLAM.Localization.ObserveRGBDRelativeFrame;
  import RGBDLocalizationTracking = SLAM.Localization.RGBDLocalizationTracking;
  import RGBDNominalCalibration = Vision.Matching.RGBDNominalCalibration;
  import RGBDProjectLandmarks = SLAM.Mapping.RGBDProjectLandmarks;
  import RGBDProperRotationValue = SLAM.Localization.RGBDProperRotationValue;
  import SLAMExactRealEqual = SLAM.Inertial.SLAMExactRealEqual;
  import SLAMRotationCoordinates = SLAM.Inertial.SLAMRotationCoordinates;
  import SchmidtCaptureReference = SLAM.Inertial.SchmidtCaptureReference;
  import SchmidtCorrectRelativePose = SLAM.Inertial.SchmidtCorrectRelativePose;
  import SchmidtImagePairEligibility = SLAM.Inertial.SchmidtImagePairEligibility;

  input Real rgb[:,:,:];
  input Real depth[size(rgb,1),size(rgb,2)];
  input Real rgbCalibration[4] = RGBDNominalCalibration({size(rgb,1),size(rgb,2)},{69.0,42.0});
  input Real depthCalibration[4] = RGBDNominalCalibration({size(rgb,1),size(rgb,2)},{87.0,58.0});
  input Real disparityNoise = 0.08;
  input Real noiseReferenceFx = defaultNoiseReferenceFx;
  input Real baseline = 0.05;
  input Real opticalToBody[3,3] = [0.0,0.0,1.0;-1.0,0.0,0.0;0.0,-1.0,0.0];
  input Real cameraOriginBody[3] = {0.18,0.0,-0.04};
  input Real frameEnabled = 0.0;
  input Real imageCaptureRequested = 0.0;
  input Real position[3] = zeros(3);
  input Real velocity[3] = zeros(3);
  input Real rotation[3,3] = identity(3);
  input Real accelBias[3] = zeros(3);
  input Real gyroBias[3] = zeros(3);
  input Real covariance[15,15] = diagonal({initialPositionVariance,initialPositionVariance,initialPositionVariance,
    initialVelocityVariance,initialVelocityVariance,initialVelocityVariance,
    initialAttitudeVariance,initialAttitudeVariance,initialAttitudeVariance,
    initialAccelBiasVariance,initialAccelBiasVariance,initialAccelBiasVariance,
    initialGyroBiasVariance,initialGyroBiasVariance,initialGyroBiasVariance});
  input Real crossCovariance[15,6] = zeros(15,6);
  input Real referenceCovariance[6,6] = zeros(6,6);
  input Real referencePosition[3] = zeros(3);
  input Real referenceRotation[3,3] = identity(3);
  input Real referenceAvailable = 0.0;
  input Real referenceEpoch = 0.0;
  input Real currentEpoch = 0.0;
  input Real referenceUsed = 0.0;
  input Real lastUsedEpoch = -1.0;
  input Real referenceDescriptor[featureCapacity,descriptorSize] = zeros(featureCapacity,descriptorSize);
  input Real referencePoint[featureCapacity,3] = zeros(featureCapacity,3);
  input Real referenceEnabled[featureCapacity] = zeros(featureCapacity);
  input Real referencePixels[featureCapacity,2] = zeros(featureCapacity,2);
  input Real referenceCount = 0.0;
  input Real referenceRgbCalibration[4] = RGBDNominalCalibration({size(rgb,1),size(rgb,2)},{69.0,42.0});
  input Real referenceDepthCalibration[4] = RGBDNominalCalibration({size(rgb,1),size(rgb,2)},{87.0,58.0});
  input Real referenceNoiseReferenceFx = defaultNoiseReferenceFx;
  input Real referenceDisparityNoise = 0.08;
  input Real referenceBaseline = 0.05;
  input Real referenceOpticalToBody[3,3] = [0.0,0.0,1.0;-1.0,0.0,0.0;0.0,-1.0,0.0];
  input Real referenceCameraOriginBody[3] = {0.18,0.0,-0.04};
  input Real accel[3] = {0.0,0.0,9.81};
  input Real gyro[3] = zeros(3);
  input Real gravity[3] = {0.0,0.0,-9.81};
  input Real h = 1.0/90.0;
  input Real density[12] = {0.06,0.06,0.06,0.006,0.006,0.006,0.002,0.002,0.002,0.0002,0.0002,0.0002};
  input Real pixels[featureCapacity,2];
  input Real activeCount;
  input Real featureScore[featureCapacity] = zeros(featureCapacity);
  output Real nextPosition[3];
  output Real nextVelocity[3];
  output Real nextRotation[3,3];
  output Real nextAccelBias[3];
  output Real nextGyroBias[3];
  output Real nextCovariance[15,15];
  output Real nextCrossCovariance[15,6];
  output Real nextReferenceCovariance[6,6];
  output Real nextReferencePosition[3];
  output Real nextReferenceRotation[3,3];
  output Real nextReferenceAvailable;
  output Real nextReferenceEpoch;
  output Real nextReferenceUsed;
  output Real nextLastUsedEpoch;
  output Real nextReferenceDescriptor[featureCapacity,descriptorSize];
  output Real nextReferencePoint[featureCapacity,3];
  output Real nextReferenceEnabled[featureCapacity];
  output Real nextReferencePixels[featureCapacity,2];
  output Real nextReferenceCount;
  output Real nextReferenceRgbCalibration[4];
  output Real nextReferenceDepthCalibration[4];
  output Real nextReferenceNoiseReferenceFx;
  output Real nextReferenceDisparityNoise;
  output Real nextReferenceBaseline;
  output Real nextReferenceOpticalToBody[3,3];
  output Real nextReferenceCameraOriginBody[3];
  output Real predictionAccepted;
  output Real observationAccepted;
  output Real observationRejected;
  output Real captureAccepted;
  output Real captureRejected;
  output Real imageReuseRejected;
  output Real imagePairEligible;
  output Real frameValid;
  output Real referenceGeometryCompatible;
  output Real visualValid;
  output Real matchCount;
  output Real uncertaintyRejectionReason;
  output Real currentDescriptor[featureCapacity,descriptorSize];
  output Real currentPoint[featureCapacity,3];
  output Real currentEnabled[featureCapacity];
  output Real currentCount;
  output Real currentFromReference[3,3];
  output Real currentFromReferenceTranslation[3];
  output Real relativeCovariance[6,6];
  output Real mapCandidatePoint[featureCapacity,3];
  output Real mapCandidateEnabled[featureCapacity];
  output Real mapCandidateCount;
  output Real nextQuaternion[4];
  output Real positionCovariance[3,3];
  output Real attitudeCovariance[3,3];
  output Real confidence;
  output Real features[featureCapacity,3];
  output Real featureEnabled[featureCapacity];
  output Real trackingCurrentPixel[featureCapacity,2];
  output Real trackingReferencePixel[featureCapacity,2];
  output Real trackingEnabled[featureCapacity];
  input Real depthUnits = 1.0 "Meters per depth sample; 1 for metric depth, SDK scale for Z16";
protected
  constant Integer featureCapacity = 350;
  constant Integer descriptorSize = 49;
  constant Real initialPositionVariance = 0.25;
  constant Real initialVelocityVariance = 0.04;
  constant Real initialAttitudeVariance = 0.01;
  constant Real initialAccelBiasVariance = 0.0004;
  constant Real initialGyroBiasVariance = 0.000025;
  constant Real defaultNoiseReferenceFx = 848.0/(2.0*tan(87.0*3.141592653589793/360.0));
  constant Integer errorDimension = 15;
  constant Integer poseDimension = 6;
  Boolean imageOn; Boolean sameGeometry; Boolean currentOriginValid;
  Real usableReferenceCount; Real pairValid; Real pairEligible; Real captureFresh;
  Real predictedPosition[3]; Real predictedVelocity[3]; Real predictedRotation[3,3];
  Real predictedCovariance[errorDimension,errorDimension]; Real predictedCross[errorDimension,poseDimension];
  Real transition[errorDimension,errorDimension]; Real processCovariance[errorDimension,errorDimension];
  Integer substeps;
  Real correctedCovariance[errorDimension,errorDimension]; Real correctedCross[errorDimension,poseDimension];
  Real correctedReferenceCovariance[poseDimension,poseDimension];
  Real correctionNis; Real correctionInnovation[poseDimension];
  Real correctionJacobian[poseDimension,errorDimension+poseDimension]; Real innovationCovariance[poseDimension,poseDimension];
  Real measurementEnabled; Real captureRequest; Real pairAttempted; Real usedEpochAfterAttempt;
  Real projectionInvalidCount; Real projectionConfigurationValid; Real projectionPoseValid;
  Real orientationVector[3]; Real orientationAngle; Real orientationValid; Real orientationQuaternion[4];
  Real visual_observedBodyRotation[3,3];
  Real visual_observedBodyPosition[3];
  Real visual_valid;
  Real visual_registrationRms;
  Real visual_registrationRejectionReason;
  Real visual_currentIndex[featureCapacity];
  Real visual_conditionalObservationCovariance[6,6];
  Real visual_uncertaintyValidCount;
  Real visual_uncertaintyInvalidCount;
  Real visual_descriptionInvalidCount;
  Real visual_matchingConfigurationValid;
  Real visual_invalidReference;
  Real visual_invalidCurrent;
  Real visual_pairEnabled[featureCapacity];
  Real visual_registrationAccepted;
  Real visual_registrationValidCount;
  Real visual_registrationInvalidCount;
  Real visual_registrationRank;
  Real visual_uncertaintyValid;
algorithm
  imageOn := SLAMExactRealEqual(frameEnabled,1.0);
  sameGeometry := SLAMExactRealEqual(referenceBaseline,baseline)
    and SLAMExactRealEqual(referenceDisparityNoise,disparityNoise);
  currentOriginValid := true;
  for axis in 1:3 loop
    currentOriginValid := currentOriginValid and abs(cameraOriginBody[axis]) <= 10.0;
    sameGeometry := sameGeometry and SLAMExactRealEqual(referenceCameraOriginBody[axis],cameraOriginBody[axis]);
    for column in 1:3 loop
      sameGeometry := sameGeometry and SLAMExactRealEqual(referenceOpticalToBody[axis,column],opticalToBody[axis,column]);
    end for;
  end for;
  referenceGeometryCompatible := if sameGeometry then 1.0 else 0.0;
  usableReferenceCount := if SLAMExactRealEqual(referenceAvailable,1.0) and sameGeometry then referenceCount else 0.0;
  (pairValid,pairEligible,captureFresh) := SchmidtImagePairEligibility(
    referenceAvailable,referenceUsed,referenceEpoch,currentEpoch,lastUsedEpoch);
  (predictionAccepted,transition,processCovariance,predictedPosition,predictedVelocity,predictedRotation,
    predictedCovariance,predictedCross,substeps) := ES15PredictHeldInterval(
      position,velocity,rotation,accelBias,gyroBias,covariance,crossCovariance,referenceCovariance,
      referencePosition,referenceRotation,referenceAvailable,accel,gyro,gravity,h,density);
  // Visual diagnostics stay observable even when prediction refuses this interval.
  (visual_observedBodyRotation,
    visual_observedBodyPosition,
    visual_valid,
    matchCount,
    visual_registrationRms,
    visual_registrationRejectionReason,
    visual_currentIndex,
    currentDescriptor,
    currentPoint,
    currentEnabled,
    currentFromReference,
    currentFromReferenceTranslation,
    relativeCovariance,
    visual_conditionalObservationCovariance,
    visualValid,
    uncertaintyRejectionReason,
    visual_uncertaintyValidCount,
    visual_uncertaintyInvalidCount,
    visual_descriptionInvalidCount,
    visual_matchingConfigurationValid,
    visual_invalidReference,
    visual_invalidCurrent,
    visual_pairEnabled,
    visual_registrationAccepted,
    visual_registrationValidCount,
    visual_registrationInvalidCount,
    visual_registrationRank,
    visual_uncertaintyValid) := ObserveRGBDRelativeFrame(
    rgb=rgb,depth=depth,pixels=pixels,activeCount=if imageOn then activeCount else 0.0,
    rgbCalibration=rgbCalibration,depthCalibration=depthCalibration,noiseReferenceFx=noiseReferenceFx,
    referenceDescriptor=referenceDescriptor,referencePoint=referencePoint,referenceEnabled=referenceEnabled,
    referenceCount=usableReferenceCount,referenceRgbFocal={referenceRgbCalibration[1],referenceRgbCalibration[2]},
    referenceNoiseReferenceFx=referenceNoiseReferenceFx,imageEnabled=imageOn,disparityNoise=disparityNoise,baseline=baseline,
    referenceBodyPosition=referencePosition,referenceBodyRotation=referenceRotation,
    opticalToBody=opticalToBody,cameraOriginBody=cameraOriginBody,depthUnits=depthUnits);
  frameValid := if imageOn and activeCount >= 0.0 and activeCount <= featureCapacity
    and SLAMExactRealEqual(activeCount,floor(activeCount)) and sum(currentEnabled) >= 3.0
    and disparityNoise > 0.0 and disparityNoise <= 1.0 and noiseReferenceFx >= 1e-6 and noiseReferenceFx <= 1e6
    and baseline >= 1e-6 and baseline <= 1.0 and RGBDProperRotationValue(opticalToBody) > 0.5
    and currentOriginValid then 1.0 else 0.0;
  measurementEnabled := if frameValid > 0.5 and sameGeometry and visualValid > 0.5 then 1.0 else 0.0;
  captureRequest := if frameValid > 0.5 then imageCaptureRequested else 0.0;
  (observationAccepted,correctionNis,correctionInnovation,correctionJacobian,innovationCovariance,
    nextPosition,nextVelocity,nextRotation,nextAccelBias,nextGyroBias,correctedCovariance,correctedCross,
    correctedReferenceCovariance) := SchmidtCorrectRelativePose(
      position=predictedPosition,velocity=predictedVelocity,rotation=predictedRotation,accelBias=accelBias,gyroBias=gyroBias,
      covariance=predictedCovariance,crossCovariance=predictedCross,referenceCovariance=referenceCovariance,
      referencePosition=referencePosition,referenceRotation=referenceRotation,opticalToBody=opticalToBody,
      cameraOriginBody=cameraOriginBody,measuredRotation=currentFromReference,measuredTranslation=currentFromReferenceTranslation,
      relativeCovariance=relativeCovariance,measurementEnabled=if predictionAccepted > 0.5 and pairEligible > 0.5 then measurementEnabled else 0.0);
  // An eligible attempted pair consumes its epoch even when correction rejects.
  pairAttempted := if predictionAccepted > 0.5 and pairEligible > 0.5
    and SLAMExactRealEqual(measurementEnabled,1.0) then 1.0 else 0.0;
  usedEpochAfterAttempt := if pairAttempted > 0.5 then currentEpoch else lastUsedEpoch;
  imagePairEligible := pairEligible;
  imageReuseRejected := if predictionAccepted > 0.5 and SLAMExactRealEqual(measurementEnabled,1.0)
    and pairEligible < 0.5 then 1.0 else 0.0;
  observationRejected := if predictionAccepted > 0.5 and not SLAMExactRealEqual(measurementEnabled,0.0)
    and observationAccepted < 0.5 then 1.0 else 0.0;
  (captureAccepted,captureRejected,nextReferenceAvailable,nextReferencePosition,nextReferenceRotation,
    nextCovariance,nextCrossCovariance,nextReferenceCovariance) := SchmidtCaptureReference(
      nextPosition,nextRotation,correctedCovariance,referencePosition,referenceRotation,
      correctedCross,correctedReferenceCovariance,referenceAvailable,captureRequest,
      if predictionAccepted > 0.5 and captureFresh > 0.5 and currentEpoch > usedEpochAfterAttempt then 1.0 else 0.0);
  if imageOn and SLAMExactRealEqual(imageCaptureRequested,1.0) and frameValid < 0.5 then captureRejected := 1.0; end if;
  nextReferenceEpoch := if captureAccepted > 0.5 then currentEpoch else referenceEpoch;
  nextReferenceUsed := if captureAccepted > 0.5 then 0.0 else if pairAttempted > 0.5 then 1.0 else referenceUsed;
  nextLastUsedEpoch := usedEpochAfterAttempt;
  nextReferenceDescriptor := if captureAccepted > 0.5 then currentDescriptor else referenceDescriptor;
  nextReferencePoint := if captureAccepted > 0.5 then currentPoint else referencePoint;
  nextReferenceEnabled := if captureAccepted > 0.5 then currentEnabled else referenceEnabled;
  nextReferencePixels := if captureAccepted > 0.5 then pixels else referencePixels;
  nextReferenceCount := if captureAccepted > 0.5 then activeCount else referenceCount;
  nextReferenceRgbCalibration := if captureAccepted > 0.5 then rgbCalibration else referenceRgbCalibration;
  nextReferenceDepthCalibration := if captureAccepted > 0.5 then depthCalibration else referenceDepthCalibration;
  nextReferenceNoiseReferenceFx := if captureAccepted > 0.5 then noiseReferenceFx else referenceNoiseReferenceFx;
  nextReferenceDisparityNoise := if captureAccepted > 0.5 then disparityNoise else referenceDisparityNoise;
  nextReferenceBaseline := if captureAccepted > 0.5 then baseline else referenceBaseline;
  nextReferenceOpticalToBody := if captureAccepted > 0.5 then opticalToBody else referenceOpticalToBody;
  nextReferenceCameraOriginBody := if captureAccepted > 0.5 then cameraOriginBody else referenceCameraOriginBody;
  (mapCandidatePoint,mapCandidateEnabled,mapCandidateCount,projectionInvalidCount,projectionConfigurationValid,projectionPoseValid)
    := RGBDProjectLandmarks(currentPoint,currentEnabled,if imageOn then activeCount else 0.0,
      if imageOn and (observationAccepted > 0.5 or captureAccepted > 0.5) then 1.0 else 0.0,
      nextRotation,nextPosition,opticalToBody,cameraOriginBody);
  (orientationVector,orientationAngle,orientationValid,orientationQuaternion) := SLAMRotationCoordinates(nextRotation);
  nextQuaternion := if orientationValid > 0.5 then orientationQuaternion else {1.0,0.0,0.0,0.0};
  positionCovariance := nextCovariance[1:3,1:3]; attitudeCovariance := nextCovariance[7:9,7:9];
  confidence := observationAccepted; featureEnabled := currentEnabled;
  currentCount := if imageOn then activeCount else 0.0;
  for feature in 1:featureCapacity loop
    features[feature,1] := if currentEnabled[feature] > 0.5 then pixels[feature,1] else 0.0;
    features[feature,2] := if currentEnabled[feature] > 0.5 then pixels[feature,2] else 0.0;
    features[feature,3] := if currentEnabled[feature] > 0.5 and abs(featureScore[feature]) <= 1e30 then featureScore[feature] else 0.0;
  end for;
  (trackingCurrentPixel,trackingReferencePixel,trackingEnabled) := RGBDLocalizationTracking(
    visual_currentIndex,pixels,referencePixels,referenceEnabled,currentEnabled,
    {size(rgb,1),size(rgb,2)});
end AdvanceRGBDLocalization;
