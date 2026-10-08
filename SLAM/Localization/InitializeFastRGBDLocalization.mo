within SLAM.Localization;
// Complete raw FAST -> selection -> time-zero RGB-D initialization transaction.
// All image/ranking/pose/covariance math is Modelica; no host intermediate stage.
function InitializeFastRGBDLocalization
  import FastFrameScores = Vision.Features.FastFrameScores;
  import FastSelectionScoreFloor = Vision.Features.FastSelectionScoreFloor;
  import InitializeRGBDLocalization = SLAM.Localization.InitializeRGBDLocalization;
  import RGBDDepthQualifiedScores = Vision.Matching.RGBDDepthQualifiedScores;
  import SLAMExactRealEqual = SLAM.Inertial.SLAMExactRealEqual;
  import SelectRasterFeatures = Vision.Features.SelectRasterFeatures;

  input Real rgb[:,:,:];
  input Real depth[size(rgb,1),size(rgb,2)];
  input Real rgbCalibration[4];
  input Real depthCalibration[4];
  input Real disparityNoise;
  input Real noiseReferenceFx;
  input Real baseline;
  input Real opticalToBody[3,3];
  input Real cameraOriginBody[3];
  input Real frameEnabled;
  input Real imageCaptureRequested;
  input Real position[3];
  input Real velocity[3];
  input Real rotation[3,3];
  input Real accelBias[3];
  input Real gyroBias[3];
  input Real covariance[15,15];
  input Real crossCovariance[15,6];
  input Real referenceCovariance[6,6];
  input Real referencePosition[3];
  input Real referenceRotation[3,3];
  input Real referenceAvailable;
  input Real referenceEpoch;
  input Real currentEpoch;
  input Real referenceUsed;
  input Real lastUsedEpoch;
  input Real referenceDescriptor[featureCapacity,descriptorSize];
  input Real referencePoint[featureCapacity,3];
  input Real referenceEnabled[featureCapacity];
  input Real referencePixels[featureCapacity,2];
  input Real referenceCount;
  input Real referenceRgbCalibration[4];
  input Real referenceDepthCalibration[4];
  input Real referenceNoiseReferenceFx;
  input Real referenceDisparityNoise;
  input Real referenceBaseline;
  input Real referenceOpticalToBody[3,3];
  input Real referenceCameraOriginBody[3];
  input Real accel[3];
  input Real gyro[3];
  input Real gravity[3];
  input Real h;
  input Real density[12];
  input Real imageTime;
  input Real initializationRequested;
  input Real selectedFeatureLimit = 350.0;
  input Real absoluteThreshold = 18.0;
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
  output Real initializationAccepted;
  output Real initializationRejected;
  output Real selectionValid;
  input Boolean depthQualifiedSelection = false "Reserve the RGB-D budget for valid calibrated depth";
  input Real depthUnits = 1.0 "Meters per depth sample; 1 for metric depth, SDK scale for Z16";
protected
  constant Integer featureCapacity = 350; constant Integer descriptorSize = 49;
  Boolean imageOn; Boolean selectionConfiguration;
  Real scores[size(rgb,1)*size(rgb,2)];
  Real selectedFeatures[featureCapacity,3];
  Real selectionCount; Real selectionStatus; Real selectedCount;
algorithm
  imageOn := SLAMExactRealEqual(frameEnabled,1.0);
  scores := FastFrameScores(rgb,imageOn,FastSelectionScoreFloor(absoluteThreshold,1e8));
  if depthQualifiedSelection then
    scores := RGBDDepthQualifiedScores(depth,scores,rgbCalibration,depthCalibration,
      0.28,10.0,disparityNoise,noiseReferenceFx,baseline,imageOn,depthUnits=depthUnits);
  end if;
  (selectedFeatures,selectionCount,selectionStatus) := SelectRasterFeatures(scores,
    size(rgb,2),size(rgb,1),featureCapacity,3,
    {absoluteThreshold,0.0,1e8,3.0,selectedFeatureLimit,1.0,3.0,3.0},false,imageOn);
  selectionConfiguration := selectedFeatureLimit >= 1.0 and selectedFeatureLimit <= featureCapacity
    and SLAMExactRealEqual(selectedFeatureLimit,floor(selectedFeatureLimit));
  selectionValid := if selectionConfiguration and selectionStatus > 0.5
    and selectionCount >= 0.0 and selectionCount <= featureCapacity then 1.0 else 0.0;
  selectedCount := if selectionValid > 0.5 then selectionCount else 0.0;
  (nextPosition,nextVelocity,nextRotation,
    nextAccelBias,nextGyroBias,nextCovariance,
    nextCrossCovariance,nextReferenceCovariance,nextReferencePosition,
    nextReferenceRotation,nextReferenceAvailable,nextReferenceEpoch,
    nextReferenceUsed,nextLastUsedEpoch,nextReferenceDescriptor,
    nextReferencePoint,nextReferenceEnabled,nextReferencePixels,
    nextReferenceCount,nextReferenceRgbCalibration,nextReferenceDepthCalibration,
    nextReferenceNoiseReferenceFx,nextReferenceDisparityNoise,nextReferenceBaseline,
    nextReferenceOpticalToBody,nextReferenceCameraOriginBody,predictionAccepted,
    observationAccepted,observationRejected,captureAccepted,
    captureRejected,imageReuseRejected,imagePairEligible,
    frameValid,referenceGeometryCompatible,visualValid,
    matchCount,uncertaintyRejectionReason,currentDescriptor,
    currentPoint,currentEnabled,currentCount,
    currentFromReference,currentFromReferenceTranslation,relativeCovariance,
    mapCandidatePoint,mapCandidateEnabled,mapCandidateCount,
    nextQuaternion,positionCovariance,attitudeCovariance,
    confidence,features,featureEnabled,
    trackingCurrentPixel,trackingReferencePixel,trackingEnabled,
    initializationAccepted,initializationRejected) := InitializeRGBDLocalization(
    rgb,depth,rgbCalibration,
    depthCalibration,disparityNoise,noiseReferenceFx,
    baseline,opticalToBody,cameraOriginBody,
    frameEnabled,imageCaptureRequested,position,
    velocity,rotation,accelBias,
    gyroBias,covariance,crossCovariance,
    referenceCovariance,referencePosition,referenceRotation,
    referenceAvailable,referenceEpoch,currentEpoch,
    referenceUsed,lastUsedEpoch,referenceDescriptor,
    referencePoint,referenceEnabled,referencePixels,
    referenceCount,referenceRgbCalibration,referenceDepthCalibration,
    referenceNoiseReferenceFx,referenceDisparityNoise,referenceBaseline,
    referenceOpticalToBody,referenceCameraOriginBody,accel,
    gyro,gravity,h,
    density,selectedFeatures[1:featureCapacity,1:2],selectedCount,
    selectedFeatures[1:featureCapacity,3],imageTime,initializationRequested,depthUnits=depthUnits);
end InitializeFastRGBDLocalization;
