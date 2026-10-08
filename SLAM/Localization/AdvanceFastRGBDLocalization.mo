within SLAM.Localization;
// Full-raster source-owned FAST selection composed with the qualified core.
// The host transports the complete state and does no selection or filter math.
pure function AdvanceFastRGBDLocalization
  import AdvanceRGBDLocalization = SLAM.Localization.AdvanceRGBDLocalization;
  import FastFrameScores = Vision.Features.FastFrameScores;
  import FastSelectionScoreFloor = Vision.Features.FastSelectionScoreFloor;
  import RGBDDepthQualifiedScores = Vision.Matching.RGBDDepthQualifiedScores;
  import RGBDNominalCalibration = Vision.Matching.RGBDNominalCalibration;
  import SLAMExactRealEqual = SLAM.Inertial.SLAMExactRealEqual;
  import SelectRasterFeatures = Vision.Features.SelectRasterFeatures;

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
  output Real selectionValid;
  input Boolean depthQualifiedSelection = false "Reserve the RGB-D budget for valid calibrated depth";
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
  Boolean imageOn; Boolean selectionConfiguration;
  Real scores[size(rgb,1)*size(rgb,2)]; Real selectedFeatures[featureCapacity,3];
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
  (nextPosition,
    nextVelocity,
    nextRotation,
    nextAccelBias,
    nextGyroBias,
    nextCovariance,
    nextCrossCovariance,
    nextReferenceCovariance,
    nextReferencePosition,
    nextReferenceRotation,
    nextReferenceAvailable,
    nextReferenceEpoch,
    nextReferenceUsed,
    nextLastUsedEpoch,
    nextReferenceDescriptor,
    nextReferencePoint,
    nextReferenceEnabled,
    nextReferencePixels,
    nextReferenceCount,
    nextReferenceRgbCalibration,
    nextReferenceDepthCalibration,
    nextReferenceNoiseReferenceFx,
    nextReferenceDisparityNoise,
    nextReferenceBaseline,
    nextReferenceOpticalToBody,
    nextReferenceCameraOriginBody,
    predictionAccepted,
    observationAccepted,
    observationRejected,
    captureAccepted,
    captureRejected,
    imageReuseRejected,
    imagePairEligible,
    frameValid,
    referenceGeometryCompatible,
    visualValid,
    matchCount,
    uncertaintyRejectionReason,
    currentDescriptor,
    currentPoint,
    currentEnabled,
    currentCount,
    currentFromReference,
    currentFromReferenceTranslation,
    relativeCovariance,
    mapCandidatePoint,
    mapCandidateEnabled,
    mapCandidateCount,
    nextQuaternion,
    positionCovariance,
    attitudeCovariance,
    confidence,
    features,
    featureEnabled,
    trackingCurrentPixel,
    trackingReferencePixel,
    trackingEnabled) := AdvanceRGBDLocalization(
    rgb=rgb,
    depth=depth,
    rgbCalibration=rgbCalibration,
    depthCalibration=depthCalibration,
    disparityNoise=disparityNoise,
    noiseReferenceFx=noiseReferenceFx,
    baseline=baseline,
    opticalToBody=opticalToBody,
    cameraOriginBody=cameraOriginBody,
    frameEnabled=frameEnabled,
    imageCaptureRequested=imageCaptureRequested,
    position=position,
    velocity=velocity,
    rotation=rotation,
    accelBias=accelBias,
    gyroBias=gyroBias,
    covariance=covariance,
    crossCovariance=crossCovariance,
    referenceCovariance=referenceCovariance,
    referencePosition=referencePosition,
    referenceRotation=referenceRotation,
    referenceAvailable=referenceAvailable,
    referenceEpoch=referenceEpoch,
    currentEpoch=currentEpoch,
    referenceUsed=referenceUsed,
    lastUsedEpoch=lastUsedEpoch,
    referenceDescriptor=referenceDescriptor,
    referencePoint=referencePoint,
    referenceEnabled=referenceEnabled,
    referencePixels=referencePixels,
    referenceCount=referenceCount,
    referenceRgbCalibration=referenceRgbCalibration,
    referenceDepthCalibration=referenceDepthCalibration,
    referenceNoiseReferenceFx=referenceNoiseReferenceFx,
    referenceDisparityNoise=referenceDisparityNoise,
    referenceBaseline=referenceBaseline,
    referenceOpticalToBody=referenceOpticalToBody,
    referenceCameraOriginBody=referenceCameraOriginBody,
    accel=accel,
    gyro=gyro,
    gravity=gravity,
    h=h,
    density=density,
    pixels=selectedFeatures[1:featureCapacity,1:2],activeCount=selectedCount,
    featureScore=selectedFeatures[1:featureCapacity,3],depthUnits=depthUnits);
end AdvanceFastRGBDLocalization;
