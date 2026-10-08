within SLAM;
// Ordered raw-camera lifecycle. Reference qualification is separate from
// compiler issuance and complete browser/runtime admission.
function InitializeFastSLAM
  import InitializeFastRGBDLocalization = SLAM.Localization.InitializeFastRGBDLocalization;
  import RGBDGraphProcessing = SLAM.PoseGraph.RGBDGraphProcessing;
  import RGBDKeyframes = SLAM.LoopClosure.RGBDKeyframes;
  import RGBDLocalizationCatalog = SLAM.Localization.RGBDLocalizationCatalog;
  import RGBDLocalizationFrame = SLAM.Localization.RGBDLocalizationFrame;
  import RGBDLocalizationProcessing = SLAM.Localization.RGBDLocalizationProcessing;
  import RGBDNominalCalibration = Vision.Matching.RGBDNominalCalibration;
  import SLAMRotationCoordinates = SLAM.Inertial.SLAMRotationCoordinates;

  input RGBDGraphProcessing.State previous;
  input Real rgb[:,:,:];
  input Real depth[size(rgb,1),size(rgb,2)];
  input Real rgbCalibration[4] = RGBDNominalCalibration({size(rgb,1),size(rgb,2)},{69.0,42.0});
  input Real depthCalibration[4] = RGBDNominalCalibration({size(rgb,1),size(rgb,2)},{87.0,58.0});
  input Real disparityNoise = 0.08;
  input Real noiseReferenceFx = 848.0/(2.0*tan(87.0*3.141592653589793/360.0));
  input Real baseline = 0.05;
  input Real opticalToBody[spaceDimension,spaceDimension] = [0.0,0.0,1.0;-1.0,0.0,0.0;0.0,-1.0,0.0];
  input Real cameraOriginBody[spaceDimension] = {0.18,0.0,-0.04};
  input Real accel[spaceDimension] = {0.0,0.0,9.81};
  input Real gyro[spaceDimension] = zeros(spaceDimension);
  input Real gravity[spaceDimension] = {0.0,0.0,-9.81};
  input Real density[noiseDimension] = {0.06,0.06,0.06,0.006,0.006,0.006,0.002,0.002,0.002,0.0002,0.0002,0.0002};
  input Real h = 0.0;
  input Real intervalTime = 0.0;
  input Integer imageEpoch;
  input Boolean imageRequested = true;
  input Boolean localCaptureRequested = true;
  input Boolean requested = true;
  input Integer minimumMeasuredDescriptors = 8;
  input Real minimumWordDistanceSquared = 0.04;
  input Real selectedFeatureLimit = featureCapacity;
  input Real absoluteThreshold = 18.0;
  input Real minimumInterval = 0.5;
  input Real maximumInterval = 2.0;
  input Real translationThreshold = 0.6;
  input Real rotationThreshold = 0.25;
  input Integer minimumFeatures = 12;
  input Real voxelWidth = 0.25;
  input Real mergeRadius = 0.15;
  input Real maximumDistance = 80.0;
  input Real tentativeLifetime = 0.5;
  input Real confirmedLifetime = 5.0;
  input Real confirmationObservations = 3.0;
  input Real maximumConfidence = 8.0;
  input Real maximumTentative = 700.0;
  input Boolean graphCorrectionRequested = true;
  input RGBDGraphProcessing.Policy graphPolicy = RGBDGraphProcessing.DefaultPolicy();
  output RGBDGraphProcessing.State next;
  output Boolean accepted;
  output Boolean imageCompleted;
  output Boolean mappingAccepted;
  output Boolean graphCorrectionAccepted;
  output Boolean roundoffCertified;
  output Integer publicationReason;
  output Integer ledgerReason;
  output Integer vocabularyReason;
  output Integer graphReason;
  output Integer graphCommitReason;
  output Integer graphFilterReason;
  output Integer covarianceStatus;
  output Real graphCostBefore;
  output Real graphCostAfter;
  output Real nextQuaternion[quaternionSize];
  output Real selectionValid;
  output Real predictionAccepted;
  output Real initializationAccepted;
  output Real observationAccepted;
  output Real captureAccepted;
  output Real matchCount;
  output Real features[featureCapacity,spaceDimension];
  output Real featureEnabled[featureCapacity];
  output Real trackingCurrentPixel[featureCapacity,2];
  output Real trackingReferencePixel[featureCapacity,2];
  output Real trackingEnabled[featureCapacity];
  input Boolean depthQualifiedSelection = true;
  input Real depthUnits = 1.0 "Meters per depth sample; 1 for metric depth, SDK scale for Z16";
protected
  constant Integer featureCapacity = RGBDKeyframes.featureCapacity;
  constant Integer descriptorSize = RGBDKeyframes.descriptorSize;
  constant Integer spaceDimension = RGBDKeyframes.dimension;
  constant Integer errorDimension = 5*spaceDimension;
  constant Integer poseDimension = 2*spaceDimension;
  constant Integer noiseDimension = 4*spaceDimension;
  constant Integer channelCount = 4;
  constant Integer quaternionSize = 4;
  Boolean transactionAllowed;
  Boolean imageOn;
  RGBDLocalizationCatalog.Estimator proposed;
  RGBDLocalizationProcessing.Result publication;
  RGBDKeyframes.Frame measurement;
  Boolean frameAccepted;
  Integer frameRejectionReason;
  Real orientationVector[spaceDimension];
  Real orientationAngle;
  Real orientationValid;
  Real orientationQuaternion[quaternionSize];
  Real producerNextPosition[spaceDimension];
  Real producerNextVelocity[spaceDimension];
  Real producerNextRotation[spaceDimension,spaceDimension];
  Real producerNextAccelBias[spaceDimension];
  Real producerNextGyroBias[spaceDimension];
  Real producerNextCovariance[errorDimension,errorDimension];
  Real producerNextCrossCovariance[errorDimension,poseDimension];
  Real producerNextReferenceCovariance[poseDimension,poseDimension];
  Real producerNextReferencePosition[spaceDimension];
  Real producerNextReferenceRotation[spaceDimension,spaceDimension];
  Real producerNextReferenceAvailable;
  Real producerNextReferenceEpoch;
  Real producerNextReferenceUsed;
  Real producerNextLastUsedEpoch;
  Real producerNextReferenceDescriptor[featureCapacity,descriptorSize];
  Real producerNextReferencePoint[featureCapacity,spaceDimension];
  Real producerNextReferenceEnabled[featureCapacity];
  Real producerNextReferencePixels[featureCapacity,2];
  Real producerNextReferenceCount;
  Real producerNextReferenceRgbCalibration[4];
  Real producerNextReferenceDepthCalibration[4];
  Real producerNextReferenceNoiseReferenceFx;
  Real producerNextReferenceDisparityNoise;
  Real producerNextReferenceBaseline;
  Real producerNextReferenceOpticalToBody[spaceDimension,spaceDimension];
  Real producerNextReferenceCameraOriginBody[spaceDimension];
  Real producerPredictionAccepted;
  Real producerObservationAccepted;
  Real producerObservationRejected;
  Real producerCaptureAccepted;
  Real producerCaptureRejected;
  Real producerImageReuseRejected;
  Real producerImagePairEligible;
  Real producerFrameValid;
  Real producerReferenceGeometryCompatible;
  Real producerVisualValid;
  Real producerMatchCount;
  Real producerUncertaintyRejectionReason;
  Real producerCurrentDescriptor[featureCapacity,descriptorSize];
  Real producerCurrentPoint[featureCapacity,spaceDimension];
  Real producerCurrentEnabled[featureCapacity];
  Real producerCurrentCount;
  Real producerCurrentFromReference[spaceDimension,spaceDimension];
  Real producerCurrentFromReferenceTranslation[spaceDimension];
  Real producerRelativeCovariance[poseDimension,poseDimension];
  Real producerMapCandidatePoint[featureCapacity,spaceDimension];
  Real producerMapCandidateEnabled[featureCapacity];
  Real producerMapCandidateCount;
  Real producerNextQuaternion[quaternionSize];
  Real producerPositionCovariance[spaceDimension,spaceDimension];
  Real producerAttitudeCovariance[spaceDimension,spaceDimension];
  Real producerConfidence;
  Real producerFeatures[featureCapacity,spaceDimension];
  Real producerFeatureEnabled[featureCapacity];
  Real producerTrackingCurrentPixel[featureCapacity,2];
  Real producerTrackingReferencePixel[featureCapacity,2];
  Real producerTrackingEnabled[featureCapacity];
  Real producerInitializationAccepted;
  Real producerInitializationRejected;
  Real producerSelectionValid;
algorithm
  transactionAllowed := if requested then RGBDGraphProcessing.Valid(previous) and RGBDLocalizationCatalog.CanAdvance(previous.estimator.localization,intervalTime,h,true,requested) else false;
  imageOn := transactionAllowed and imageRequested
    and RGBDLocalizationCatalog.ImageFresh(previous.estimator.localization,imageEpoch,intervalTime);
  (producerNextPosition,
    producerNextVelocity,
    producerNextRotation,
    producerNextAccelBias,
    producerNextGyroBias,
    producerNextCovariance,
    producerNextCrossCovariance,
    producerNextReferenceCovariance,
    producerNextReferencePosition,
    producerNextReferenceRotation,
    producerNextReferenceAvailable,
    producerNextReferenceEpoch,
    producerNextReferenceUsed,
    producerNextLastUsedEpoch,
    producerNextReferenceDescriptor,
    producerNextReferencePoint,
    producerNextReferenceEnabled,
    producerNextReferencePixels,
    producerNextReferenceCount,
    producerNextReferenceRgbCalibration,
    producerNextReferenceDepthCalibration,
    producerNextReferenceNoiseReferenceFx,
    producerNextReferenceDisparityNoise,
    producerNextReferenceBaseline,
    producerNextReferenceOpticalToBody,
    producerNextReferenceCameraOriginBody,
    producerPredictionAccepted,
    producerObservationAccepted,
    producerObservationRejected,
    producerCaptureAccepted,
    producerCaptureRejected,
    producerImageReuseRejected,
    producerImagePairEligible,
    producerFrameValid,
    producerReferenceGeometryCompatible,
    producerVisualValid,
    producerMatchCount,
    producerUncertaintyRejectionReason,
    producerCurrentDescriptor,
    producerCurrentPoint,
    producerCurrentEnabled,
    producerCurrentCount,
    producerCurrentFromReference,
    producerCurrentFromReferenceTranslation,
    producerRelativeCovariance,
    producerMapCandidatePoint,
    producerMapCandidateEnabled,
    producerMapCandidateCount,
    producerNextQuaternion,
    producerPositionCovariance,
    producerAttitudeCovariance,
    producerConfidence,
    producerFeatures,
    producerFeatureEnabled,
    producerTrackingCurrentPixel,
    producerTrackingReferencePixel,
    producerTrackingEnabled,
    producerInitializationAccepted,
    producerInitializationRejected,
    producerSelectionValid) := InitializeFastRGBDLocalization(
    imageTime=intervalTime,
    initializationRequested=if transactionAllowed then 1.0 else 0.0,
    selectedFeatureLimit=selectedFeatureLimit,
    absoluteThreshold=absoluteThreshold,
      depthQualifiedSelection=depthQualifiedSelection,
    rgb=rgb,
    depth=depth,
    rgbCalibration=rgbCalibration,
    depthCalibration=depthCalibration,
    disparityNoise=disparityNoise,
    noiseReferenceFx=noiseReferenceFx,
    baseline=baseline,
    opticalToBody=opticalToBody,
    cameraOriginBody=cameraOriginBody,
    accel=accel,
    gyro=gyro,
    gravity=gravity,
    density=density,
    position=previous.estimator.localization.estimator.position,
    velocity=previous.estimator.localization.estimator.velocity,
    rotation=previous.estimator.localization.estimator.rotation,
    accelBias=previous.estimator.localization.estimator.accelBias,
    gyroBias=previous.estimator.localization.estimator.gyroBias,
    covariance=previous.estimator.localization.estimator.covariance,
    crossCovariance=previous.estimator.localization.estimator.crossCovariance,
    referenceCovariance=previous.estimator.localization.estimator.referenceCovariance,
    referencePosition=previous.estimator.localization.estimator.referencePosition,
    referenceRotation=previous.estimator.localization.estimator.referenceRotation,
    referenceAvailable=previous.estimator.localization.estimator.referenceAvailable,
    referenceEpoch=previous.estimator.localization.estimator.referenceEpoch,
    referenceUsed=previous.estimator.localization.estimator.referenceUsed,
    lastUsedEpoch=previous.estimator.localization.estimator.lastUsedEpoch,
    referenceDescriptor=previous.estimator.localization.estimator.referenceDescriptor,
    referencePoint=previous.estimator.localization.estimator.referencePoint,
    referenceEnabled=previous.estimator.localization.estimator.referenceEnabled,
    referencePixels=previous.estimator.localization.estimator.referencePixels,
    referenceCount=previous.estimator.localization.estimator.referenceCount,
    referenceRgbCalibration=previous.estimator.localization.estimator.referenceRgbCalibration,
    referenceDepthCalibration=previous.estimator.localization.estimator.referenceDepthCalibration,
    referenceNoiseReferenceFx=previous.estimator.localization.estimator.referenceNoiseReferenceFx,
    referenceDisparityNoise=previous.estimator.localization.estimator.referenceDisparityNoise,
    referenceBaseline=previous.estimator.localization.estimator.referenceBaseline,
    referenceOpticalToBody=previous.estimator.localization.estimator.referenceOpticalToBody,
    referenceCameraOriginBody=previous.estimator.localization.estimator.referenceCameraOriginBody,
    currentEpoch=imageEpoch,
    h=h,
    frameEnabled=if imageOn then 1.0 else 0.0,
    imageCaptureRequested=if imageOn and localCaptureRequested then 1.0 else 0.0,depthUnits=depthUnits);
  proposed.position := producerNextPosition;
  proposed.velocity := producerNextVelocity;
  proposed.rotation := producerNextRotation;
  proposed.accelBias := producerNextAccelBias;
  proposed.gyroBias := producerNextGyroBias;
  proposed.covariance := producerNextCovariance;
  proposed.crossCovariance := producerNextCrossCovariance;
  proposed.referenceCovariance := producerNextReferenceCovariance;
  proposed.referencePosition := producerNextReferencePosition;
  proposed.referenceRotation := producerNextReferenceRotation;
  proposed.referenceAvailable := producerNextReferenceAvailable;
  proposed.referenceEpoch := producerNextReferenceEpoch;
  proposed.referenceUsed := producerNextReferenceUsed;
  proposed.lastUsedEpoch := producerNextLastUsedEpoch;
  proposed.referenceDescriptor := producerNextReferenceDescriptor;
  proposed.referencePoint := producerNextReferencePoint;
  proposed.referenceEnabled := producerNextReferenceEnabled;
  proposed.referencePixels := producerNextReferencePixels;
  proposed.referenceCount := producerNextReferenceCount;
  proposed.referenceRgbCalibration := producerNextReferenceRgbCalibration;
  proposed.referenceDepthCalibration := producerNextReferenceDepthCalibration;
  proposed.referenceNoiseReferenceFx := producerNextReferenceNoiseReferenceFx;
  proposed.referenceDisparityNoise := producerNextReferenceDisparityNoise;
  proposed.referenceBaseline := producerNextReferenceBaseline;
  proposed.referenceOpticalToBody := producerNextReferenceOpticalToBody;
  proposed.referenceCameraOriginBody := producerNextReferenceCameraOriginBody;
  (measurement,frameAccepted,frameRejectionReason) := RGBDLocalizationFrame.Build(
    previous.estimator.localization.generation,previous.estimator.localization.catalog.nextId,imageEpoch,imageEpoch,intervalTime,
    producerCurrentCount,producerCurrentDescriptor,producerCurrentPoint,
    producerCurrentEnabled,producerFeatures[:,1:2],{size(rgb,1),size(rgb,2)},{size(rgb,1),size(rgb,2)},
    rgbCalibration,depthCalibration,opticalToBody,cameraOriginBody,disparityNoise,noiseReferenceFx,baseline,
    proposed.position,proposed.rotation,proposed.covariance,previous.estimator.localization.catalog.vocabularyVersion,
    if producerFrameValid > 0.5 and (producerObservationAccepted > 0.5
      or producerCaptureAccepted > 0.5) then 1.0 else 0.0,
    imageOn and producerInitializationAccepted > 0.5);
  publication := RGBDLocalizationProcessing.Publish(previous,proposed,measurement,
    producerInitializationAccepted,
    producerObservationAccepted,producerCaptureAccepted,imageOn,imageEpoch,intervalTime,h,
    true,frameAccepted,frameRejectionReason,transactionAllowed,
    producerMapCandidatePoint,producerMapCandidateEnabled,
    minimumInterval,maximumInterval,translationThreshold,rotationThreshold,minimumFeatures,
    voxelWidth,mergeRadius,maximumDistance,tentativeLifetime,confirmedLifetime,
    confirmationObservations,maximumConfidence,maximumTentative,
    minimumMeasuredDescriptors=minimumMeasuredDescriptors,minimumWordDistanceSquared=minimumWordDistanceSquared);
  accepted := publication.accepted;
  imageCompleted := publication.publication.imageCompleted;
  mappingAccepted := publication.publication.mappingAccepted;
  publicationReason := publication.reason;
  ledgerReason := publication.ledgerReason;
  vocabularyReason := publication.vocabularyReason;
  // One initialized capture cannot form a relative graph. The next Step
  // invokes correction only after a real admitted catalog capture.
  next := publication.next;
  graphCorrectionAccepted := false;
  graphReason := 1;
  graphCommitReason := 0;
  graphFilterReason := 0;
  covarianceStatus := 0;
  roundoffCertified := false;
  graphCostBefore := 0;
  graphCostAfter := 0;
  (orientationVector,orientationAngle,orientationValid,orientationQuaternion) :=
    SLAMRotationCoordinates(next.estimator.localization.estimator.rotation);
  nextQuaternion := if orientationValid > 0.5 then orientationQuaternion else {1.0,0.0,0.0,0.0};
  selectionValid := producerSelectionValid;
  predictionAccepted := 0.0;
  initializationAccepted := if publication.accepted then producerInitializationAccepted else 0.0;
  observationAccepted := if publication.accepted then producerObservationAccepted else 0.0;
  captureAccepted := if publication.accepted then producerCaptureAccepted else 0.0;
  matchCount := if publication.publication.imageCompleted then producerMatchCount else 0.0;
  features := if publication.publication.imageCompleted then producerFeatures else zeros(featureCapacity,3);
  featureEnabled := if publication.publication.imageCompleted then producerFeatureEnabled else zeros(featureCapacity);
  trackingCurrentPixel := if publication.publication.imageCompleted then producerTrackingCurrentPixel else zeros(featureCapacity,2);
  trackingReferencePixel := if publication.publication.imageCompleted then producerTrackingReferencePixel else zeros(featureCapacity,2);
  trackingEnabled := if publication.publication.imageCompleted then producerTrackingEnabled else zeros(featureCapacity);
end InitializeFastSLAM;
