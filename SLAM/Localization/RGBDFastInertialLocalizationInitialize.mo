within SLAM.Localization;
// Time-zero source initialization with the full original raster and selection.
// No h/IMU prediction component is instantiated; the outer owner schedules once.
model RGBDFastInertialLocalizationInitialize
  import InitializeFastRGBDLocalization = SLAM.Localization.InitializeFastRGBDLocalization;
  import RGBDInertialLocalizationInterface = SLAM.Localization.RGBDInertialLocalizationInterface;

  extends RGBDInertialLocalizationInterface;
  parameter Real selectedFeatureLimit = 350.0;
  parameter Real absoluteThreshold = 18.0;
  input Real imageTime = 0.0;
  input Real initializationRequested = 1.0;
  output Real initializationAccepted;
  output Real initializationRejected;
  output Real selectionValid;
algorithm
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
    initializationAccepted,initializationRejected,selectionValid) := InitializeFastRGBDLocalization(
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
    density,imageTime,initializationRequested,
    selectedFeatureLimit,absoluteThreshold,depthUnits=depthUnits);
end RGBDFastInertialLocalizationInitialize;
