within SLAM.Localization;
// Time-zero initialization is a separate source transaction, not h=0 prediction.
// The outer owner schedules it once and publishes the complete returned tuple.
model RGBDInertialLocalizationInitialize
  import InitializeRGBDLocalization = SLAM.Localization.InitializeRGBDLocalization;
  import RGBDInertialLocalizationInterface = SLAM.Localization.RGBDInertialLocalizationInterface;

  extends RGBDInertialLocalizationInterface;
  input Real pixels[featureCapacity,2];
  input Real activeCount;
  input Real featureScore[featureCapacity] = zeros(featureCapacity);
  input Real imageTime = 0.0;
  input Real initializationRequested = 1.0;
  output Real initializationAccepted;
  output Real initializationRejected;
protected
  parameter Real minimumContrast = 1e-6;
  parameter Real nearDepth = 0.28;
  parameter Real farDepth = 10.0;
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
    density,pixels,activeCount,
    featureScore,imageTime,initializationRequested,
    minimumContrast,nearDepth,farDepth,depthUnits=depthUnits);
end RGBDInertialLocalizationInitialize;
