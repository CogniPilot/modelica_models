within SLAM;
// Source-owned held-IMU batching for the complete raw-camera SLAM lifecycle.
model RGBDFastSLAMIntervals
  import AdvanceFastSLAMIntervals = SLAM.AdvanceFastSLAMIntervals;
  import RGBDGraphProcessing = SLAM.PoseGraph.RGBDGraphProcessing;
  import RGBDKeyframes = SLAM.LoopClosure.RGBDKeyframes;
  import RGBDNominalCalibration = Vision.Matching.RGBDNominalCalibration;

  parameter Integer imageHeight(min=1) = RGBDKeyframes.imageHeight;
  parameter Integer imageWidth(min=1) = RGBDKeyframes.imageWidth;
  parameter Boolean depthQualifiedSelection = true;
  input RGBDGraphProcessing.State previous;
  input Real rgb[imageHeight,imageWidth,channelCount];
  input Real depth[imageHeight,imageWidth];
  input Real depthUnits = 1.0 "Meters per depth sample";
  input Real rgbCalibration[4] = defaultRgbCalibration;
  input Real depthCalibration[4] = defaultDepthCalibration;
  input Real disparityNoise = 0.08;
  input Real noiseReferenceFx = 848.0/(2.0*tan(87.0*3.141592653589793/360.0));
  input Real baseline = 0.05;
  input Real opticalToBody[spaceDimension,spaceDimension] = [0.0,0.0,1.0;-1.0,0.0,0.0;0.0,-1.0,0.0];
  input Real cameraOriginBody[spaceDimension] = {0.18,0.0,-0.04};
  input Real accel[maximumIntervals,spaceDimension] = {{0.0,0.0,9.81} for interval in 1:maximumIntervals};
  input Real gyro[maximumIntervals,spaceDimension] = zeros(maximumIntervals,spaceDimension);
  input Real gravity[spaceDimension] = {0.0,0.0,-9.81};
  input Real density[noiseDimension] = {0.06,0.06,0.06,0.006,0.006,0.006,0.002,0.002,0.002,0.0002,0.0002,0.0002};
  input Real durations[maximumIntervals] = fill(1.0/90.0,maximumIntervals);
  input Real intervalTimes[maximumIntervals];
  input Integer intervalCount;
  input Real frameTime;
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
  output Integer processedIntervals "Successfully accepted calls before success or whole-batch rollback";
  output Integer failedInterval "One-based failing call; zero for preflight refusal or no failure";
  output Integer batchReason "0 success, 1 idle, 2 count, 3 chronology, 4 stale image, 5 interval refusal, 6 previous state";
protected
  constant Integer maximumIntervals = 36;
  constant Integer featureCapacity = RGBDKeyframes.featureCapacity;
  constant Integer descriptorSize = RGBDKeyframes.descriptorSize;
  final parameter Real defaultRgbCalibration[4] = RGBDNominalCalibration({imageHeight,imageWidth},{69.0,42.0});
  final parameter Real defaultDepthCalibration[4] = RGBDNominalCalibration({imageHeight,imageWidth},{87.0,58.0});
  constant Integer spaceDimension = RGBDKeyframes.dimension;
  constant Integer errorDimension = 5*spaceDimension;
  constant Integer poseDimension = 2*spaceDimension;
  constant Integer noiseDimension = 4*spaceDimension;
  parameter Integer channelCount(min=3,max=4) = 4;
  constant Integer quaternionSize = 4;
equation
  (next,accepted,imageCompleted,mappingAccepted,
    graphCorrectionAccepted,roundoffCertified,publicationReason,ledgerReason,
    vocabularyReason,graphReason,graphCommitReason,graphFilterReason,
    covarianceStatus,graphCostBefore,graphCostAfter,nextQuaternion,
    selectionValid,predictionAccepted,initializationAccepted,observationAccepted,
    captureAccepted,matchCount,features,featureEnabled,
    trackingCurrentPixel,trackingReferencePixel,trackingEnabled,processedIntervals,
    failedInterval,batchReason) =
    AdvanceFastSLAMIntervals(
      previous=previous,
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
      durations=durations,
      intervalTimes=intervalTimes,
      intervalCount=intervalCount,
      frameTime=frameTime,
      imageEpoch=imageEpoch,
      imageRequested=imageRequested,
      localCaptureRequested=localCaptureRequested,
      requested=requested,
      minimumMeasuredDescriptors=minimumMeasuredDescriptors,
      minimumWordDistanceSquared=minimumWordDistanceSquared,
      selectedFeatureLimit=selectedFeatureLimit,
      absoluteThreshold=absoluteThreshold,
                  depthQualifiedSelection=depthQualifiedSelection,
      minimumInterval=minimumInterval,
      maximumInterval=maximumInterval,
      translationThreshold=translationThreshold,
      rotationThreshold=rotationThreshold,
      minimumFeatures=minimumFeatures,
      voxelWidth=voxelWidth,
      mergeRadius=mergeRadius,
      maximumDistance=maximumDistance,
      tentativeLifetime=tentativeLifetime,
      confirmedLifetime=confirmedLifetime,
      confirmationObservations=confirmationObservations,
      maximumConfidence=maximumConfidence,
      maximumTentative=maximumTentative,
      graphCorrectionRequested=graphCorrectionRequested,
      graphPolicy=graphPolicy,depthUnits=depthUnits);
end RGBDFastSLAMIntervals;
