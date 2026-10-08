within SLAM;
// All active clocks are preflighted before evaluation. Only the last held
// interval can consume an image, capture a reference or attempt graph correction.
// A failed call rolls back the complete State, including graph attempt receipts.
function AdvanceFastSLAMIntervals
  import AdvanceFastSLAM = SLAM.AdvanceFastSLAM;
  import ES15HeldIntervalValid = SLAM.Inertial.ES15HeldIntervalValid;
  import RGBDGraphProcessing = SLAM.PoseGraph.RGBDGraphProcessing;
  import RGBDKeyframes = SLAM.LoopClosure.RGBDKeyframes;
  import RGBDLocalizationCatalog = SLAM.Localization.RGBDLocalizationCatalog;
  import RGBDNominalCalibration = Vision.Matching.RGBDNominalCalibration;
  import SLAMExactRealEqual = SLAM.Inertial.SLAMExactRealEqual;

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
  input Boolean depthQualifiedSelection = true;
  input Real depthUnits = 1.0 "Meters per depth sample; 1 for metric depth, SDK scale for Z16";
protected
  constant Integer maximumIntervals = 36;
  constant Integer featureCapacity = RGBDKeyframes.featureCapacity;
  constant Integer descriptorSize = RGBDKeyframes.descriptorSize;
  constant Integer spaceDimension = RGBDKeyframes.dimension;
  constant Integer errorDimension = 5*spaceDimension;
  constant Integer poseDimension = 2*spaceDimension;
  constant Integer noiseDimension = 4*spaceDimension;
  constant Integer channelCount = 4;
  constant Integer quaternionSize = 4;
  Boolean chronologyValid;
  Boolean continueBatch;
  Real previousEndpoint;
algorithm
  next := previous;
  accepted := false;
  imageCompleted := false;
  mappingAccepted := false;
  graphCorrectionAccepted := false;
  roundoffCertified := false;
  graphCostBefore := 0.0;
  graphCostAfter := 0.0;
  nextQuaternion := {1.0,0.0,0.0,0.0};
  selectionValid := 0.0;
  predictionAccepted := 0.0;
  initializationAccepted := 0.0;
  observationAccepted := 0.0;
  captureAccepted := 0.0;
  matchCount := 0.0;
  features := zeros(featureCapacity,spaceDimension);
  featureEnabled := zeros(featureCapacity);
  trackingCurrentPixel := zeros(featureCapacity,2);
  trackingReferencePixel := zeros(featureCapacity,2);
  trackingEnabled := zeros(featureCapacity);
  publicationReason := 1;
  ledgerReason := 1;
  vocabularyReason := 0;
  graphReason := 1;
  graphCommitReason := 0;
  graphFilterReason := 0;
  covarianceStatus := 0;
  processedIntervals := 0;
  failedInterval := 0;
  batchReason := 1;
  chronologyValid := false;
  continueBatch := false;
  previousEndpoint := 0.0;
  if requested then
    batchReason := 2;
    if intervalCount >= 1 and intervalCount <= maximumIntervals then
      batchReason := 6;
      if RGBDGraphProcessing.Valid(previous) and previous.estimator.localization.initialized then
        batchReason := 3;
        chronologyValid := frameTime >= 0.0 and frameTime <= 1e9;
        previousEndpoint := previous.estimator.localization.predictionTime;
        for interval in 1:intervalCount loop
          if chronologyValid then
            chronologyValid := ES15HeldIntervalValid(durations[interval])
              and intervalTimes[interval] >= 0.0 and intervalTimes[interval] <= 1e9
              and intervalTimes[interval] > previousEndpoint
              and abs(intervalTimes[interval]-previousEndpoint-durations[interval])
                <= 1e-12*max(1.0,abs(intervalTimes[interval]));
            if chronologyValid then
              previousEndpoint := intervalTimes[interval];
            end if;
          end if;
        end for;
        if chronologyValid then
          chronologyValid := abs(previousEndpoint-frameTime) <= 1e-12*max(1.0,abs(frameTime));
        end if;
        if chronologyValid then
          batchReason := 4;
          // Image identity is checked against the original pre-batch ledger.
          // No intermediate prediction can turn a stale image into a fresh one.
          continueBatch := if imageRequested then
            RGBDLocalizationCatalog.ImageFresh(previous.estimator.localization,imageEpoch,frameTime) else true;
          if continueBatch then
            batchReason := 0;
            for interval in 1:intervalCount loop
              if continueBatch then
                (next,accepted,imageCompleted,mappingAccepted,
                  graphCorrectionAccepted,roundoffCertified,publicationReason,ledgerReason,
                  vocabularyReason,graphReason,graphCommitReason,graphFilterReason,
                  covarianceStatus,graphCostBefore,graphCostAfter,nextQuaternion,
                  selectionValid,predictionAccepted,initializationAccepted,observationAccepted,
                  captureAccepted,matchCount,features,featureEnabled,
                  trackingCurrentPixel,trackingReferencePixel,trackingEnabled) := AdvanceFastSLAM(
                  previous=next,
                  rgb=rgb,
                  depth=depth,
                  rgbCalibration=rgbCalibration,
                  depthCalibration=depthCalibration,
                  disparityNoise=disparityNoise,
                  noiseReferenceFx=noiseReferenceFx,
                  baseline=baseline,
                  opticalToBody=opticalToBody,
                  cameraOriginBody=cameraOriginBody,
                  accel=accel[interval,:],
                  gyro=gyro[interval,:],
                  gravity=gravity,
                  density=density,
                  h=durations[interval],
                  intervalTime=intervalTimes[interval],
                  imageEpoch=imageEpoch,
                  imageRequested=imageRequested and interval == intervalCount,
                  localCaptureRequested=localCaptureRequested and interval == intervalCount,
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
                  graphCorrectionRequested=graphCorrectionRequested and interval == intervalCount,
                  graphPolicy=graphPolicy,depthUnits=depthUnits);
                if accepted and SLAMExactRealEqual(predictionAccepted,1.0) then
                  processedIntervals := processedIntervals+1;
                else
                  failedInterval := interval;
                  batchReason := 5;
                  continueBatch := false;
                end if;
              end if;
            end for;
          end if;
        end if;
      end if;
    end if;
  end if;
  if batchReason == 5 then
    // Keep only the failing call's rejection codes and batch diagnostics.
    // Neither a partial State nor an earlier image/producer/display result escapes.
    next := previous;
    accepted := false;
    imageCompleted := false;
    mappingAccepted := false;
    graphCorrectionAccepted := false;
    roundoffCertified := false;
    graphCostBefore := 0.0;
    graphCostAfter := 0.0;
    nextQuaternion := {1.0,0.0,0.0,0.0};
    selectionValid := 0.0;
    predictionAccepted := 0.0;
    initializationAccepted := 0.0;
    observationAccepted := 0.0;
    captureAccepted := 0.0;
    matchCount := 0.0;
    features := zeros(featureCapacity,spaceDimension);
    featureEnabled := zeros(featureCapacity);
    trackingCurrentPixel := zeros(featureCapacity,2);
    trackingReferencePixel := zeros(featureCapacity,2);
    trackingEnabled := zeros(featureCapacity);
  end if;
end AdvanceFastSLAMIntervals;
