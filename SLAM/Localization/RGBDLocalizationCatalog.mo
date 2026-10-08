within SLAM.Localization;
// One source-owned publication of localization and optional catalog/map work.
// The enclosing model binds proposed to the actual localization outputs once.
// This is not graph correction, independent graph noise, or a complete SLAM preset.
package RGBDLocalizationCatalog
  import ES15HeldIntervalValid = SLAM.Inertial.ES15HeldIntervalValid;
  import RGBDCatalogFrame = SLAM.Mapping.RGBDCatalogFrame;
  import RGBDCatalogMapping = SLAM.Mapping.RGBDCatalogMapping;
  import RGBDGraphMeasurements = SLAM.PoseGraph.RGBDGraphMeasurements;
  import RGBDKeyframePolicy = SLAM.LoopClosure.RGBDKeyframePolicy;
  import RGBDKeyframes = SLAM.LoopClosure.RGBDKeyframes;
  import RGBDLocalizationFrame = SLAM.Localization.RGBDLocalizationFrame;
  import RGBDMapAnchors = SLAM.Mapping.RGBDMapAnchors;
  import SLAMCovariancePSDCheck = SLAM.Inertial.SLAMCovariancePSDCheck;
  import SLAMExactRealEqual = SLAM.Inertial.SLAMExactRealEqual;

  constant Integer dimension = RGBDKeyframes.dimension;
  constant Integer currentDimension = RGBDLocalizationFrame.currentDimension;
  constant Integer referenceDimension = RGBDKeyframes.poseDimension;
  constant Integer featureCapacity = RGBDKeyframes.featureCapacity;
  constant Integer descriptorSize = RGBDKeyframes.descriptorSize;
  constant Integer identifierLimit = RGBDKeyframes.identifierLimit;

  record Estimator
    Real position[dimension]; Real velocity[dimension]; Real rotation[dimension,dimension];
    Real accelBias[dimension]; Real gyroBias[dimension];
    Real covariance[currentDimension,currentDimension];
    Real crossCovariance[currentDimension,referenceDimension];
    Real referenceCovariance[referenceDimension,referenceDimension];
    Real referencePosition[dimension]; Real referenceRotation[dimension,dimension];
    Real referenceAvailable; Real referenceEpoch; Real referenceUsed; Real lastUsedEpoch;
    Real referenceDescriptor[featureCapacity,descriptorSize];
    Real referencePoint[featureCapacity,dimension]; Real referenceEnabled[featureCapacity];
    Real referencePixels[featureCapacity,2]; Real referenceCount;
    Real referenceRgbCalibration[4]; Real referenceDepthCalibration[4];
    Real referenceNoiseReferenceFx; Real referenceDisparityNoise; Real referenceBaseline;
    Real referenceOpticalToBody[dimension,dimension]; Real referenceCameraOriginBody[dimension];
  end Estimator;

  record ReferenceBirth
    Integer generation; Integer epoch;
    Integer sequence "Accepted processing-step identity at reference birth, not a replacement counter";
    Integer catalogId "0 means this local reference has no catalog capture binding";
  end ReferenceBirth;

  record State
    Estimator estimator;
    RGBDKeyframes.Catalog catalog;
    RGBDGraphMeasurements.State graph;
    RGBDCatalogMapping.State map;
    Integer generation; Integer sourceRevision;
    Boolean initialized;
    Real predictionTime; Integer steps;
    Integer lastProcessedImageEpoch; Real lastProcessedImageTime;
    ReferenceBirth referenceBirth;
  end State;

  record Result
    State next;
    Boolean accepted; Integer rejectionReason "0 accepted; 1 idle; 2 binding; 3 chronology; 4 producer; 5 estimator; 6 reference lifecycle";
    Boolean imageCompleted; Boolean mappingAccepted;
    Integer frameRejectionReason;
    RGBDKeyframePolicy.Decision decision;
    RGBDCatalogMapping.Diagnostics mapDiagnostics;
    RGBDGraphMeasurements.Problem problem;
  end Result;

  function EmptyEstimator
    input Real position[dimension] = zeros(dimension);
    input Real velocity[dimension] = zeros(dimension);
    input Real rotation[dimension,dimension] = identity(dimension);
    input Real accelBias[dimension] = zeros(dimension);
    input Real gyroBias[dimension] = zeros(dimension);
    input Real covariance[currentDimension,currentDimension] = diagonal({0.25,0.25,0.25,
      0.04,0.04,0.04,0.01,0.01,0.01,0.0004,0.0004,0.0004,0.000025,0.000025,0.000025});
    output Estimator result;
  algorithm
    result.position := position; result.velocity := velocity; result.rotation := rotation;
    result.accelBias := accelBias; result.gyroBias := gyroBias; result.covariance := covariance;
    result.crossCovariance := zeros(currentDimension,referenceDimension);
    result.referenceCovariance := zeros(referenceDimension,referenceDimension);
    result.referencePosition := zeros(dimension); result.referenceRotation := identity(dimension);
    result.referenceAvailable := 0.0; result.referenceEpoch := 0.0;
    result.referenceUsed := 0.0; result.lastUsedEpoch := -1.0;
    result.referenceDescriptor := zeros(featureCapacity,descriptorSize);
    result.referencePoint := zeros(featureCapacity,dimension); result.referenceEnabled := zeros(featureCapacity);
    result.referencePixels := zeros(featureCapacity,2); result.referenceCount := 0.0;
    result.referenceRgbCalibration := {1.0,1.0,0.0,0.0};
    result.referenceDepthCalibration := {1.0,1.0,0.0,0.0};
    result.referenceNoiseReferenceFx := 1.0; result.referenceDisparityNoise := 0.08;
    result.referenceBaseline := 0.05; result.referenceOpticalToBody := identity(dimension);
    result.referenceCameraOriginBody := zeros(dimension);
  end EmptyEstimator;

  function Empty
    input Estimator estimator;
    input Integer generation = 1;
    input Integer sourceRevision = 1;
    input Integer vocabularyVersion = 1;
    input Real worldFrame = 0.0;
    output State result;
  algorithm
    result.estimator := estimator;
    result.catalog := RGBDKeyframes.Empty(generation,vocabularyVersion);
    result.graph := RGBDGraphMeasurements.Empty(generation);
    result.map := RGBDCatalogMapping.Empty(generation,worldFrame);
    result.generation := generation; result.sourceRevision := sourceRevision;
    result.initialized := false; result.predictionTime := 0.0; result.steps := 0;
    result.lastProcessedImageEpoch := -1; result.lastProcessedImageTime := 0.0;
    result.referenceBirth.generation := generation; result.referenceBirth.epoch := -1;
    result.referenceBirth.sequence := 0; result.referenceBirth.catalogId := 0;
  end Empty;

  function ValidEstimator
    input Estimator value;
    output Boolean valid;
  protected
    Real joint[currentDimension+referenceDimension,currentDimension+referenceDimension];
  algorithm
    valid := RGBDMapAnchors.ProperRotation(value.rotation)
      and SLAMCovariancePSDCheck(value.covariance,1e-12) == 1.0
      and (SLAMExactRealEqual(value.referenceAvailable,0.0) or SLAMExactRealEqual(value.referenceAvailable,1.0))
      and (SLAMExactRealEqual(value.referenceUsed,0.0) or SLAMExactRealEqual(value.referenceUsed,1.0))
      and value.lastUsedEpoch >= -1.0 and value.lastUsedEpoch <= identifierLimit
      and SLAMExactRealEqual(value.lastUsedEpoch,floor(value.lastUsedEpoch));
    for axis in 1:dimension loop
      valid := valid and abs(value.position[axis]) <= 1e6 and abs(value.velocity[axis]) <= 1e6
        and abs(value.accelBias[axis]) <= 2.0 and abs(value.gyroBias[axis]) <= 0.3;
    end for;
    // Unavailable reference buffers stay opaque, matching the propagation owner.
    if SLAMExactRealEqual(value.referenceAvailable,0.0) then
      valid := valid and SLAMExactRealEqual(value.referenceUsed,0.0);
    elseif SLAMExactRealEqual(value.referenceAvailable,1.0) then
      valid := valid and RGBDMapAnchors.ProperRotation(value.referenceRotation)
        and value.referenceEpoch >= 0.0 and value.referenceEpoch <= identifierLimit
        and SLAMExactRealEqual(value.referenceEpoch,floor(value.referenceEpoch))
        and ((SLAMExactRealEqual(value.referenceUsed,0.0) and value.referenceEpoch > value.lastUsedEpoch)
          or (SLAMExactRealEqual(value.referenceUsed,1.0) and value.referenceEpoch <= value.lastUsedEpoch));
      for axis in 1:dimension loop
        valid := valid and abs(value.referencePosition[axis]) <= 1e6;
      end for;
      joint := cat(1,cat(2,value.covariance,value.crossCovariance),
        cat(2,transpose(value.crossCovariance),value.referenceCovariance));
      valid := valid and SLAMCovariancePSDCheck(joint,1e-12) == 1.0;
    end if;
  end ValidEstimator;

  function ValidHeader
    input State state;
    output Boolean valid;
  algorithm
    valid := state.generation >= 1 and state.generation <= identifierLimit
      and state.sourceRevision >= 1 and state.sourceRevision <= identifierLimit
      and state.steps >= 0 and state.steps < identifierLimit
      and state.predictionTime >= 0.0 and state.predictionTime <= 1e9
      and state.lastProcessedImageEpoch >= -1 and state.lastProcessedImageEpoch <= identifierLimit
      and state.lastProcessedImageTime >= 0.0 and state.lastProcessedImageTime <= state.predictionTime
      and RGBDKeyframes.ValidHeader(state.catalog)
      and state.catalog.generation == state.generation and state.graph.generation == state.generation
      and state.map.generation == state.generation
      and state.graph.revision >= 0 and state.graph.revision < identifierLimit
      and state.graph.lastCaptureId == state.catalog.nextId-1
      and state.graph.nextEdgeId >= 1 and state.graph.nextEdgeId <= identifierLimit
      and state.map.catalogRevision >= 0 and state.map.catalogRevision <= state.graph.revision
      and state.catalog.lastEpoch <= state.lastProcessedImageEpoch
      and state.map.imageEpoch >= -1 and state.map.imageEpoch <= state.lastProcessedImageEpoch
      and state.catalog.lastTime <= state.predictionTime and state.map.imageTime >= 0.0 and state.map.imageTime <= state.predictionTime
      and state.map.frame >= 0.0 and state.map.frame < identifierLimit
      and SLAMExactRealEqual(state.map.frame,floor(state.map.frame))
      and abs(state.map.worldFrame) <= 1e9
      and state.referenceBirth.generation == state.generation;
    if state.initialized then
      valid := valid and state.steps >= 1;
    else
      valid := valid and state.steps == 0 and SLAMExactRealEqual(state.predictionTime,0.0)
        and state.lastProcessedImageEpoch == -1 and state.catalog.nextId == 1
        and state.graph.revision == 0 and SLAMExactRealEqual(state.map.frame,0.0)
        and SLAMExactRealEqual(state.estimator.referenceAvailable,0.0)
        and SLAMExactRealEqual(state.estimator.lastUsedEpoch,-1.0);
    end if;
    if SLAMExactRealEqual(state.estimator.referenceAvailable,1.0) then
      valid := valid and state.referenceBirth.sequence >= 1 and state.referenceBirth.sequence <= state.steps
        and state.referenceBirth.epoch >= 0 and state.referenceBirth.epoch <= state.lastProcessedImageEpoch
        and SLAMExactRealEqual(state.estimator.referenceEpoch,state.referenceBirth.epoch)
        and state.referenceBirth.catalogId >= 0 and state.referenceBirth.catalogId < state.catalog.nextId;
    else
      valid := valid and state.referenceBirth.sequence == 0 and state.referenceBirth.epoch == -1
        and state.referenceBirth.catalogId == 0;
    end if;
  end ValidHeader;

  function FrameBound
    input State previous; input Estimator proposed; input RGBDKeyframes.Frame measurement;
    input Integer imageEpoch; input Real imageTime;
    input Real observationAccepted; input Real captureAccepted; input Boolean frameAccepted;
    output Boolean frameBound;
  algorithm
    frameBound := frameAccepted and measurement.generation == previous.generation
      and measurement.id == previous.catalog.nextId and measurement.epoch == imageEpoch
      and SLAMExactRealEqual(measurement.imageTime,imageTime)
      and measurement.vocabularyVersion == previous.catalog.vocabularyVersion
      and (SLAMExactRealEqual(observationAccepted,1.0) or SLAMExactRealEqual(captureAccepted,1.0));
    if frameBound then
      for axis in 1:dimension loop
        frameBound := frameBound and SLAMExactRealEqual(measurement.bodyPosition[axis],proposed.position[axis]);
        for column in 1:dimension loop
          frameBound := frameBound and SLAMExactRealEqual(measurement.bodyRotation[axis,column],proposed.rotation[axis,column]);
        end for;
      end for;
      for row in 1:referenceDimension loop
        for column in 1:referenceDimension loop
          frameBound := frameBound and SLAMExactRealEqual(measurement.poseCovariance[row,column],
            proposed.covariance[RGBDLocalizationFrame.poseIndices[row],RGBDLocalizationFrame.poseIndices[column]]);
        end for;
      end for;
    end if;
  end FrameBound;

  function ImageFresh
    input State previous; input Integer imageEpoch; input Real imageTime;
    output Boolean fresh;
  algorithm
    fresh := imageEpoch >= 0 and imageEpoch <= identifierLimit
      and imageEpoch > previous.lastProcessedImageEpoch
      and imageEpoch > previous.catalog.lastEpoch and imageEpoch > previous.map.imageEpoch
      and imageTime >= 0.0 and imageTime <= 1e9
      and (previous.lastProcessedImageEpoch == -1 or imageTime > previous.lastProcessedImageTime);
  end ImageFresh;

  function CanAdvance
    input State previous; input Real intervalTime; input Real h;
    input Boolean initializing; input Boolean requested;
    output Boolean allowed;
  algorithm
    allowed := false;
    if requested then
      allowed := ValidHeader(previous) and intervalTime >= 0.0 and intervalTime <= 1e9;
      if initializing then
        allowed := allowed and not previous.initialized
          and SLAMExactRealEqual(intervalTime,0.0) and SLAMExactRealEqual(h,0.0);
      else
        allowed := allowed and previous.initialized and ES15HeldIntervalValid(h)
          and intervalTime > previous.predictionTime
          and abs(intervalTime-previous.predictionTime-h) <= 1e-12*max(1.0,abs(intervalTime));
      end if;
    end if;
  end CanAdvance;

  function ReferenceHeld
    input Estimator previous; input Estimator proposed;
    output Boolean held;
  algorithm
    held := SLAMExactRealEqual(previous.referenceAvailable,proposed.referenceAvailable)
      and SLAMExactRealEqual(previous.referenceEpoch,proposed.referenceEpoch);
    if SLAMExactRealEqual(previous.referenceAvailable,1.0) then
      held := held and SLAMExactRealEqual(previous.referenceCount,proposed.referenceCount)
        and SLAMExactRealEqual(previous.referenceNoiseReferenceFx,proposed.referenceNoiseReferenceFx)
        and SLAMExactRealEqual(previous.referenceDisparityNoise,proposed.referenceDisparityNoise)
        and SLAMExactRealEqual(previous.referenceBaseline,proposed.referenceBaseline);
      for axis in 1:dimension loop
        held := held and SLAMExactRealEqual(previous.referencePosition[axis],proposed.referencePosition[axis])
          and SLAMExactRealEqual(previous.referenceCameraOriginBody[axis],proposed.referenceCameraOriginBody[axis]);
        for column in 1:dimension loop
          held := held and SLAMExactRealEqual(previous.referenceRotation[axis,column],proposed.referenceRotation[axis,column])
            and SLAMExactRealEqual(previous.referenceOpticalToBody[axis,column],proposed.referenceOpticalToBody[axis,column]);
        end for;
      end for;
      for coordinate in 1:4 loop
        held := held and SLAMExactRealEqual(previous.referenceRgbCalibration[coordinate],proposed.referenceRgbCalibration[coordinate])
          and SLAMExactRealEqual(previous.referenceDepthCalibration[coordinate],proposed.referenceDepthCalibration[coordinate]);
      end for;
      for row in 1:referenceDimension loop
        for column in 1:referenceDimension loop
          held := held and SLAMExactRealEqual(previous.referenceCovariance[row,column],proposed.referenceCovariance[row,column]);
        end for;
      end for;
      for feature in 1:featureCapacity loop
        held := held and SLAMExactRealEqual(previous.referenceEnabled[feature],proposed.referenceEnabled[feature]);
        if SLAMExactRealEqual(previous.referenceEnabled[feature],1.0) then
          for axis in 1:dimension loop
            held := held and SLAMExactRealEqual(previous.referencePoint[feature,axis],proposed.referencePoint[feature,axis]);
          end for;
          for coordinate in 1:2 loop
            held := held and SLAMExactRealEqual(previous.referencePixels[feature,coordinate],proposed.referencePixels[feature,coordinate]);
          end for;
          for coordinate in 1:descriptorSize loop
            held := held and SLAMExactRealEqual(previous.referenceDescriptor[feature,coordinate],proposed.referenceDescriptor[feature,coordinate]);
          end for;
        end if;
      end for;
    end if;
  end ReferenceHeld;

  function Publish
    input State previous;
    input Estimator proposed "Same compiled localization invocation; no host reconstruction";
    input RGBDKeyframes.Frame measurement;
    input Real producerAccepted "Prediction accepted, or explicit initialization accepted";
    input Real observationAccepted; input Real captureAccepted;
    input Boolean imageOn; input Integer imageEpoch; input Real imageTime; input Real h;
    input Boolean initializing;
    input Boolean frameAccepted; input Integer frameRejectionReason;
    input Boolean requested;
    input Real candidatePoint[featureCapacity,dimension]; input Real candidateEnabled[featureCapacity];
    input Real vocabulary[RGBDKeyframes.wordCapacity,descriptorSize];
    input Real vocabularyEnabled[RGBDKeyframes.wordCapacity];
    input Real minimumInterval = 0.5; input Real maximumInterval = 2.0;
    input Real translationThreshold = 0.6; input Real rotationThreshold = 0.25;
    input Integer minimumFeatures = 12;
    input Real voxelWidth = 0.25; input Real mergeRadius = 0.15;
    input Real maximumDistance = 80.0; input Real tentativeLifetime = 0.5;
    input Real confirmedLifetime = 5.0; input Real confirmationObservations = 3.0;
    input Real maximumConfidence = 8.0; input Real maximumTentative = 700.0;
    input Real posePositions[RGBDKeyframes.keyframeCapacity,dimension] = previous.catalog.bodyPositions;
    input Real poseRotations[RGBDKeyframes.keyframeCapacity,dimension,dimension] = previous.catalog.bodyRotations;
    input Integer poseRevision = previous.map.catalogRevision;
    output Result result;
  protected
    Boolean valid; Boolean frameBound;
    RGBDCatalogMapping.Result mapped;
  algorithm
    result.next := previous; result.accepted := false; result.rejectionReason := 1;
    result.imageCompleted := false; result.mappingAccepted := false;
    result.frameRejectionReason := frameRejectionReason;
    result.decision.valid := false; result.decision.captureRequested := false; result.decision.reason := 1;
    result.decision.enabledCount := 0; result.decision.elapsed := 0.0;
    result.decision.translationSquared := 0.0; result.decision.rotationCosine := 1.0;
    result.mapDiagnostics := RGBDCatalogMapping.EmptyDiagnostics();
    result.problem := RGBDGraphMeasurements.EmptyProblem();
    if requested then
      result.rejectionReason := 2;
      valid := ValidHeader(previous) and ValidEstimator(previous.estimator);
      if valid then
        result.rejectionReason := 3;
        valid := imageTime >= 0.0 and imageTime <= 1e9;
        if initializing then
          valid := valid and not previous.initialized and imageOn
            and SLAMExactRealEqual(imageTime,0.0) and SLAMExactRealEqual(h,0.0);
        else
          valid := valid and previous.initialized and ES15HeldIntervalValid(h)
            and imageTime > previous.predictionTime
            and abs(imageTime-previous.predictionTime-h) <= 1e-12*max(1.0,abs(imageTime));
        end if;
        if imageOn then valid := valid and ImageFresh(previous,imageEpoch,imageTime); end if;
        if valid then
          result.rejectionReason := 4;
          valid := SLAMExactRealEqual(producerAccepted,1.0)
            and (SLAMExactRealEqual(observationAccepted,0.0) or SLAMExactRealEqual(observationAccepted,1.0))
            and (SLAMExactRealEqual(captureAccepted,0.0) or SLAMExactRealEqual(captureAccepted,1.0))
            and (imageOn or (SLAMExactRealEqual(observationAccepted,0.0) and SLAMExactRealEqual(captureAccepted,0.0)))
            and (not initializing or SLAMExactRealEqual(observationAccepted,0.0));
          if valid then
            result.rejectionReason := 5; valid := ValidEstimator(proposed);
            if valid then
              result.rejectionReason := 6;
              valid := proposed.lastUsedEpoch >= previous.estimator.lastUsedEpoch
                and (not imageOn or proposed.lastUsedEpoch <= imageEpoch);
              if proposed.lastUsedEpoch > previous.estimator.lastUsedEpoch then
                valid := valid and imageOn and SLAMExactRealEqual(proposed.lastUsedEpoch,imageEpoch)
                  and SLAMExactRealEqual(proposed.referenceUsed,1.0);
              end if;
              if SLAMExactRealEqual(observationAccepted,1.0) then
                valid := valid and SLAMExactRealEqual(proposed.lastUsedEpoch,imageEpoch)
                  and SLAMExactRealEqual(proposed.referenceUsed,1.0)
                  and SLAMExactRealEqual(captureAccepted,0.0);
              end if;
              if SLAMExactRealEqual(captureAccepted,1.0) then
                valid := valid and previous.referenceBirth.sequence < identifierLimit
                  and SLAMExactRealEqual(proposed.referenceAvailable,1.0)
                  and SLAMExactRealEqual(proposed.referenceEpoch,imageEpoch)
                  and SLAMExactRealEqual(proposed.referenceUsed,0.0)
                  and imageEpoch > previous.estimator.lastUsedEpoch;
                for axis in 1:dimension loop
                  valid := valid and SLAMExactRealEqual(proposed.referencePosition[axis],proposed.position[axis]);
                  for column in 1:dimension loop
                    valid := valid and SLAMExactRealEqual(proposed.referenceRotation[axis,column],proposed.rotation[axis,column]);
                  end for;
                end for;
                for row in 1:currentDimension loop
                  for column in 1:referenceDimension loop
                    valid := valid and SLAMExactRealEqual(proposed.crossCovariance[row,column],
                      proposed.covariance[row,RGBDLocalizationFrame.poseIndices[column]]);
                  end for;
                end for;
                for row in 1:referenceDimension loop
                  for column in 1:referenceDimension loop
                    valid := valid and SLAMExactRealEqual(proposed.referenceCovariance[row,column],
                      proposed.covariance[RGBDLocalizationFrame.poseIndices[row],RGBDLocalizationFrame.poseIndices[column]]);
                  end for;
                end for;
              else
                valid := valid and ReferenceHeld(previous.estimator,proposed)
                  and proposed.referenceUsed >= previous.estimator.referenceUsed;
              end if;
              if not imageOn then
                valid := valid and SLAMExactRealEqual(proposed.lastUsedEpoch,previous.estimator.lastUsedEpoch)
                  and SLAMExactRealEqual(proposed.referenceUsed,previous.estimator.referenceUsed);
              end if;
              if valid then
                // Completed localization and its raw-noise ledger commit even
                // when optional Frame/BoW/registration/map admission refuses.
                result.next.estimator := proposed; result.next.initialized := true;
                if SLAMExactRealEqual(captureAccepted,0.0) then
                  // Retained image payload is immutable, including unavailable
                  // and disabled cells. Copy its owner rather than comparing NaN
                  // payload with numerical equality or inventing canonical data.
                  result.next.estimator.referenceDescriptor := previous.estimator.referenceDescriptor;
                  result.next.estimator.referencePoint := previous.estimator.referencePoint;
                  result.next.estimator.referenceEnabled := previous.estimator.referenceEnabled;
                  result.next.estimator.referencePixels := previous.estimator.referencePixels;
                  result.next.estimator.referenceCount := previous.estimator.referenceCount;
                  result.next.estimator.referenceRgbCalibration := previous.estimator.referenceRgbCalibration;
                  result.next.estimator.referenceDepthCalibration := previous.estimator.referenceDepthCalibration;
                  result.next.estimator.referenceNoiseReferenceFx := previous.estimator.referenceNoiseReferenceFx;
                  result.next.estimator.referenceDisparityNoise := previous.estimator.referenceDisparityNoise;
                  result.next.estimator.referenceBaseline := previous.estimator.referenceBaseline;
                  result.next.estimator.referenceOpticalToBody := previous.estimator.referenceOpticalToBody;
                  result.next.estimator.referenceCameraOriginBody := previous.estimator.referenceCameraOriginBody;
                  result.next.estimator.referencePosition := previous.estimator.referencePosition;
                  result.next.estimator.referenceRotation := previous.estimator.referenceRotation;
                  result.next.estimator.referenceCovariance := previous.estimator.referenceCovariance;
                  if SLAMExactRealEqual(previous.estimator.referenceAvailable,0.0) then
                    result.next.estimator.crossCovariance := previous.estimator.crossCovariance;
                  end if;
                end if;
                result.next.predictionTime := imageTime; result.next.steps := previous.steps+1;
                result.accepted := true; result.rejectionReason := 0;
                if SLAMExactRealEqual(captureAccepted,1.0) then
                  result.next.referenceBirth.generation := previous.generation;
                  result.next.referenceBirth.epoch := imageEpoch;
                  result.next.referenceBirth.sequence := previous.steps+1;
                  result.next.referenceBirth.catalogId := 0;
                end if;
                if imageOn then
                  result.imageCompleted := true;
                  result.next.lastProcessedImageEpoch := imageEpoch;
                  result.next.lastProcessedImageTime := imageTime;
                  // Check binding in addition to the builder's own certificate.
                  // Pose/covariance are from the same accepted localization.
                  frameBound := FrameBound(previous,proposed,measurement,imageEpoch,imageTime,
                    observationAccepted,captureAccepted,frameAccepted);
                  if frameAccepted and not frameBound then result.frameRejectionReason := 8; end if;
                  if frameBound then
                    (mapped,result.decision) := RGBDCatalogFrame.Advance(previous.catalog,previous.graph,previous.map,
                      measurement,vocabulary,vocabularyEnabled,candidatePoint,candidateEnabled,true,1.0,true,
                      minimumInterval=minimumInterval,maximumInterval=maximumInterval,
                      translationThreshold=translationThreshold,rotationThreshold=rotationThreshold,minimumFeatures=minimumFeatures,
                      voxelWidth=voxelWidth,mergeRadius=mergeRadius,maximumDistance=maximumDistance,
                      tentativeLifetime=tentativeLifetime,confirmedLifetime=confirmedLifetime,
                      confirmationObservations=confirmationObservations,maximumConfidence=maximumConfidence,maximumTentative=maximumTentative,
                      nodePosition=posePositions,nodeRotation=poseRotations,poseRevision=poseRevision);
                    result.mapDiagnostics := mapped.diagnostics; result.problem := mapped.problem;
                    if mapped.accepted then
                      result.next.catalog := mapped.catalog; result.next.graph := mapped.graph; result.next.map := mapped.map;
                      result.mappingAccepted := true;
                      if result.next.catalog.nextId == previous.catalog.nextId+1
                        and result.next.referenceBirth.epoch == imageEpoch
                        and result.next.referenceBirth.generation == previous.generation then
                        result.next.referenceBirth.catalogId := measurement.id;
                      end if;
                    end if;
                  end if;
                end if;
              end if;
            end if;
          end if;
        end if;
      end if;
    end if;
  end Publish;
end RGBDLocalizationCatalog;
