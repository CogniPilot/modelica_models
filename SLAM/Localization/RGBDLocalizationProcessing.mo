within SLAM.Localization;
// Ordinary acquisition publication retains the authoritative corrected pose view.
// This owner never reconstructs a post-correction state from raw captures.
package RGBDLocalizationProcessing
  import RGBDGraphCaptureLedger = SLAM.PoseGraph.RGBDGraphCaptureLedger;
  import RGBDGraphEstimatorCommit = SLAM.PoseGraph.RGBDGraphEstimatorCommit;
  import RGBDGraphProcessing = SLAM.PoseGraph.RGBDGraphProcessing;
  import RGBDKeyframes = SLAM.LoopClosure.RGBDKeyframes;
  import RGBDLocalizationCatalog = SLAM.Localization.RGBDLocalizationCatalog;
  import RGBDVisualVocabulary = SLAM.LoopClosure.RGBDVisualVocabulary;
  import SLAMExactRealEqual = SLAM.Inertial.SLAMExactRealEqual;

  constant Integer capacity = RGBDKeyframes.keyframeCapacity;
  constant Integer dimension = RGBDKeyframes.dimension;
  constant Integer identifierLimit = RGBDKeyframes.identifierLimit;
  record Result
    RGBDGraphProcessing.State next;
    RGBDLocalizationCatalog.Result publication;
    Boolean accepted;
    Integer reason;
    Integer ledgerReason;
    Integer vocabularyReason;
  end Result;
  function Publish
    input RGBDGraphProcessing.State previous;
    input RGBDLocalizationCatalog.Estimator proposed "Same compiled localization invocation; no host reconstruction";
    input RGBDKeyframes.Frame measurement;
    input Real producerAccepted "Prediction accepted, or explicit initialization accepted";
    input Real observationAccepted;
    input Real captureAccepted;
    input Boolean imageOn;
    input Integer imageEpoch;
    input Real imageTime;
    input Real h;
    input Boolean initializing;
    input Boolean frameAccepted;
    input Integer frameRejectionReason;
    input Boolean requested;
    input Real candidatePoint[RGBDKeyframes.featureCapacity,dimension];
    input Real candidateEnabled[RGBDKeyframes.featureCapacity];
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

    input Integer minimumMeasuredDescriptors = 8;
    input Real minimumWordDistanceSquared = 0.04;

    output Result result;
  protected
    RGBDGraphProcessing.State candidate;
    RGBDLocalizationCatalog.Result published;
    RGBDGraphCaptureLedger.State ledger;
    RGBDVisualVocabulary.State dictionary;
    Real descriptorEnabled[RGBDKeyframes.featureCapacity];
    Boolean learnAccepted;
    Boolean learnRequested;
    Integer mappingFrameReason;
    Boolean ledgerAccepted;
    Boolean valid;
    Boolean captured;
    Integer slot;
  algorithm
    result.next := previous;
    result.accepted := false;
    result.reason := 1;
    result.ledgerReason := 1;
    result.vocabularyReason := 0;
    dictionary := previous.vocabulary;
    descriptorEnabled := zeros(RGBDKeyframes.featureCapacity);
    learnAccepted := false;
    learnRequested := false;
    mappingFrameReason := frameRejectionReason;
    // Disabled call initializes only diagnostics and its held next; payload is opaque.
    result.publication := RGBDLocalizationCatalog.Publish(previous.estimator.localization,proposed,measurement,
      producerAccepted,observationAccepted,captureAccepted,imageOn,imageEpoch,imageTime,h,initializing,
      frameAccepted,frameRejectionReason,false,candidatePoint,candidateEnabled,dictionary.words,dictionary.enabled);
    if requested then
      result.reason := 2;
      valid := RGBDGraphProcessing.Valid(previous);
      if valid then
        // Frozen dictionaries are opaque to learning, including their padding.
        if not dictionary.ready then
          learnRequested := imageOn and frameAccepted and SLAMExactRealEqual(producerAccepted,1.0)
            and ((SLAMExactRealEqual(observationAccepted,1.0) and SLAMExactRealEqual(captureAccepted,0.0))
              or (SLAMExactRealEqual(observationAccepted,0.0) and SLAMExactRealEqual(captureAccepted,1.0)))
            and (not initializing or SLAMExactRealEqual(observationAccepted,0.0))
            and RGBDLocalizationCatalog.CanAdvance(previous.estimator.localization,imageTime,h,initializing,true)
            and RGBDLocalizationCatalog.ImageFresh(previous.estimator.localization,imageEpoch,imageTime)
            and RGBDLocalizationCatalog.ValidEstimator(proposed)
            and RGBDLocalizationCatalog.FrameBound(previous.estimator.localization,proposed,measurement,
              imageEpoch,imageTime,observationAccepted,captureAccepted,frameAccepted);
          if learnRequested then
            for feature in 1:RGBDKeyframes.featureCapacity loop
              descriptorEnabled[feature] := if measurement.enabled[feature] then 1.0 else 0.0;
            end for;
            (dictionary,learnAccepted,result.vocabularyReason) := RGBDVisualVocabulary.Learn(previous.vocabulary,
              measurement.descriptor,descriptorEnabled,measurement.count,previous.estimator.localization.generation,
              previous.estimator.localization.sourceRevision,measurement.vocabularyVersion,true,
              minimumMeasuredDescriptors,minimumWordDistanceSquared);
          end if;
        else
          result.vocabularyReason := 3;
        end if;
        if frameAccepted and not dictionary.ready then
          mappingFrameReason := if RGBDLocalizationCatalog.FrameBound(previous.estimator.localization,proposed,measurement,
            imageEpoch,imageTime,observationAccepted,captureAccepted,true) then 9 else 8;
        end if;
        result.reason := 3;
        published := RGBDLocalizationCatalog.Publish(previous.estimator.localization,proposed,measurement,
          producerAccepted,observationAccepted,captureAccepted,imageOn,imageEpoch,imageTime,h,initializing,
          frameAccepted and dictionary.ready,mappingFrameReason,true,candidatePoint,candidateEnabled,dictionary.words,dictionary.enabled,
          minimumInterval=minimumInterval,maximumInterval=maximumInterval,
          translationThreshold=translationThreshold,rotationThreshold=rotationThreshold,minimumFeatures=minimumFeatures,
          voxelWidth=voxelWidth,mergeRadius=mergeRadius,maximumDistance=maximumDistance,
          tentativeLifetime=tentativeLifetime,confirmedLifetime=confirmedLifetime,
          confirmationObservations=confirmationObservations,maximumConfidence=maximumConfidence,maximumTentative=maximumTentative,
          posePositions=previous.estimator.poses.positions,poseRotations=previous.estimator.poses.rotations,
          poseRevision=previous.estimator.poses.revision);
        result.publication := published;
        if published.accepted then
          result.reason := 4;
          (ledger,ledgerAccepted,result.ledgerReason) := RGBDGraphCaptureLedger.Advance(previous.captures,
            previous.estimator.localization.catalog,published.next.catalog,published.next.sourceRevision,published.next.steps,true,true);
          if ledgerAccepted then
            candidate := previous;
            candidate.estimator.localization := published.next;
            candidate.captures := ledger;
            candidate.vocabulary := dictionary;
            result.reason := 5;
            captured := published.next.catalog.nextId == previous.estimator.localization.catalog.nextId+1;
            valid := published.next.catalog.nextId == previous.estimator.localization.catalog.nextId or captured;
            if captured then
              valid := valid and previous.estimator.poses.revision < identifierLimit-1;
              if valid then
                slot := previous.estimator.localization.catalog.nextSlot;
                candidate.estimator.poses.catalogNextId := published.next.catalog.nextId;
                candidate.estimator.poses.revision := previous.estimator.poses.revision+1;
                candidate.estimator.poses.enabled[slot] := published.next.catalog.occupied[slot];
                candidate.estimator.poses.ids[slot] := published.next.catalog.ids[slot];
                candidate.estimator.poses.positions[slot,:] := published.next.catalog.bodyPositions[slot,:];
                candidate.estimator.poses.rotations[slot,:,:] := published.next.catalog.bodyRotations[slot,:,:];
              end if;
            end if;
            if valid then
              valid := RGBDGraphEstimatorCommit.ValidView(candidate.estimator.localization.catalog,
                candidate.estimator.poses,candidate.estimator.localization.sourceRevision);
            end if;
            if valid then
              result.reason := 6;
              valid := RGBDGraphProcessing.Valid(candidate);
              if valid then
                result.next := candidate;
                result.accepted := true;
                result.reason := 0;
              end if;
            end if;
          end if;
        end if;
        if not result.accepted then
          // No uncommitted inner next may escape as an authoritative receipt.
          if published.accepted then
            result.publication.rejectionReason := 2;
          end if;
          result.publication.next := previous.estimator.localization;
          result.publication.accepted := false;
          result.publication.imageCompleted := false;
          result.publication.mappingAccepted := false;
        end if;
      end if;
    end if;
  end Publish;
end RGBDLocalizationProcessing;
