within SLAM;
// Public lifecycle adapter; Rumoca compiles the complete Modelica implementation.
model RGBDFastSLAMStep
  import AdvanceFastSLAM = SLAM.AdvanceFastSLAM;
  import RGBDFastSLAMInterface = SLAM.RGBDFastSLAMInterface;

  extends RGBDFastSLAMInterface;
equation
  (next,accepted,imageCompleted,mappingAccepted,
    graphCorrectionAccepted,roundoffCertified,publicationReason,ledgerReason,
    vocabularyReason,graphReason,graphCommitReason,graphFilterReason,
    covarianceStatus,graphCostBefore,graphCostAfter,nextQuaternion,
    selectionValid,predictionAccepted,initializationAccepted,observationAccepted,
    captureAccepted,matchCount,features,featureEnabled,
    trackingCurrentPixel,trackingReferencePixel,trackingEnabled,
    keyframeDecisionReason,keyframeRejectionReason,sequentialRejectionReason,frameRejectionDiagnostic,
    loopAttempted,loopRetrievalReason,loopGraphReason,loopCandidateCount,loopCandidateReferenceId,loopCandidateCurrentId,loopCandidateScore,loopCandidateReason,loopCandidateMatchedCount,loopCandidateInliers,loopCandidateRms,loopCandidateVerified) =
    AdvanceFastSLAM(
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
      h=h,
      intervalTime=intervalTime,
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
      graphPolicy=graphPolicy,depthUnits=depthUnits,
      loopMinimumSimilarity=loopMinimumSimilarity, loopMinimumAge=loopMinimumAge, loopMinimumInliers=loopMinimumInliers, loopMinimumFraction=loopMinimumFraction, loopInlierDistance=loopInlierDistance, loopMaximumRms=loopMaximumRms, loopCalibratedResiduals=loopCalibratedResiduals, loopMaximumNormalizedSquared=loopMaximumNormalizedSquared);
end RGBDFastSLAMStep;
