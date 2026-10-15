within SLAM;
// Raw sensor data and the complete persistent Modelica session owner.
partial model RGBDFastSLAMInterface
  import RGBDGraphProcessing = SLAM.PoseGraph.RGBDGraphProcessing;
  import RGBDKeyframeRetrieval = SLAM.LoopClosure.RGBDKeyframeRetrieval;
  import RGBDKeyframes = SLAM.LoopClosure.RGBDKeyframes;
  import RGBDNominalCalibration = Vision.Matching.RGBDNominalCalibration;

  parameter Integer imageHeight(min=1) = RGBDKeyframes.imageHeight;
  parameter Integer imageWidth(min=1) = RGBDKeyframes.imageWidth;
  constant Integer featureCapacity = RGBDKeyframes.featureCapacity;
  constant Integer descriptorSize = RGBDKeyframes.descriptorSize;
  final parameter Real defaultRgbCalibration[4] = RGBDNominalCalibration({imageHeight,imageWidth},{69.0,42.0});
  final parameter Real defaultDepthCalibration[4] = RGBDNominalCalibration({imageHeight,imageWidth},{87.0,58.0});

  input RGBDGraphProcessing.State previous;
  parameter Integer channelCount(min=3,max=4) = 4 "RGB or historical RGBA storage";
  input Real rgb[imageHeight,imageWidth,channelCount];
  input Real depth[imageHeight,imageWidth];
  input Real depthUnits = 1.0 "Meters per depth sample";
  input Real rgbCalibration[4] = defaultRgbCalibration;
  input Real depthCalibration[4] = defaultDepthCalibration;
  input Real disparityNoise = 0.08;
  input Real noiseReferenceFx = 848.0/(2.0*tan(87.0*3.141592653589793/360.0));
  input Real baseline = 0.05;
  input Real opticalToBody[3,3] = [0.0,0.0,1.0;-1.0,0.0,0.0;0.0,-1.0,0.0];
  input Real cameraOriginBody[3] = {0.18,0.0,-0.04};

  input Real accel[3] = {0.0,0.0,9.81};
  input Real gyro[3] = zeros(3);
  input Real gravity[3] = {0.0,0.0,-9.81} annotation(Dialog(tab="SLAM", group="Inertial prediction"));
  input Real density[12] = {0.06,0.06,0.06,0.006,0.006,0.006,0.002,0.002,0.002,0.0002,0.0002,0.0002} annotation(Dialog(tab="SLAM", group="Inertial prediction"));
  input Real h = 1.0/90.0;
  input Real intervalTime;
  input Integer imageEpoch;
  input Boolean imageRequested = true;
  input Boolean localCaptureRequested = true annotation(Dialog(tab="SLAM", group="Visual reference capture"));
  input Boolean requested = true;

  input Integer minimumMeasuredDescriptors = 8 annotation(Dialog(tab="SLAM", group="Appearance vocabulary"));
  input Real minimumWordDistanceSquared = 0.04 annotation(Dialog(tab="SLAM", group="Appearance vocabulary"));
  input Real selectedFeatureLimit = featureCapacity annotation(Dialog(tab="SLAM", group="Feature selection"));
  input Real absoluteThreshold = max(8.0,18.0*min(1.0,imageWidth/848.0))
    "Retain lower-contrast corners in reduced rasters; keep the full-raster threshold" annotation(Dialog(tab="SLAM", group="Feature selection"));
  input Boolean depthQualifiedSelection = true annotation(Dialog(tab="SLAM", group="Feature selection"));
  input Real minimumInterval(unit="s") = 0.5 annotation(Dialog(tab="SLAM", group="Keyframe capture"));
  input Real maximumInterval(unit="s") = 2.0 annotation(Dialog(tab="SLAM", group="Keyframe capture"));
  input Real translationThreshold(unit="m") = 0.6 annotation(Dialog(tab="SLAM", group="Keyframe capture"));
  input Real rotationThreshold(unit="rad") = 0.25 annotation(Dialog(tab="SLAM", group="Keyframe capture"));
  input Integer minimumFeatures = 12 annotation(Dialog(tab="SLAM", group="Keyframe capture"));
  input Real voxelWidth = 0.25 annotation(Dialog(tab="SLAM", group="Map maintenance"));
  input Real mergeRadius = 0.15 annotation(Dialog(tab="SLAM", group="Map maintenance"));
  input Real maximumDistance = 80.0 annotation(Dialog(tab="SLAM", group="Map maintenance"));
  input Real tentativeLifetime = 0.5 annotation(Dialog(tab="SLAM", group="Map maintenance"));
  input Real confirmedLifetime = 5.0 annotation(Dialog(tab="SLAM", group="Map maintenance"));
  input Real confirmationObservations = 3.0 annotation(Dialog(tab="SLAM", group="Map maintenance"));
  input Real maximumConfidence = 8.0 annotation(Dialog(tab="SLAM", group="Map maintenance"));
  input Real maximumTentative = 700.0 annotation(Dialog(tab="SLAM", group="Map maintenance"));
  input Boolean graphCorrectionRequested = true annotation(Dialog(tab="SLAM", group="Graph optimization"));
  input RGBDGraphProcessing.Policy graphPolicy = RGBDGraphProcessing.DefaultPolicy() annotation(Dialog(tab="SLAM", group="Graph optimization"));

  input Real loopMinimumSimilarity = 0.35 annotation(Dialog(tab="SLAM", group="Loop retrieval"));
  input Real loopMinimumAge(unit="s") = 5.0 annotation(Dialog(tab="SLAM", group="Loop retrieval"));
  input Integer loopMinimumInliers = 12 annotation(Dialog(tab="SLAM", group="Loop verification"));
  input Real loopMinimumFraction = 0.5 annotation(Dialog(tab="SLAM", group="Loop verification"));
  input Real loopInlierDistance(unit="m") = 0.08 annotation(Dialog(tab="SLAM", group="Loop verification"));
  input Real loopMaximumRms(unit="m") = 0.03 annotation(Dialog(tab="SLAM", group="Loop verification"));
  input Boolean loopCalibratedResiduals = true annotation(Dialog(tab="SLAM", group="Loop verification"));
  input Real loopMaximumNormalizedSquared = 9.0 annotation(Dialog(tab="SLAM", group="Loop verification"));
  input Real loopMaximumPoseInnovation(unit="m") = 2.0 annotation(Dialog(tab="SLAM", group="Loop pose consistency"));
  input Real loopPoseConsistencySquared = 9.0 annotation(Dialog(tab="SLAM", group="Loop pose consistency"));
  input Boolean loopPoseConsistency = true annotation(Dialog(tab="SLAM", group="Loop pose consistency"));

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
  output Real nextQuaternion[4];
  output Real selectionValid;
  output Real predictionAccepted;
  output Real initializationAccepted;
  output Real observationAccepted;
  output Real captureAccepted;
  output Real matchCount;
  output Real features[featureCapacity,3];
  output Real featureEnabled[featureCapacity];
  output Real trackingCurrentPixel[featureCapacity,2];
  output Real trackingReferencePixel[featureCapacity,2];
  output Real trackingEnabled[featureCapacity];
  output Integer keyframeDecisionReason;
  output Integer keyframeRejectionReason;
  output Integer sequentialRejectionReason;
  output Integer frameRejectionDiagnostic;
  output Boolean loopAttempted;
  output Integer loopRetrievalReason;
  output Integer loopGraphReason;
  output Integer loopCandidateCount;
  output Integer loopCandidateReferenceId[RGBDKeyframeRetrieval.proposalCapacity];
  output Integer loopCandidateCurrentId[RGBDKeyframeRetrieval.proposalCapacity];
  output Real loopCandidateScore[RGBDKeyframeRetrieval.proposalCapacity];
  output Integer loopCandidateReason[RGBDKeyframeRetrieval.proposalCapacity];
  output Integer loopCandidateMatchedCount[RGBDKeyframeRetrieval.proposalCapacity];
  output Integer loopCandidateInliers[RGBDKeyframeRetrieval.proposalCapacity];
  output Real loopCandidateRms[RGBDKeyframeRetrieval.proposalCapacity];
  output Boolean loopCandidateVerified[RGBDKeyframeRetrieval.proposalCapacity];
  annotation(Documentation(info = "<html>
    <h4>Runtime loop tuning</h4>
<p>The loopMinimumSimilarity, loopMinimumAge, loopMinimumInliers, loopMinimumFraction, loopInlierDistance, loopMaximumRms, loopCalibratedResiduals and loopMaximumNormalizedSquared inputs tune retrieved-loop verification. The loopMaximumPoseInnovation, loopPoseConsistencySquared and loopPoseConsistency controls tune retained-pose screening. All loop controls preserve the separate consecutive-frame odometry settings. Calibrated residuals use the squared uncertainty-normalized gate; metric distance and RMS limits apply in metric mode. Keyframe interval, translation and rotation inputs tune capture cadence. Defaults retain the existing behavior.</p>
<p>loopAttempted marks capture attempts. Per-rank loopCandidate outputs describe the retrieved references and their actual verification outcomes, including rejected proposals. A verified proposal is not a committed loop: inspect graph admission and the published graph separately.</p>
    <h4>Session lifecycle</h4>
<p>This boundary owns the complete persistent visual session. Supply the prior
session as <code>previous</code>; initialization and step adapters produce
<code>next</code>. Keep frame epochs, interval time, calibration and requested
operations consistent with the supplied image and IMU data.</p>
<p>Use <a href=\"modelica://SLAM.RGBDFastSLAMInitialize\">RGBDFastSLAMInitialize</a>
for initialization, <a href=\"modelica://SLAM.RGBDFastSLAMStep\">RGBDFastSLAMStep</a>
for advancement, and <a href=\"modelica://SLAM.RGBDFastSLAMReset\">RGBDFastSLAMReset</a>
for an explicit reset. Inspect operation-specific acceptance and reason outputs;
a produced array alone does not mean its update was accepted.</p>
<p>The browser application handles transport and rendering. Modelica owns image
processing, geometry, estimator and map state. Export/compiler support of the
complete graph must be validated separately from the smaller fixed-map visual
fusion replays.</p>
    </html>"));
end RGBDFastSLAMInterface;
