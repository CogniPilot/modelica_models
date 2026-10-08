within SLAM;
// Raw sensor data and the complete persistent Modelica session owner.
partial model RGBDFastSLAMInterface
  import RGBDGraphProcessing = SLAM.PoseGraph.RGBDGraphProcessing;
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
  input Real gravity[3] = {0.0,0.0,-9.81};
  input Real density[12] = {0.06,0.06,0.06,0.006,0.006,0.006,0.002,0.002,0.002,0.0002,0.0002,0.0002};
  input Real h = 1.0/90.0;
  input Real intervalTime;
  input Integer imageEpoch;
  input Boolean imageRequested = true;
  input Boolean localCaptureRequested = true;
  input Boolean requested = true;

  parameter Integer minimumMeasuredDescriptors = 8;
  parameter Real minimumWordDistanceSquared = 0.04;
  parameter Real selectedFeatureLimit = featureCapacity;
  parameter Real absoluteThreshold = 18.0;
  parameter Boolean depthQualifiedSelection = true;
  parameter Real minimumInterval = 0.5;
  parameter Real maximumInterval = 2.0;
  parameter Real translationThreshold = 0.6;
  parameter Real rotationThreshold = 0.25;
  parameter Integer minimumFeatures = 12;
  parameter Real voxelWidth = 0.25;
  parameter Real mergeRadius = 0.15;
  parameter Real maximumDistance = 80.0;
  parameter Real tentativeLifetime = 0.5;
  parameter Real confirmedLifetime = 5.0;
  parameter Real confirmationObservations = 3.0;
  parameter Real maximumConfidence = 8.0;
  parameter Real maximumTentative = 700.0;
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
  annotation(Documentation(info = "<html>
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
