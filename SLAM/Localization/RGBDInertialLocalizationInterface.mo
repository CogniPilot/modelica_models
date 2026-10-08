within SLAM.Localization;
// Concrete carried-state localization transaction. Persistence copies these
// inputs/outputs verbatim; all image, pose, covariance and commit math is Modelica.
// Image dimensions are structural parameters;350 selected slots remain bounded.
// Pixel selection is a source-owned
// upstream stage (see RGBDFastInertialLocalizationStep). No loop closure here.
partial model RGBDInertialLocalizationInterface
  import RGBDNominalCalibration = Vision.Matching.RGBDNominalCalibration;

  parameter Integer imageHeight(min=1) = 90;
  parameter Integer imageWidth(min=1) = 160;
  constant Integer featureCapacity = 350;
  constant Integer descriptorSize = 49;
  parameter Real initialPositionVariance = 0.25;
  parameter Real initialVelocityVariance = 0.04;
  parameter Real initialAttitudeVariance = 0.01;
  parameter Real initialAccelBiasVariance = 0.0004;
  parameter Real initialGyroBiasVariance = 0.000025;
  final parameter Real defaultRgbCalibration[4] = RGBDNominalCalibration({imageHeight,imageWidth},{69.0,42.0});
  final parameter Real defaultDepthCalibration[4] = RGBDNominalCalibration({imageHeight,imageWidth},{87.0,58.0});
  constant Real defaultNoiseReferenceFx = 848.0/(2.0*tan(87.0*3.141592653589793/360.0));
  parameter Integer channelCount(min=3,max=4) = 4 "RGB or historical RGBA storage";
  input Real rgb[imageHeight,imageWidth,channelCount];
  input Real depth[imageHeight,imageWidth];
  input Real depthUnits = 1.0 "Meters per depth sample";
  input Real rgbCalibration[4] = defaultRgbCalibration;
  input Real depthCalibration[4] = defaultDepthCalibration;
  input Real disparityNoise = 0.08;
  input Real noiseReferenceFx = defaultNoiseReferenceFx;
  input Real baseline = 0.05;
  input Real opticalToBody[3,3] = [0.0,0.0,1.0;-1.0,0.0,0.0;0.0,-1.0,0.0];
  input Real cameraOriginBody[3] = {0.18,0.0,-0.04};
  input Real frameEnabled = 0.0 "Exactly1 only on final held-IMU interval for this image";
  input Real imageCaptureRequested = 0.0;
  input Real position[3] = zeros(3);
  input Real velocity[3] = zeros(3);
  input Real rotation[3,3] = identity(3);
  input Real accelBias[3] = zeros(3);
  input Real gyroBias[3] = zeros(3);
  // Estimated level origin, zero velocity/bias, gravity as declared below.
  // These tunable prior variances are initialization hypotheses, not truth or
  // registration noise. Relative corrections use the actual sandwich output.
  input Real covariance[15,15] = diagonal({initialPositionVariance,initialPositionVariance,initialPositionVariance,
    initialVelocityVariance,initialVelocityVariance,initialVelocityVariance,
    initialAttitudeVariance,initialAttitudeVariance,initialAttitudeVariance,
    initialAccelBiasVariance,initialAccelBiasVariance,initialAccelBiasVariance,
    initialGyroBiasVariance,initialGyroBiasVariance,initialGyroBiasVariance});
  input Real crossCovariance[15,6] = zeros(15,6);
  input Real referenceCovariance[6,6] = zeros(6,6);
  input Real referencePosition[3] = zeros(3);
  input Real referenceRotation[3,3] = identity(3);
  input Real referenceAvailable = 0.0;
  input Real referenceEpoch = 0.0;
  input Real currentEpoch = 0.0;
  input Real referenceUsed = 0.0;
  input Real lastUsedEpoch = -1.0;
  input Real referenceDescriptor[featureCapacity,descriptorSize] = zeros(featureCapacity,descriptorSize);
  input Real referencePoint[featureCapacity,3] = zeros(featureCapacity,3);
  input Real referenceEnabled[featureCapacity] = zeros(featureCapacity);
  input Real referencePixels[featureCapacity,2] = zeros(featureCapacity,2);
  input Real referenceCount = 0.0;
  input Real referenceRgbCalibration[4] = defaultRgbCalibration;
  input Real referenceDepthCalibration[4] = defaultDepthCalibration;
  input Real referenceNoiseReferenceFx = defaultNoiseReferenceFx;
  input Real referenceDisparityNoise = 0.08;
  input Real referenceBaseline = 0.05;
  input Real referenceOpticalToBody[3,3] = [0.0,0.0,1.0;-1.0,0.0,0.0;0.0,-1.0,0.0];
  input Real referenceCameraOriginBody[3] = {0.18,0.0,-0.04};
  input Real accel[3] = {0.0,0.0,9.81};
  input Real gyro[3] = zeros(3);
  input Real gravity[3] = {0.0,0.0,-9.81};
  input Real h = 1.0/90.0;
  input Real density[12] = {0.06,0.06,0.06,0.006,0.006,0.006,0.002,0.002,0.002,0.0002,0.0002,0.0002};
  output Real nextPosition[3]; output Real nextVelocity[3];
  output Real nextRotation[3,3]; output Real nextAccelBias[3]; output Real nextGyroBias[3];
  output Real nextCovariance[15,15]; output Real nextCrossCovariance[15,6];
  output Real nextReferenceCovariance[6,6]; output Real nextReferencePosition[3];
  output Real nextReferenceRotation[3,3]; output Real nextReferenceAvailable;
  output Real nextReferenceEpoch; output Real nextReferenceUsed; output Real nextLastUsedEpoch;
  output Real nextReferenceDescriptor[featureCapacity,descriptorSize];
  output Real nextReferencePoint[featureCapacity,3]; output Real nextReferenceEnabled[featureCapacity];
  output Real nextReferencePixels[featureCapacity,2];
  output Real nextReferenceCount; output Real nextReferenceRgbCalibration[4];
  output Real nextReferenceDepthCalibration[4]; output Real nextReferenceNoiseReferenceFx;
  output Real nextReferenceDisparityNoise; output Real nextReferenceBaseline;
  output Real nextReferenceOpticalToBody[3,3]; output Real nextReferenceCameraOriginBody[3];
  output Real predictionAccepted; output Real observationAccepted; output Real observationRejected;
  output Real captureAccepted; output Real captureRejected; output Real imageReuseRejected;
  output Real imagePairEligible; output Real frameValid; output Real referenceGeometryCompatible;
  output Real visualValid; output Real matchCount; output Real uncertaintyRejectionReason;
  output Real currentDescriptor[featureCapacity,descriptorSize];
  output Real currentPoint[featureCapacity,3]; output Real currentEnabled[featureCapacity];
  output Real currentCount "Selected feature domain extent, including disabled slots; not a valid-point count";
  output Real currentFromReference[3,3]; output Real currentFromReferenceTranslation[3];
  output Real relativeCovariance[6,6];
  output Real mapCandidatePoint[featureCapacity,3]; output Real mapCandidateEnabled[featureCapacity];
  output Real mapCandidateCount;
  output Real nextQuaternion[4] "Normalized w,x,y,z; body FLU to world ENU";
  output Real positionCovariance[3,3]; output Real attitudeCovariance[3,3];
  output Real confidence "Binary accepted relative correction indicator; not posterior probability";
  output Real features[featureCapacity,3]; output Real featureEnabled[featureCapacity];
  output Real trackingCurrentPixel[featureCapacity,2];
  output Real trackingReferencePixel[featureCapacity,2]; output Real trackingEnabled[featureCapacity];
end RGBDInertialLocalizationInterface;
