within SLAM.Localization;
// Complete time-zero raw RGB-D transaction; no positive-duration prediction.
// Ordered array math stays in this reusable function, with unchanged model I/O.
function InitializeRGBDLocalization
  import DescribeRGBDFrame = Vision.Matching.DescribeRGBDFrame;
  import RGBDProjectLandmarks = SLAM.Mapping.RGBDProjectLandmarks;
  import RGBDProperRotationValue = SLAM.Localization.RGBDProperRotationValue;
  import SLAMCovariancePSDCheck = SLAM.Inertial.SLAMCovariancePSDCheck;
  import SLAMExactRealEqual = SLAM.Inertial.SLAMExactRealEqual;
  import SLAMRotationCoordinates = SLAM.Inertial.SLAMRotationCoordinates;
  import SchmidtCaptureReference = SLAM.Inertial.SchmidtCaptureReference;
  import SchmidtImagePairEligibility = SLAM.Inertial.SchmidtImagePairEligibility;

  input Real rgb[:,:,:];
  input Real depth[size(rgb,1),size(rgb,2)];
  input Real rgbCalibration[4];
  input Real depthCalibration[4];
  input Real disparityNoise;
  input Real noiseReferenceFx;
  input Real baseline;
  input Real opticalToBody[3,3];
  input Real cameraOriginBody[3];
  input Real frameEnabled;
  input Real imageCaptureRequested;
  input Real position[3];
  input Real velocity[3];
  input Real rotation[3,3];
  input Real accelBias[3];
  input Real gyroBias[3];
  input Real covariance[15,15];
  input Real crossCovariance[15,6];
  input Real referenceCovariance[6,6];
  input Real referencePosition[3];
  input Real referenceRotation[3,3];
  input Real referenceAvailable;
  input Real referenceEpoch;
  input Real currentEpoch;
  input Real referenceUsed;
  input Real lastUsedEpoch;
  input Real referenceDescriptor[featureCapacity,descriptorSize];
  input Real referencePoint[featureCapacity,3];
  input Real referenceEnabled[featureCapacity];
  input Real referencePixels[featureCapacity,2];
  input Real referenceCount;
  input Real referenceRgbCalibration[4];
  input Real referenceDepthCalibration[4];
  input Real referenceNoiseReferenceFx;
  input Real referenceDisparityNoise;
  input Real referenceBaseline;
  input Real referenceOpticalToBody[3,3];
  input Real referenceCameraOriginBody[3];
  input Real accel[3];
  input Real gyro[3];
  input Real gravity[3];
  input Real h;
  input Real density[12];
  input Real pixels[featureCapacity,2];
  input Real activeCount;
  input Real featureScore[featureCapacity];
  input Real imageTime;
  input Real initializationRequested;
  output Real nextPosition[3];
  output Real nextVelocity[3];
  output Real nextRotation[3,3];
  output Real nextAccelBias[3];
  output Real nextGyroBias[3];
  output Real nextCovariance[15,15];
  output Real nextCrossCovariance[15,6];
  output Real nextReferenceCovariance[6,6];
  output Real nextReferencePosition[3];
  output Real nextReferenceRotation[3,3];
  output Real nextReferenceAvailable;
  output Real nextReferenceEpoch;
  output Real nextReferenceUsed;
  output Real nextLastUsedEpoch;
  output Real nextReferenceDescriptor[featureCapacity,descriptorSize];
  output Real nextReferencePoint[featureCapacity,3];
  output Real nextReferenceEnabled[featureCapacity];
  output Real nextReferencePixels[featureCapacity,2];
  output Real nextReferenceCount;
  output Real nextReferenceRgbCalibration[4];
  output Real nextReferenceDepthCalibration[4];
  output Real nextReferenceNoiseReferenceFx;
  output Real nextReferenceDisparityNoise;
  output Real nextReferenceBaseline;
  output Real nextReferenceOpticalToBody[3,3];
  output Real nextReferenceCameraOriginBody[3];
  output Real predictionAccepted;
  output Real observationAccepted;
  output Real observationRejected;
  output Real captureAccepted;
  output Real captureRejected;
  output Real imageReuseRejected;
  output Real imagePairEligible;
  output Real frameValid;
  output Real referenceGeometryCompatible;
  output Real visualValid;
  output Real matchCount;
  output Real uncertaintyRejectionReason;
  output Real currentDescriptor[featureCapacity,descriptorSize];
  output Real currentPoint[featureCapacity,3];
  output Real currentEnabled[featureCapacity];
  output Real currentCount;
  output Real currentFromReference[3,3];
  output Real currentFromReferenceTranslation[3];
  output Real relativeCovariance[6,6];
  output Real mapCandidatePoint[featureCapacity,3];
  output Real mapCandidateEnabled[featureCapacity];
  output Real mapCandidateCount;
  output Real nextQuaternion[4];
  output Real positionCovariance[3,3];
  output Real attitudeCovariance[3,3];
  output Real confidence;
  output Real features[featureCapacity,3];
  output Real featureEnabled[featureCapacity];
  output Real trackingCurrentPixel[featureCapacity,2];
  output Real trackingReferencePixel[featureCapacity,2];
  output Real trackingEnabled[featureCapacity];
  output Real initializationAccepted;
  output Real initializationRejected;
  input Real minimumContrast = 1e-6;
  input Real nearDepth = 0.28;
  input Real farDepth = 10.0;
  input Real depthUnits = 1.0 "Meters per depth sample; 1 for metric depth, SDK scale for Z16";
protected
  constant Integer featureCapacity = 350;
  constant Integer descriptorSize = 49;
  Boolean imageOn;
  Real nominalChecks[3]; Real densityChecks[12];
  Real originChecks[3]; Real cameraChecks[3,3]; Real referenceOriginChecks[3];
  Real currentCovarianceValid; Real jointCovarianceValid;
  Real priorJoint[21,21]; Real priorValid;
  Real pairValid; Real pairEligible; Real captureFresh;
  Real descriptorInvalidCount; Real captureRejectedInternal;
  Real projectionInvalidCount; Real projectionConfigurationValid; Real projectionPoseValid;
  Real orientationVector[3]; Real orientationAngle; Real orientationValid; Real orientationQuaternion[4];
algorithm
  imageOn := SLAMExactRealEqual(frameEnabled,1.0);
  (pairValid,pairEligible,captureFresh) := SchmidtImagePairEligibility(referenceAvailable,
    referenceUsed,referenceEpoch,currentEpoch,lastUsedEpoch);
  priorJoint := cat(1,cat(2,covariance,crossCovariance),
    cat(2,transpose(crossCovariance),referenceCovariance));
  currentCovarianceValid := SLAMCovariancePSDCheck(covariance,1e-12);
  jointCovarianceValid := SLAMCovariancePSDCheck(priorJoint,1e-12);
  for axis in 1:3 loop
    nominalChecks[axis] := if noEvent(abs(position[axis]) <= 1e6
      and abs(velocity[axis]) <= 1e6 and abs(accelBias[axis]) <= 2.0
      and abs(gyroBias[axis]) <= 0.3 and abs(gravity[axis]) <= 1e3
      and abs(referencePosition[axis]) <= 1e6) then 0.0 else 1.0;
    originChecks[axis] := if noEvent(abs(cameraOriginBody[axis]) <= 10.0) then 0.0 else 1.0;
    referenceOriginChecks[axis] := if noEvent(SLAMExactRealEqual(referenceCameraOriginBody[axis],cameraOriginBody[axis]))
      then 0.0 else 1.0;
    for column in 1:3 loop
      cameraChecks[axis,column] := if noEvent(SLAMExactRealEqual(referenceOpticalToBody[axis,column],opticalToBody[axis,column]))
        then 0.0 else 1.0;
    end for;
  end for;
  for channel in 1:12 loop
    densityChecks[channel] := if noEvent(density[channel] >= 0.0 and density[channel] <= 1e6) then 0.0 else 1.0;
  end for;
  // Complete prior validation includes unavailable cross/reference buffers;
  // initialization never erases a malformed prior to manufacture a cold clone.
  priorValid := if noEvent(RGBDProperRotationValue(rotation) > 0.5 and RGBDProperRotationValue(referenceRotation) > 0.5
    and sum(nominalChecks)+sum(densityChecks) < 0.5
    and currentCovarianceValid > 0.5 and jointCovarianceValid > 0.5
    and pairValid > 0.5) then 1.0 else 0.0;
  initializationAccepted := if noEvent(SLAMExactRealEqual(initializationRequested,1.0)
    and SLAMExactRealEqual(imageTime,0.0) and priorValid > 0.5
    and (SLAMExactRealEqual(frameEnabled,0.0) or imageOn)) then 1.0 else 0.0;
  initializationRejected := if noEvent(SLAMExactRealEqual(initializationRequested,0.0)
    or initializationAccepted > 0.5) then 0.0 else 1.0;
  // Raw image reads stay inside the existing acquisition guard. Alpha is ignored.
  (currentDescriptor,currentPoint,currentEnabled,descriptorInvalidCount) := DescribeRGBDFrame(
    rgb,depth,pixels,if imageOn and initializationAccepted > 0.5 then activeCount else 0.0,
    rgbCalibration,depthCalibration,disparityNoise,noiseReferenceFx,baseline,
    nearDepth,farDepth,minimumContrast,imageOn and initializationAccepted > 0.5,depthUnits=depthUnits);
  frameValid := if noEvent(imageOn and initializationAccepted > 0.5
    and activeCount >= 0.0 and activeCount <= featureCapacity
    and SLAMExactRealEqual(activeCount,floor(activeCount)) and sum(currentEnabled) >= 3.0
    and disparityNoise > 0.0 and disparityNoise <= 1.0
    and noiseReferenceFx >= 1e-6 and noiseReferenceFx <= 1e6
    and baseline >= 1e-6 and baseline <= 1.0 and RGBDProperRotationValue(opticalToBody) > 0.5
    and sum(originChecks) < 0.5) then 1.0 else 0.0;
  referenceGeometryCompatible := if noEvent(sum(cameraChecks)+sum(referenceOriginChecks) < 0.5
    and SLAMExactRealEqual(referenceBaseline,baseline)
    and SLAMExactRealEqual(referenceDisparityNoise,disparityNoise)) then 1.0 else 0.0;
  (captureAccepted,captureRejectedInternal,nextReferenceAvailable,nextReferencePosition,
    nextReferenceRotation,nextCovariance,nextCrossCovariance,nextReferenceCovariance)
    := SchmidtCaptureReference(position,rotation,covariance,referencePosition,referenceRotation,
      crossCovariance,referenceCovariance,referenceAvailable,
      if initializationAccepted > 0.5 and frameValid > 0.5 then imageCaptureRequested else 0.0,
      if initializationAccepted > 0.5 and frameValid > 0.5 and captureFresh > 0.5 then 1.0 else 0.0);
  (mapCandidatePoint,mapCandidateEnabled,mapCandidateCount,projectionInvalidCount,
    projectionConfigurationValid,projectionPoseValid) := RGBDProjectLandmarks(
      currentPoint,currentEnabled,if imageOn then activeCount else 0.0,captureAccepted,
      rotation,position,opticalToBody,cameraOriginBody);
  // No positive-duration component is instantiated. These unused inherited
  // inputs (h,accel,gyro) cannot trigger prediction or alter any current mean/P.
  predictionAccepted := 0.0; observationAccepted := 0.0; observationRejected := 0.0;
  captureRejected := if noEvent(imageOn and SLAMExactRealEqual(imageCaptureRequested,1.0)
    and (initializationAccepted < 0.5 or frameValid < 0.5)) then 1.0 else captureRejectedInternal;
  imagePairEligible := 0.0; imageReuseRejected := 0.0;
  nextPosition := position; nextVelocity := velocity; nextRotation := rotation;
  nextAccelBias := accelBias; nextGyroBias := gyroBias;
  nextReferenceEpoch := if noEvent(captureAccepted > 0.5) then currentEpoch else referenceEpoch;
  nextReferenceUsed := if noEvent(captureAccepted > 0.5) then 0.0 else referenceUsed;
  nextLastUsedEpoch := lastUsedEpoch;
  nextReferenceDescriptor := if noEvent(captureAccepted > 0.5) then currentDescriptor else referenceDescriptor;
  nextReferencePoint := if noEvent(captureAccepted > 0.5) then currentPoint else referencePoint;
  nextReferenceEnabled := if noEvent(captureAccepted > 0.5) then currentEnabled else referenceEnabled;
  nextReferencePixels := if noEvent(captureAccepted > 0.5) then pixels else referencePixels;
  nextReferenceCount := if noEvent(captureAccepted > 0.5) then activeCount else referenceCount;
  nextReferenceRgbCalibration := if noEvent(captureAccepted > 0.5) then rgbCalibration else referenceRgbCalibration;
  nextReferenceDepthCalibration := if noEvent(captureAccepted > 0.5) then depthCalibration else referenceDepthCalibration;
  nextReferenceNoiseReferenceFx := if noEvent(captureAccepted > 0.5) then noiseReferenceFx else referenceNoiseReferenceFx;
  nextReferenceDisparityNoise := if noEvent(captureAccepted > 0.5) then disparityNoise else referenceDisparityNoise;
  nextReferenceBaseline := if noEvent(captureAccepted > 0.5) then baseline else referenceBaseline;
  nextReferenceOpticalToBody := if noEvent(captureAccepted > 0.5) then opticalToBody else referenceOpticalToBody;
  nextReferenceCameraOriginBody := if noEvent(captureAccepted > 0.5) then cameraOriginBody else referenceCameraOriginBody;
  currentCount := if noEvent(imageOn) then activeCount else 0.0;
  visualValid := 0.0; matchCount := 0.0; uncertaintyRejectionReason := 0.0;
  currentFromReference := identity(3); currentFromReferenceTranslation := zeros(3);
  relativeCovariance := zeros(6,6);
  (orientationVector,orientationAngle,orientationValid,orientationQuaternion) := SLAMRotationCoordinates(rotation);
  nextQuaternion := if orientationValid > 0.5 then orientationQuaternion else {1.0,0.0,0.0,0.0};
  positionCovariance := covariance[1:3,1:3]; attitudeCovariance := covariance[7:9,7:9];
  confidence := 0.0;
  featureEnabled := currentEnabled;
  for feature in 1:featureCapacity loop
    features[feature,1] := if noEvent(currentEnabled[feature] > 0.5) then pixels[feature,1] else 0.0;
    features[feature,2] := if noEvent(currentEnabled[feature] > 0.5) then pixels[feature,2] else 0.0;
    features[feature,3] := if noEvent(currentEnabled[feature] > 0.5 and abs(featureScore[feature]) <= 1e30)
      then featureScore[feature] else 0.0;
  end for;
  trackingCurrentPixel := zeros(featureCapacity,2); trackingReferencePixel := zeros(featureCapacity,2);
  trackingEnabled := zeros(featureCapacity);
end InitializeRGBDLocalization;
