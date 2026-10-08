within SLAM.Localization;
// Review graph: raw RGB/depth -> descriptors -> matches -> rigid registration
// -> map-frame body pose. Compile with RGBDFeatureMatching,
// RigidPointRegistration and RGBDRelativePose source files.
// The reference descriptors, optical points and body pose are retained estimated
// keyframe data. No ground-truth pose or host feature matching enters this graph.
// This is a visual observation frontend, not a persistent SLAM estimator.
model RGBDVisualObservation
  import RGBDDescriptorFrame = Vision.Matching.RGBDDescriptorFrame;
  import RGBDFeatureMatching = Vision.Matching.RGBDFeatureMatching;
  import RGBDRelativePose = SLAM.Localization.RGBDRelativePose;
  import RigidPointRegistration = Vision.Registration.RigidPointRegistration;

  parameter Integer imageWidth(min=1) = 160;
  parameter Integer imageHeight(min=1) = 90;
  // RGB8 uses three channels; historical RGBA recordings may retain alpha.
  parameter Integer channelCount(min=3,max=4) = 4;
  constant Integer featureCapacity = 350;
  constant Integer descriptorWidth = 7;
  constant Integer descriptorSize = descriptorWidth*descriptorWidth;
  input Boolean imageEnabled = true;
  input Real rgb[imageHeight,imageWidth,channelCount];
  input Real depth[imageHeight,imageWidth];
  input Real depthUnits = 1.0 "Meters per depth sample";
  input Real pixels[featureCapacity,2];
  input Real activeCount;
  input Real rgbCalibration[4];
  input Real depthCalibration[4];
  input Real disparityNoise = 0.08;
  input Real noiseReferenceFx;
  input Real baseline = 0.05;
  input Real referenceDescriptor[featureCapacity,descriptorSize];
  input Real referencePoint[featureCapacity,3];
  input Real referenceEnabled[featureCapacity];
  input Real referenceCount;
  input Real referenceBodyRotation[3,3] = identity(3);
  input Real referenceBodyPosition[3] = zeros(3);
  input Real opticalToBody[3,3] = [0.0,0.0,1.0;-1.0,0.0,0.0;0.0,-1.0,0.0];
  input Real cameraOriginBody[3] = {0.18,0.0,-0.04};
  // Optional predicted optical reference->current transform for association.
  input Real usePrediction = 0.0;
  input Real predictedRotation[3,3] = identity(3);
  input Real predictedTranslation[3] = zeros(3);
  output Real observedBodyRotation[3,3];
  output Real observedBodyPosition[3];
  output Real valid;
  output Real matchCount;
  output Real registrationRms;
  output Real registrationRejectionReason;
  output Real currentIndex[featureCapacity];
  // Complete current frame data for retaining a future reference keyframe.
  output Real currentDescriptor[featureCapacity,descriptorSize];
  output Real currentPoint[featureCapacity,3];
  output Real currentEnabled[featureCapacity];
protected
  RGBDDescriptorFrame description(imageHeight=imageHeight,imageWidth=imageWidth,
    channelCount=channelCount,imageEnabled=imageEnabled,rgb=rgb,depth=depth,depthUnits=depthUnits,pixels=pixels,
    activeCount=activeCount,rgbCalibration=rgbCalibration,depthCalibration=depthCalibration,
    disparityNoise=disparityNoise,noiseReferenceFx=noiseReferenceFx,baseline=baseline);
  RGBDFeatureMatching matching(referenceDescriptor=referenceDescriptor,
    currentDescriptor=description.descriptor,referencePoint=referencePoint,
    currentPoint=description.point,referenceEnabled=referenceEnabled,
    currentEnabled=description.enabled,referenceCount=referenceCount,currentCount=activeCount,
    usePrediction=usePrediction,predictedRotation=predictedRotation,
    predictedTranslation=predictedTranslation);
  // Pair masks can have holes anywhere in the full reference domain. Using
  // matching.count as activeCount would silently discard matches in later slots.
  RigidPointRegistration registration(capacity=featureCapacity,
    activeCount=featureCapacity,sourcePoint=matching.sourcePoint,
    targetPoint=matching.targetPoint,pairEnabled=matching.pairEnabled);
  RGBDRelativePose pose(referenceBodyRotation=referenceBodyRotation,
    referenceBodyPosition=referenceBodyPosition,currentFromReference=registration.rotation,
    currentFromReferenceTranslation=registration.translation,opticalToBody=opticalToBody,
    cameraOriginBody=cameraOriginBody,registrationAccepted=registration.accepted);
equation
  observedBodyRotation = pose.observedBodyRotation;
  observedBodyPosition = pose.observedBodyPosition;
  valid = if noEvent(matching.configurationValid > 0.5) then pose.valid else 0.0;
  matchCount = matching.count;
  registrationRms = registration.rms;
  registrationRejectionReason = registration.rejectionReason;
  currentIndex = matching.currentIndex;
  currentDescriptor = description.descriptor;
  currentPoint = description.point;
  currentEnabled = description.enabled;
end RGBDVisualObservation;
