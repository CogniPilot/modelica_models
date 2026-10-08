within Vision.Matching;
model RGBDFeatureMatching
  import MatchRGBDDescriptors = Vision.Matching.MatchRGBDDescriptors;

  constant Integer featureCapacity = 350;
  constant Integer descriptorWidth = 7;
  constant Integer descriptorSize = descriptorWidth*descriptorWidth;
  parameter Real ratio = 0.8;
  parameter Real maximumDescriptorDistance = 0.8;
  parameter Real maximumGeometricDistance = 0.5;
  input Real referenceDescriptor[featureCapacity,descriptorSize];
  input Real currentDescriptor[featureCapacity,descriptorSize];
  input Real referencePoint[featureCapacity,3];
  input Real currentPoint[featureCapacity,3];
  input Real referenceEnabled[featureCapacity];
  input Real currentEnabled[featureCapacity];
  input Real referenceCount;
  input Real currentCount;
  input Real usePrediction = 0.0;
  input Real predictedRotation[3,3] = identity(3);
  input Real predictedTranslation[3] = zeros(3);
  output Real currentIndex[featureCapacity];
  output Real pairEnabled[featureCapacity];
  output Real sourcePoint[featureCapacity,3];
  output Real targetPoint[featureCapacity,3];
  output Real count;
  output Real configurationValid;
  output Real invalidReference;
  output Real invalidCurrent;
  output Real nearestDistance[featureCapacity];
  output Real secondDistance[featureCapacity];
equation
  (currentIndex,pairEnabled,sourcePoint,targetPoint,count,configurationValid,invalidReference,invalidCurrent,nearestDistance,secondDistance) =
    MatchRGBDDescriptors(referenceDescriptor,currentDescriptor,referencePoint,currentPoint,referenceEnabled,currentEnabled,
      referenceCount,currentCount,ratio,maximumDescriptorDistance,usePrediction,predictedRotation,predictedTranslation,maximumGeometricDistance);
end RGBDFeatureMatching;
