within SLAM.LoopClosure;
model RGBDBagOfWords
  import RetrieveVisualWords = SLAM.LoopClosure.RetrieveVisualWords;

  constant Integer featureCapacity = 350;
  constant Integer descriptorSize = 49;
  constant Integer vocabularyCapacity = 256;
  constant Integer keyframeCapacity = 128;
  constant Integer candidateCapacity = 4;
  parameter Real maximumWordDistanceSquared = 0.8;
  parameter Real minimumAssignments = 8.0;
  parameter Real minimumSimilarity = 0.35;
  parameter Real minimumAge = 2.0 "Simulation seconds; independent of frame rate";
  input Real descriptor[featureCapacity,descriptorSize] = zeros(featureCapacity,descriptorSize);
  input Real descriptorEnabled[featureCapacity] = zeros(featureCapacity);
  input Real descriptorCount = 0.0;
  input Real vocabulary[vocabularyCapacity,descriptorSize] = zeros(vocabularyCapacity,descriptorSize);
  input Real vocabularyEnabled[vocabularyCapacity] = zeros(vocabularyCapacity);
  input Real vocabularyVersion = 1.0;
  input Real previousHistogram[keyframeCapacity,vocabularyCapacity] = zeros(keyframeCapacity,vocabularyCapacity);
  input Real previousEnabled[keyframeCapacity] = zeros(keyframeCapacity);
  input Real previousKeyframeId[keyframeCapacity] = zeros(keyframeCapacity);
  input Real previousKeyframeTime[keyframeCapacity] = zeros(keyframeCapacity);
  input Real previousVersion = 0.0;
  input Real previousNextSlot = 1.0;
  input Real previousTime = 0.0;
  input Real queryKeyframeId = 0.0;
  input Real timeNow = 0.0;
  input Real storeKeyframe = 0.0;
  input Real resetRequested = 0.0;
  output Real wordIndex[featureCapacity];
  output Real histogram[vocabularyCapacity];
  output Real candidateId[candidateCapacity];
  output Real candidateSlot[candidateCapacity];
  output Real candidateScore[candidateCapacity];
  output Real nextHistogram[keyframeCapacity,vocabularyCapacity];
  output Real nextEnabled[keyframeCapacity];
  output Real nextKeyframeId[keyframeCapacity];
  output Real nextKeyframeTime[keyframeCapacity];
  output Real nextVersion;
  output Real nextSlot;
  output Real nextTime;
  output Real configurationValid;
  output Real vocabularyCount;
  output Real assignmentCount;
  output Real retrievalValid;
  output Real candidateCount;
  output Real stored;
  output Real invalidHistoryCount;
equation
  (wordIndex,histogram,candidateId,candidateSlot,candidateScore,nextHistogram,nextEnabled,nextKeyframeId,nextKeyframeTime,
    nextVersion,nextSlot,nextTime,configurationValid,vocabularyCount,assignmentCount,retrievalValid,candidateCount,stored,invalidHistoryCount) =
    RetrieveVisualWords(descriptor,descriptorEnabled,descriptorCount,vocabulary,vocabularyEnabled,vocabularyVersion,
      previousHistogram,previousEnabled,previousKeyframeId,previousKeyframeTime,previousVersion,previousNextSlot,previousTime,
      queryKeyframeId,timeNow,storeKeyframe,resetRequested,maximumWordDistanceSquared,minimumAssignments,minimumSimilarity,minimumAge);
end RGBDBagOfWords;
