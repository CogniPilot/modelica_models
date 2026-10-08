within SLAM;
// Fresh state construction runs in the compiled Modelica program. Reload must
// restore the complete source-bound State, rather than invoking this reset.
model RGBDFastSLAMReset
  import RGBDGraphProcessing = SLAM.PoseGraph.RGBDGraphProcessing;
  import RGBDKeyframes = SLAM.LoopClosure.RGBDKeyframes;
  import RGBDLocalizationCatalog = SLAM.Localization.RGBDLocalizationCatalog;
  import RGBDLocalizationFrame = SLAM.Localization.RGBDLocalizationFrame;

  constant Integer dimension = RGBDKeyframes.dimension;
  constant Integer currentDimension = RGBDLocalizationFrame.currentDimension;
  input Integer generation = 1;
  input Integer sourceRevision = 1;
  input Integer vocabularyVersion = 1;
  input Real worldFrame = 0;
  input Real position[dimension] = zeros(dimension);
  input Real velocity[dimension] = zeros(dimension);
  input Real rotation[dimension,dimension] = identity(dimension);
  input Real accelBias[dimension] = zeros(dimension);
  input Real gyroBias[dimension] = zeros(dimension);
  input Real covariance[currentDimension,currentDimension] = diagonal({0.25,0.25,0.25,
    0.04,0.04,0.04,0.01,0.01,0.01,0.0004,0.0004,0.0004,0.000025,0.000025,0.000025});
  output RGBDGraphProcessing.State next;
equation
  next = RGBDGraphProcessing.Empty(RGBDLocalizationCatalog.Empty(
    RGBDLocalizationCatalog.EmptyEstimator(position,velocity,rotation,accelBias,gyroBias,covariance),
    generation,sourceRevision,vocabularyVersion,worldFrame));
end RGBDFastSLAMReset;
