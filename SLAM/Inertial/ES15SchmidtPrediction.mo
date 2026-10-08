within SLAM.Inertial;
// Predict a complete current/reference state using the actual ES15 transition.
model ES15SchmidtPrediction
  import ES15PredictHeldInterval = SLAM.Inertial.ES15PredictHeldInterval;

  input Real position[3] = zeros(3);
  input Real velocity[3] = zeros(3);
  input Real rotation[3,3] = identity(3);
  input Real accelBias[3] = zeros(3);
  input Real gyroBias[3] = zeros(3);
  input Real covariance[15,15];
  input Real crossCovariance[15,6];
  input Real referenceCovariance[6,6];
  input Real referencePosition[3] = zeros(3);
  input Real referenceRotation[3,3] = identity(3);
  input Real referenceAvailable = 0.0;
  input Real accel[3] = {0.0,0.0,9.81};
  input Real gyro[3] = zeros(3);
  input Real gravity[3] = {0.0,0.0,-9.81};
  input Real h = 1.0/90.0;
  input Real density[12] = {0.06,0.06,0.06,0.006,0.006,0.006,0.002,0.002,0.002,0.0002,0.0002,0.0002};
  output Real accepted;
  output Real transition[15,15];
  output Real processCovariance[15,15];
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
  output Integer substeps "Numerical substeps within this unchanged held measurement";
algorithm
  (accepted,transition,processCovariance,nextPosition,nextVelocity,nextRotation,
    nextCovariance,nextCrossCovariance,substeps) := ES15PredictHeldInterval(
      position,velocity,rotation,accelBias,gyroBias,covariance,crossCovariance,
      referenceCovariance,referencePosition,referenceRotation,referenceAvailable,
      accel,gyro,gravity,h,density);
  nextAccelBias := accelBias;
  nextGyroBias := gyroBias;
  nextReferenceCovariance := referenceCovariance;
  nextReferencePosition := referencePosition;
  nextReferenceRotation := referenceRotation;
  nextReferenceAvailable := referenceAvailable;
end ES15SchmidtPrediction;
