within SLAM.Inertial;
// One complete ordering transaction: IMU -> relative correction -> reference capture.
// Each rejected substep preserves its incoming state. A valid IMU prediction
// still advances when a visual observation or reference replacement is rejected.
// The host persists these outputs together; it never rebuilds covariance blocks.
model ES15SchmidtReferenceStep
  import ES15SchmidtPrediction = SLAM.Inertial.ES15SchmidtPrediction;
  import SLAMExactRealEqual = SLAM.Inertial.SLAMExactRealEqual;
  import SchmidtImagePairGate = SLAM.Inertial.SchmidtImagePairGate;
  import SchmidtReferenceCapture = SLAM.Inertial.SchmidtReferenceCapture;
  import SchmidtRelativePoseCorrection = SLAM.Inertial.SchmidtRelativePoseCorrection;

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
  input Real opticalToBody[3,3] = [0.0,0.0,1.0;-1.0,0.0,0.0;0.0,-1.0,0.0];
  input Real cameraOriginBody[3] = {0.18,0.0,-0.04};
  input Real measuredRotation[3,3] = identity(3);
  input Real measuredTranslation[3] = zeros(3);
  input Real relativeCovariance[6,6] = identity(6);
  input Real measurementEnabled = 0.0;
  input Real captureRequested = 0.0;
  input Real referenceEpoch = 0.0;
  input Real currentEpoch = 0.0;
  input Real referenceUsed = 0.0;
  input Real lastUsedEpoch = -1.0;
  output Real predictionAccepted;
  output Real observationAccepted;
  output Real observationRejected;
  output Real captureAccepted;
  output Real captureRejected;
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
  output Real imagePairEligible;
  output Real imageReuseRejected;
protected
  SchmidtImagePairGate imageGate(referenceAvailable=referenceAvailable,referenceUsed=referenceUsed,
    referenceEpoch=referenceEpoch,currentEpoch=currentEpoch,lastUsedEpoch=lastUsedEpoch);
  Real pairAttempted;
  Real usedEpochAfterAttempt;
  ES15SchmidtPrediction prediction(position=position,velocity=velocity,rotation=rotation,
    accelBias=accelBias,gyroBias=gyroBias,covariance=covariance,crossCovariance=crossCovariance,
    referenceCovariance=referenceCovariance,referencePosition=referencePosition,
    referenceRotation=referenceRotation,referenceAvailable=referenceAvailable,
    accel=accel,gyro=gyro,gravity=gravity,h=h,density=density);
  SchmidtRelativePoseCorrection correction(position=prediction.nextPosition,
    velocity=prediction.nextVelocity,rotation=prediction.nextRotation,
    accelBias=accelBias,gyroBias=gyroBias,covariance=prediction.nextCovariance,
    crossCovariance=prediction.nextCrossCovariance,referenceCovariance=referenceCovariance,
    referencePosition=referencePosition,referenceRotation=referenceRotation,
    opticalToBody=opticalToBody,cameraOriginBody=cameraOriginBody,
    measuredRotation=measuredRotation,measuredTranslation=measuredTranslation,
    relativeCovariance=relativeCovariance,
    measurementEnabled=if noEvent(prediction.accepted > 0.5 and imageGate.eligible > 0.5)
      then measurementEnabled else 0.0);
  SchmidtReferenceCapture capture(position=correction.nextPosition,rotation=correction.nextRotation,
    covariance=correction.nextCovariance,crossCovariance=correction.nextCrossCovariance,
    referenceCovariance=correction.nextReferenceCovariance,
    referencePosition=referencePosition,referenceRotation=referenceRotation,
    referenceAvailable=referenceAvailable,captureRequested=captureRequested,
    currentValid=if noEvent(prediction.accepted > 0.5 and imageGate.captureFresh > 0.5
      and currentEpoch > usedEpochAfterAttempt) then 1.0 else 0.0);
equation
  // Consume an eligible evaluated pair even when its innovation is rejected.
  // This prevents conditioning on repeated trials of the same raw sensor noise.
  pairAttempted = if noEvent(prediction.accepted > 0.5 and imageGate.eligible > 0.5
    and SLAMExactRealEqual(measurementEnabled,1.0)) then 1.0 else 0.0;
  usedEpochAfterAttempt = if noEvent(pairAttempted > 0.5) then currentEpoch else lastUsedEpoch;
  imagePairEligible = imageGate.eligible;
  imageReuseRejected = if noEvent(prediction.accepted > 0.5 and SLAMExactRealEqual(measurementEnabled,1.0)
    and imageGate.eligible < 0.5) then 1.0 else 0.0;
  nextReferenceEpoch = if noEvent(capture.accepted > 0.5) then currentEpoch else referenceEpoch;
  nextReferenceUsed = if noEvent(capture.accepted > 0.5) then 0.0
    else if noEvent(pairAttempted > 0.5) then 1.0 else referenceUsed;
  nextLastUsedEpoch = usedEpochAfterAttempt;
  predictionAccepted = prediction.accepted;
  observationAccepted = correction.accepted;
  observationRejected = if noEvent(prediction.accepted > 0.5
    and not (SLAMExactRealEqual(measurementEnabled,0.0)) and correction.accepted < 0.5) then 1.0 else 0.0;
  captureAccepted = capture.accepted;
  captureRejected = capture.rejected;
  nextPosition = correction.nextPosition;
  nextVelocity = correction.nextVelocity;
  nextRotation = correction.nextRotation;
  nextAccelBias = correction.nextAccelBias;
  nextGyroBias = correction.nextGyroBias;
  nextCovariance = capture.nextCovariance;
  nextCrossCovariance = capture.nextCrossCovariance;
  nextReferenceCovariance = capture.nextReferenceCovariance;
  nextReferencePosition = capture.nextReferencePosition;
  nextReferenceRotation = capture.nextReferenceRotation;
  nextReferenceAvailable = capture.nextReferenceAvailable;
end ES15SchmidtReferenceStep;
