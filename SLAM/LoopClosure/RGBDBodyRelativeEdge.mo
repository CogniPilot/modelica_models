within SLAM.LoopClosure;
model RGBDBodyRelativeEdge
  import RGBDOpticalToBodyEdge = SLAM.LoopClosure.RGBDOpticalToBodyEdge;

  parameter Real coordinateLimit = 100.0;
  parameter Real minimumPivot = 1e-10;
  input Real currentFromReference[3,3] = identity(3);
  input Real opticalTranslation[3] = zeros(3);
  input Real opticalCovariance[6,6] = identity(6);
  input Real referenceOpticalToBody[3,3] = identity(3);
  input Real currentOpticalToBody[3,3] = identity(3);
  input Real referenceCameraOrigin[3] = zeros(3);
  input Real currentCameraOrigin[3] = zeros(3);
  input Boolean requested = false;
  output Real measuredRotation[3,3]; output Real measuredTranslation[3];
  output Real residualJacobian[6,6]; output Real residualCovariance[6,6];
  output Real information[6,6]; output Boolean valid;
  output Integer rejectionReason; output Real minimumScaledPivot;
equation
  (measuredRotation,measuredTranslation,residualJacobian,residualCovariance,
    information,valid,rejectionReason,minimumScaledPivot) = RGBDOpticalToBodyEdge(
      currentFromReference,opticalTranslation,opticalCovariance,
      referenceOpticalToBody,currentOpticalToBody,referenceCameraOrigin,
      currentCameraOrigin,requested,coordinateLimit,minimumPivot);
end RGBDBodyRelativeEdge;
