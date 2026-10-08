within SLAM.Localization;
model RGBDRegistrationUncertainty
  import RGBDRegistrationSandwich = SLAM.Localization.RGBDRegistrationSandwich;

  parameter Integer capacity = 350;
  parameter Real coordinateLimit = 100.0;
  parameter Real minimumPivot = 1e-10;
  parameter Real localizationSigma = 0.5;
  parameter Real disparitySigma = 0.1;
  parameter Real depthInflation = 1.0;
  input Real referencePoint[capacity,3] = zeros(capacity,3);
  input Real currentPoint[capacity,3] = zeros(capacity,3);
  input Real pairEnabled[capacity] = zeros(capacity);
  input Real activeCount = 0.0;
  input Real registrationAccepted = 0.0;
  input Real currentFromReference[3,3] = identity(3);
  input Real translation[3] = zeros(3);
  input Real referenceBodyRotation[3,3] = identity(3);
  input Real opticalToBody[3,3] = [0.0,0.0,1.0;-1.0,0.0,0.0;0.0,-1.0,0.0];
  input Real cameraOriginBody[3] = {0.18,0.0,-0.04};
  input Real referenceRgbFocal[2] = {116.4,116.4};
  input Real currentRgbFocal[2] = {116.4,116.4};
  input Real referenceNoiseFx = 848.0/(2.0*tan(87.0*3.141592653589793/360.0));
  input Real currentNoiseFx = 848.0/(2.0*tan(87.0*3.141592653589793/360.0));
  input Real baseline = 0.05;
  output Real valid; output Real rejectionReason; output Real validCount; output Real invalidCount;
  output Real relativeCovariance[6,6]; output Real observationCovariance[6,6];
  output Real normalMatrix[6,6]; output Real noiseMatrix[6,6];
  output Real observationJacobian[6,6]; output Real minimumScaledPivot;
equation
  (valid,rejectionReason,validCount,invalidCount,relativeCovariance,observationCovariance,
    normalMatrix,noiseMatrix,observationJacobian,minimumScaledPivot) = RGBDRegistrationSandwich(
    referencePoint,currentPoint,pairEnabled,activeCount,registrationAccepted,currentFromReference,translation,
    referenceBodyRotation,opticalToBody,cameraOriginBody,referenceRgbFocal,currentRgbFocal,
    referenceNoiseFx,currentNoiseFx,baseline,localizationSigma,disparitySigma,depthInflation,coordinateLimit,minimumPivot);
end RGBDRegistrationUncertainty;
