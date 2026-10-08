within SLAM.Localization;
// Review composition: full calibrated visual frontend plus its registration
// noise model. Compile together with RGBDVisualObservation, RGBDFeatureMatching,
// RigidPointRegistration, RGBDRelativePose and RGBDRegistrationUncertainty.
// This source has not yet passed a connected source-issued numerical gate.
model RGBDVisualRelativeObservation
  import RGBDRegistrationSandwich = SLAM.Localization.RGBDRegistrationSandwich;
  import RGBDVisualObservation = SLAM.Localization.RGBDVisualObservation;

  extends RGBDVisualObservation;
  // Retain these with the reference image rather than substituting the current
  // image calibration. The noise model assumes a common stereo baseline and
  // disparity sigma for the pair; changing those requires a new noise model.
  input Real referenceRgbFocal[2];
  input Real referenceNoiseReferenceFx;
  parameter Real localizationSigma = 0.5;
  parameter Real depthInflation = 1.0;
  parameter Real uncertaintyCoordinateLimit = 100.0;
  parameter Real uncertaintyMinimumPivot = 1e-10;
  output Real currentFromReference[3,3];
  output Real currentFromReferenceTranslation[3];
  output Real relativeCovariance[6,6]
    "Optical translation and optical left-angle perturbation; use with Schmidt update";
  output Real conditionalObservationCovariance[6,6]
    "Conditional on retained reference pose; not independent absolute noise";
  output Real relativeValid;
  output Real uncertaintyRejectionReason;
  output Real uncertaintyValidCount;
  output Real uncertaintyInvalidCount;
protected
  // Registration and uncertainty consume exactly the same complete sparse
  // pair domain. matching.count is not a prefix extent.
  Real uncertaintyValid;
  Real normalMatrix[6,6];
  Real noiseMatrix[6,6];
  Real observationJacobian[6,6];
  Real minimumScaledPivot;
equation
  // Call the existing function so per-frame calibrated noise remains a runtime
  // input; it must not initialize a parameter from a varying input binding.
  (uncertaintyValid,uncertaintyRejectionReason,uncertaintyValidCount,
    uncertaintyInvalidCount,relativeCovariance,conditionalObservationCovariance,
    normalMatrix,noiseMatrix,observationJacobian,minimumScaledPivot) =
    RGBDRegistrationSandwich(matching.sourcePoint,matching.targetPoint,
      matching.pairEnabled,featureCapacity,valid,registration.rotation,
      registration.translation,referenceBodyRotation,opticalToBody,
      cameraOriginBody,referenceRgbFocal,{rgbCalibration[1],rgbCalibration[2]},
      referenceNoiseReferenceFx,noiseReferenceFx,baseline,localizationSigma,
      disparityNoise,depthInflation,uncertaintyCoordinateLimit,uncertaintyMinimumPivot);
  currentFromReference = registration.rotation;
  currentFromReferenceTranslation = registration.translation;
  relativeValid = if noEvent(valid > 0.5 and uncertaintyValid > 0.5) then 1.0 else 0.0;
end RGBDVisualRelativeObservation;
