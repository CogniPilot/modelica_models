within SLAM.LoopClosure;
// Convert an already verified optical registration into the product residual
// used by ModelicaPoseGraph. This component does not verify correspondences,
// admit graph edges, or resolve correlations between reused images.
// Dependencies: RGBDRegistrationUncertainty.mo (rotation/SPD helpers).
function RGBDOpticalToBodyEdge
  import RGBDUncertaintyInverse6 = SLAM.Localization.RGBDUncertaintyInverse6;
  import RGBDUncertaintyProper = SLAM.Localization.RGBDUncertaintyProper;
  import RGBDUncertaintySkew = SLAM.Localization.RGBDUncertaintySkew;

  input Real currentFromReference[3,3];
  input Real opticalTranslation[3];
  input Real opticalCovariance[6,6]
    "Independent translation increment, then left optical angle increment";
  input Real referenceOpticalToBody[3,3];
  input Real currentOpticalToBody[3,3];
  input Real referenceCameraOrigin[3];
  input Real currentCameraOrigin[3];
  input Boolean requested;
  input Real coordinateLimit;
  input Real minimumPivot;
  output Real measuredRotation[3,3] "Current body coordinates to reference body";
  output Real measuredTranslation[3] "Current body origin in reference body";
  output Real residualJacobian[6,6];
  output Real residualCovariance[6,6];
  output Real information[6,6];
  output Boolean valid;
  output Integer rejectionReason;
  output Real minimumScaledPivot;
protected
  Boolean configuration; Boolean geometry; Boolean opticalValid; Boolean edgeValid;
  Real rotationCandidate[3,3]; Real translationCandidate[3];
  Real opticalToReferenceBody[3,3]; Real arm[3]; Real J[6,6];
  Real covarianceCandidate[6,6]; Real inverseCandidate[6,6];
  Real unusedInverse[6,6]; Real opticalPivot;
algorithm
  measuredRotation := identity(3); measuredTranslation := zeros(3);
  residualJacobian := zeros(6,6); residualCovariance := zeros(6,6);
  information := zeros(6,6); valid := false; rejectionReason := 1;
  minimumScaledPivot := 0.0;
  // Disabled payload is not read as a measurement.
  if requested then
    configuration := coordinateLimit > 0.0 and coordinateLimit <= 1e4
      and minimumPivot > 0.0 and minimumPivot < 1.0;
    rejectionReason := 2;
    if configuration then
      geometry := RGBDUncertaintyProper(currentFromReference)
        and RGBDUncertaintyProper(referenceOpticalToBody)
        and RGBDUncertaintyProper(currentOpticalToBody);
      for axis in 1:3 loop
        geometry := geometry and abs(opticalTranslation[axis]) <= coordinateLimit
          and abs(referenceCameraOrigin[axis]) <= coordinateLimit
          and abs(currentCameraOrigin[axis]) <= coordinateLimit;
      end for;
      rejectionReason := 3;
      if geometry then
        (unusedInverse,opticalValid,opticalPivot) :=
          RGBDUncertaintyInverse6(opticalCovariance,minimumPivot);
        rejectionReason := 4;
        if opticalValid then
          opticalToReferenceBody := referenceOpticalToBody*transpose(currentFromReference);
          rotationCandidate := opticalToReferenceBody*transpose(currentOpticalToBody);
          arm := opticalTranslation+transpose(currentOpticalToBody)*currentCameraOrigin;
          translationCandidate := referenceCameraOrigin-opticalToReferenceBody*arm;
          // At the nominal edge, r = [Ri'*(pj-pi)-u, Log(D'*Ri'*Rj)].
          // Perturb C as Exp(dtheta)*C and t independently as t+dt.
          // Differentiating this residual gives [A, A*skew(arm); 0, Ecurrent].
          // It is not the coupled SE(3) logarithm/adjoint covariance convention.
          J := zeros(6,6);
          J[1:3,1:3] := opticalToReferenceBody;
          J[1:3,4:6] := opticalToReferenceBody*RGBDUncertaintySkew(arm);
          J[4:6,4:6] := currentOpticalToBody;
          // Only roundoff asymmetry within the SPD helper's tolerance is
          // averaged. No damping or covariance floor invents an accepted edge.
          covarianceCandidate := J*((opticalCovariance+transpose(opticalCovariance))/2.0)*transpose(J);
          covarianceCandidate := (covarianceCandidate+transpose(covarianceCandidate))/2.0;
          (inverseCandidate,edgeValid,minimumScaledPivot) :=
            RGBDUncertaintyInverse6(covarianceCandidate,minimumPivot);
          rejectionReason := 5;
          if edgeValid then
            measuredRotation := rotationCandidate;
            measuredTranslation := translationCandidate;
            residualJacobian := J;
            residualCovariance := covarianceCandidate;
            information := (inverseCandidate+transpose(inverseCandidate))/2.0;
            valid := true; rejectionReason := 0;
          end if;
        end if;
      end if;
    end if;
  end if;
end RGBDOpticalToBodyEdge;
