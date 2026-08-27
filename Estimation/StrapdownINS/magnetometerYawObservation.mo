within Estimation.StrapdownINS;

function magnetometerYawObservation
  "Tilt-compensate a raw magnetometer into a yaw-only observation"
  input Real quaternionWorldBody[4];
  input Real magneticFieldBodyFlu_T[3];
  input Real covarianceBody_T2[3, 3];
  input Real magneticFieldWorldEnu_T[3];
  output Real heading_rad(unit = "rad");
  output Real variance_rad2(unit = "rad2");
  output Boolean accepted;
  output Real yawSensitivityBodyFlu[3]
    "Derivative of the reported heading with respect to a local right
     attitude error, keeping only the rotation about the estimated
     vertical: the part a yaw-only fusion mode acts on";
  output Real tiltSensitivityBodyFlu[3]
    "The remaining derivative, contributed by levelling the field with the
     ESTIMATED roll and pitch. It is not sensor noise and not zero: a
     tilt error rotates the levelled field and moves the reported heading
     by roughly tan(inclination) times that error";
protected
  Real eulerYpr[3];
  Real levelingQuaternion[4];
  Real levelingRotation[3, 3];
  Real fieldLeveled[3];
  Real covarianceLeveled[3, 3];
  Real horizontalMagnitudeSquared;
  Real referenceHorizontalMagnitudeSquared;
  Real headingGradient[3];
  Real declination;
  Real levelledHorizontalGradient[3];
algorithm
  eulerYpr := LieGroups.SO3.EulerB321.from_Quat(quaternionWorldBody);
  levelingQuaternion := LieGroups.SO3.EulerB321.to_Quat(
    {0.0, eulerYpr[2], eulerYpr[3]});
  levelingRotation := LieGroups.SO3.Quat.to_DCM(levelingQuaternion);
  fieldLeveled := levelingRotation * magneticFieldBodyFlu_T;
  covarianceLeveled := levelingRotation * covarianceBody_T2
    * transpose(levelingRotation);
  horizontalMagnitudeSquared := fieldLeveled[1] * fieldLeveled[1]
    + fieldLeveled[2] * fieldLeveled[2];
  referenceHorizontalMagnitudeSquared :=
    magneticFieldWorldEnu_T[1] * magneticFieldWorldEnu_T[1]
      + magneticFieldWorldEnu_T[2] * magneticFieldWorldEnu_T[2];
  declination := atan2(magneticFieldWorldEnu_T[2],
    magneticFieldWorldEnu_T[1]);
  heading_rad := MathUtilities.wrapAngle(declination
    - atan2(fieldLeveled[2], fieldLeveled[1]));
  headingGradient := if horizontalMagnitudeSquared > 1.0e-20 then
    {fieldLeveled[2] / horizontalMagnitudeSquared,
     -fieldLeveled[1] / horizontalMagnitudeSquared, 0.0}
    else zeros(3);
  variance_rad2 := headingGradient * covarianceLeveled * headingGradient;
  // EXACT SENSITIVITY OF THE REPORTED HEADING TO THE ATTITUDE ERROR.
  //
  // The reported heading is decl - atan2(fL2, fL1) with fL = R_level m_b,
  // and R_level = Rz(-yaw_hat) R_hat is built from the ESTIMATED roll and
  // pitch. Writing R_true = R_hat Exp(eps) for the local right error and
  // linearizing, fL = Rz(-yaw_hat) Exp(-R_hat eps) B_w, so
  //
  //   d(heading) = -eps' * R_level' * (fL3*fL1/H2, fL3*fL2/H2, -1)
  //
  // with H2 the squared horizontal magnitude. The third component is the
  // rotation about the estimated vertical -- the yaw signal the fusion
  // mode wants. The first two are the levelling transfer: they are
  // proportional to fL3/|fLh|, i.e. the tangent of the magnetic
  // inclination, which is about 2.4 at the RDD2 test site. Reporting only
  // the sensor variance and a yaw-only row hides that term from BOTH the
  // innovation covariance and the covariance update, which is why the
  // heading channel reads optimistic while its NIS looks nearly calibrated.
  yawSensitivityBodyFlu := levelingRotation[3, :];
  levelledHorizontalGradient := if horizontalMagnitudeSquared > 1.0e-20 then
    {-fieldLeveled[3] * fieldLeveled[1] / horizontalMagnitudeSquared,
     -fieldLeveled[3] * fieldLeveled[2] / horizontalMagnitudeSquared,
     0.0} else zeros(3);
  tiltSensitivityBodyFlu :=
    transpose(levelingRotation) * levelledHorizontalGradient;
  accepted := horizontalMagnitudeSquared > 1.0e-20
    and referenceHorizontalMagnitudeSquared > 1.0e-20
    and variance_rad2 > 0.0
    and variance_rad2 < Estimation.StrapdownINS.ESKF.FiniteMagnitudeLimit;
  for index in 1:3 loop
    accepted := accepted
      and abs(magneticFieldBodyFlu_T[index])
        < Estimation.StrapdownINS.ESKF.FiniteMagnitudeLimit
      and abs(magneticFieldWorldEnu_T[index])
        < Estimation.StrapdownINS.ESKF.FiniteMagnitudeLimit;
  end for;
end magnetometerYawObservation;
