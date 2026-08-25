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
