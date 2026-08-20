within Estimation.StrapdownINS;

function initialAlignmentQuaternion
  "Level from stationary specific force and align yaw from raw magnetic field"
  input Real specificForceBodyFlu_m_s2[3];
  input Real magneticFieldBodyFlu_T[3];
  input Real magneticFieldWorldEnu_T[3];
  input Real fallbackQuaternionWorldBody[4] = {1.0, 0.0, 0.0, 0.0};
  output Real quaternionWorldBody[4];
  output Boolean accepted;
protected
  Real magnitude;
  Real roll;
  Real pitch;
  Real levelingQuaternion[4];
  Real fieldLeveled[3];
  Real heading;
  Real horizontalMagnitudeSquared;
  Real referenceHorizontalMagnitudeSquared;
algorithm
  magnitude := sqrt(specificForceBodyFlu_m_s2
    * specificForceBodyFlu_m_s2);
  accepted := magnitude > 1.0e-3
    and magnitude < Estimation.StrapdownINS.ESKF.FiniteMagnitudeLimit;
  // For R_world_body = Rz(yaw) Ry(pitch) Rx(roll), a stationary
  // accelerometer observes R' * {0,0,g} =
  // {-sin(pitch), cos(pitch)sin(roll), cos(pitch)cos(roll)} * g.
  pitch := asin(MathUtilities.clip(
    -specificForceBodyFlu_m_s2[1] / max(magnitude, 1.0e-3), -1.0, 1.0));
  roll := atan2(specificForceBodyFlu_m_s2[2],
    specificForceBodyFlu_m_s2[3]);
  levelingQuaternion := LieGroups.SO3.EulerB321.to_Quat(
    {0.0, pitch, roll});
  fieldLeveled := LieGroups.SO3.Quat.to_DCM(levelingQuaternion)
    * magneticFieldBodyFlu_T;
  horizontalMagnitudeSquared := fieldLeveled[1] * fieldLeveled[1]
    + fieldLeveled[2] * fieldLeveled[2];
  referenceHorizontalMagnitudeSquared :=
    magneticFieldWorldEnu_T[1] * magneticFieldWorldEnu_T[1]
      + magneticFieldWorldEnu_T[2] * magneticFieldWorldEnu_T[2];
  accepted := accepted and horizontalMagnitudeSquared > 1.0e-20
    and referenceHorizontalMagnitudeSquared > 1.0e-20;
  heading := MathUtilities.wrapAngle(
    atan2(magneticFieldWorldEnu_T[2], magneticFieldWorldEnu_T[1])
      - atan2(fieldLeveled[2], fieldLeveled[1]));
  if accepted then
    quaternionWorldBody := LieGroups.SO3.EulerB321.to_Quat(
      {heading, pitch, roll});
  else
    quaternionWorldBody := LieGroups.SO3.Quat.normalize(
      fallbackQuaternionWorldBody);
  end if;
end initialAlignmentQuaternion;
