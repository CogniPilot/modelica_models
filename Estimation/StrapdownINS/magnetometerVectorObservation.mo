within Estimation.StrapdownINS;

function magnetometerVectorObservation
  "Scale a magnetic-vector residual and its symmetric SO(3) output linearization"
  input Real quaternionWorldBody[4];
  input Real magneticFieldBodyFlu_T[3];
  input Real covarianceBody_T2[3, 3];
  input Real magneticFieldWorldEnu_T[3];
  output Real residual[3];
  output Real attitudeJacobian[3, 3];
  output Real covariance[3, 3];
  output Boolean accepted;
protected
  Real rotationWorldBody[3, 3];
  Real fieldMagnitudeSquared;
  Real fieldScale;
  Real predictedField[3];
  Real measuredField[3];
  Real inverseCovariance[3, 3];
  Boolean covarianceUsable;
algorithm
  rotationWorldBody := LieGroups.SO3.Quat.to_DCM(quaternionWorldBody);
  fieldMagnitudeSquared := magneticFieldWorldEnu_T * magneticFieldWorldEnu_T;
  fieldScale := sqrt(max(fieldMagnitudeSquared, 1.0e-20));
  predictedField := transpose(rotationWorldBody)
    * magneticFieldWorldEnu_T / fieldScale;
  measuredField := magneticFieldBodyFlu_T / fieldScale;
  residual := measuredField - predictedField;
  attitudeJacobian := LieGroups.SO3.Quat.wedge(
    0.5 * (predictedField + measuredField));
  covariance := covarianceBody_T2 / fieldScale^2;
  (inverseCovariance, covarianceUsable) := LinearAlgebra.solveSPD(
    covariance, identity(3));
  accepted := covarianceUsable and fieldMagnitudeSquared > 1.0e-20
    and fieldMagnitudeSquared < ESKF.FiniteMagnitudeLimit;
  for axis in 1:3 loop
    accepted := accepted
      and abs(magneticFieldBodyFlu_T[axis]) < ESKF.FiniteMagnitudeLimit
      and abs(magneticFieldWorldEnu_T[axis]) < ESKF.FiniteMagnitudeLimit;
  end for;
end magnetometerVectorObservation;
