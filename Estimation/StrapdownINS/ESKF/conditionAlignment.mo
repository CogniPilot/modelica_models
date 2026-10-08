within Estimation.StrapdownINS.ESKF;

function conditionAlignment
  "Condition the initial tangent covariance on the gravity vector used for alignment"
  input State prior;
  input Real specificForceBodyFlu_m_s2[3];
  input Real specificForceCovariance[3, 3];
  input Real gravityWorldEnu_m_s2[3];
  input Real innovationGate;
  input Boolean useSemiDirectBias = false;
  output State conditioned;
protected
  Real gravityBodyFlu[3];
  Real residual[3];
  Real H[3, TangentLength];
  Boolean accepted;
  Integer outcome;
  Real nis;
algorithm
  gravityBodyFlu := -transpose(LieGroups.SO3.Quat.to_DCM(
    prior.quaternionWorldBody)) * gravityWorldEnu_m_s2;
  residual := specificForceBodyFlu_m_s2 - gravityBodyFlu
    - prior.accelerometerBiasBodyFlu_m_s2;
  H := cat(2, zeros(3, 6), LieGroups.SO3.Quat.wedge(gravityBodyFlu),
    zeros(3, 3), identity(3));
  (conditioned, accepted, outcome, nis) := correctLinear(
    prior, residual, H, specificForceCovariance, innovationGate, zeros(3), zeros(TangentLength, size(residual, 1)),
        false, useSemiDirectBias);
end conditionAlignment;
