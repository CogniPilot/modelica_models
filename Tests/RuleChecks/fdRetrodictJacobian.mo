within Tests.RuleChecks;
function fdRetrodictJacobian
  "Central difference of retrodict in the ESKF tangent, right-trivialized pose"
  input Estimation.StrapdownINS.ESKF.State current;
  input Real angularVelocityMeasuredBodyFlu_rad_s[3];
  input Real specificForceMeasuredBodyFlu_m_s2[3];
  input Real gravityWorldEnu_m_s2[3];
  input Real age_s(unit = "s");
  input Real step "Central-difference step";
  output Real J[15, 15];
protected
  Real baseVector[16];
  Real baseInverse[10];
  Real baseBiases[6];
  Real basis[15, 15];
  Real forwardError[15];
  Real backwardError[15];
algorithm
  baseVector := Estimation.StrapdownINS.ESKF.retrodict(current,
    angularVelocityMeasuredBodyFlu_rad_s,
    specificForceMeasuredBodyFlu_m_s2, gravityWorldEnu_m_s2, age_s);
  baseInverse := LieGroups.SE23.Quat.inverse(baseVector[1:10]);
  baseBiases := baseVector[11:16];
  basis := identity(15);
  for j in 1:15 loop
    forwardError := Tests.RuleChecks.retrodictTangentError(current,
      angularVelocityMeasuredBodyFlu_rad_s,
      specificForceMeasuredBodyFlu_m_s2, gravityWorldEnu_m_s2, age_s,
      step * basis[:, j], baseInverse, baseBiases);
    backwardError := Tests.RuleChecks.retrodictTangentError(current,
      angularVelocityMeasuredBodyFlu_rad_s,
      specificForceMeasuredBodyFlu_m_s2, gravityWorldEnu_m_s2, age_s,
      -step * basis[:, j], baseInverse, baseBiases);
    J[:, j] := (forwardError - backwardError) / (2.0 * step);
  end for;
end fdRetrodictJacobian;
