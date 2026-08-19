within Estimation.StrapdownINS.UKF;

function injectVector
  "Right-inject a local tangent into a flattened nominal state"
  input Estimation.StrapdownINS.UKF.NominalVector nominal;
  input Estimation.StrapdownINS.UKF.TangentVector correction;
  output Estimation.StrapdownINS.UKF.NominalVector corrected;
protected
  Real correctedExtendedPose[10];
algorithm
  correctedExtendedPose := LieGroups.SE23.Quat.product(
    nominal[1:10], LieGroups.SE23.Quat.exp_map(correction[1:9]));
  corrected := cat(1,
    correctedExtendedPose[1:6],
    LieGroups.SO3.Quat.normalize(correctedExtendedPose[7:10]),
    nominal[11:13] + correction[10:12],
    nominal[14:16] + correction[13:15]);
end injectVector;
