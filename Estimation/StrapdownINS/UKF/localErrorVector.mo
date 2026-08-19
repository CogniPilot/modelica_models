within Estimation.StrapdownINS.UKF;

function localErrorVector
  "Body-local tangent carrying one flattened nominal state to another"
  input Estimation.StrapdownINS.UKF.NominalVector reference;
  input Estimation.StrapdownINS.UKF.NominalVector target;
  output Estimation.StrapdownINS.UKF.TangentVector error;
protected
  Real groupError[10];
algorithm
  groupError := LieGroups.SE23.Quat.product(
    LieGroups.SE23.Quat.inverse(reference[1:10]), target[1:10]);
  error := cat(1, LieGroups.SE23.Quat.log_map(groupError),
    target[11:13] - reference[11:13],
    target[14:16] - reference[14:16]);
end localErrorVector;
