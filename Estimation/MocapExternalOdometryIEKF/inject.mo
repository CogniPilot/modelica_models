within Estimation.MocapExternalOdometryIEKF;
function inject "Inject a tangent correction into the nominal state"
  input Estimation.MocapExternalOdometryIEKF.NominalState state;
  input TangentVector correction
    "{attitude error, velocity error, position error, angular velocity error}";
  output Estimation.MocapExternalOdometryIEKF.NominalState corrected;
protected
  Real attitudeDelta[4];
algorithm
  attitudeDelta := LieGroups.SO3.Quat.exp_map(correction[1:3]);
  corrected.attitude := LieGroups.SO3.Quat.normalize(
    LieGroups.SO3.Quat.product(state.attitude, attitudeDelta));
  corrected.velocity := state.velocity + correction[4:6];
  corrected.position := state.position + correction[7:9];
  corrected.angularVelocity := state.angularVelocity + correction[10:12];
end inject;
