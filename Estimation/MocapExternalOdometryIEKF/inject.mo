within Estimation.MocapExternalOdometryIEKF;
function inject "Inject a 12D tangent correction into the 13D nominal state"
  input Real state[13] "{attitude, velocity, position, angular velocity}";
  input Real correction[12]
    "{attitude error, velocity error, position error, angular velocity error}";
  output Real state_next[13];
protected
  Real attitude_delta[4];
  Real attitude_next[4];
algorithm
  attitude_delta := LieGroups.SO3.Quat.exp_map(correction[1:3]);
  attitude_next :=
    LieGroups.SO3.Quat.product(state[1:4], attitude_delta);
  attitude_next := LieGroups.SO3.Quat.normalize(attitude_next);

  for i in 1:4 loop
    state_next[i] := attitude_next[i];
  end for;
  for i in 1:3 loop
    state_next[i + 4] := state[i + 4] + correction[i + 3];
    state_next[i + 7] := state[i + 7] + correction[i + 6];
    state_next[i + 10] := state[i + 10] + correction[i + 9];
  end for;
end inject;
