within Estimation.StrapdownINS.ESKF;

function reseed
  "Re-seed position and velocity from a fresh anchor sample after a sustained
   rejection, restoring their covariance blocks to the initial variances"
  input Estimation.StrapdownINS.ESKF.State predicted
    "Predicted state whose attitude and bias blocks are kept";
  input Real positionWorldEnu_m[3]
    "Position taken directly from the anchor sample";
  input Real velocityWorldEnu_m_s[3]
    "Velocity taken from the anchor sample when it reports one, otherwise the
     predicted velocity carried through by the caller";
  input Estimation.StrapdownINS.InitialVariances initialVariances;
  output Estimation.StrapdownINS.ESKF.State reseededState;
protected
  Estimation.StrapdownINS.ESKF.Covariance restored;
algorithm
  // Zero the position and velocity rows and columns entirely, then rebuild
  // their diagonal at the initial variances. This restores the two blocks and
  // zeroes every cross-covariance they held with the attitude and bias
  // blocks in one pass; the lower-right 9x9 attitude and bias block is kept
  // exactly as the caller's prediction produced it.
  restored := predicted.covariance;
  for i in 1:6 loop
    for j in 1:15 loop
      restored[i, j] := 0.0;
      restored[j, i] := 0.0;
    end for;
  end for;
  restored[1, 1] := initialVariances.position_m2[1];
  restored[2, 2] := initialVariances.position_m2[2];
  restored[3, 3] := initialVariances.position_m2[3];
  restored[4, 4] := initialVariances.velocity_m2_s2[1];
  restored[5, 5] := initialVariances.velocity_m2_s2[2];
  restored[6, 6] := initialVariances.velocity_m2_s2[3];
  reseededState := Estimation.StrapdownINS.ESKF.State(
    positionWorldEnu_m=positionWorldEnu_m,
    velocityWorldEnu_m_s=velocityWorldEnu_m_s,
    quaternionWorldBody=predicted.quaternionWorldBody,
    gyroscopeBiasBodyFlu_rad_s=predicted.gyroscopeBiasBodyFlu_rad_s,
    accelerometerBiasBodyFlu_m_s2=predicted.accelerometerBiasBodyFlu_m_s2,
    covariance=restored);
end reseed;
