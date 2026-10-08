within Estimation.StrapdownINS.ESKF;

function reseed
  "Re-seed position and velocity from a fresh anchor sample after a sustained
   rejection, restoring their covariance blocks to the initial variances"
  input State predicted
    "Predicted state whose attitude and bias blocks are kept";
  input Real positionWorldEnu_m[3]
    "Position taken directly from the anchor sample";
  input Real velocityWorldEnu_m_s[3]
    "Velocity taken from the anchor sample when it reports one, otherwise the
     predicted velocity carried through by the caller";
  input Estimation.StrapdownINS.InitialVariances initialVariances;
  output State reseededState;
protected
  Covariance restored;
  Covariance root;
algorithm
  // Zero the position and velocity rows and columns entirely, then rebuild
  // their diagonal at the initial variances. This restores the two blocks and
  // zeroes every cross-covariance they held with the attitude and bias
  // blocks in one pass; the lower-right 9x9 attitude and bias block is kept
  // exactly as the caller's prediction produced it.
  restored := diagonal(cat(1, initialVariances.position_m2,
    initialVariances.velocity_m2_s2, zeros(TangentLength - 6)));
  restored[7:TangentLength, 7:TangentLength] :=
    predicted.covariance[7:TangentLength, 7:TangentLength];
  reseededState := State(
    positionWorldEnu_m=positionWorldEnu_m,
    velocityWorldEnu_m_s=velocityWorldEnu_m_s,
    quaternionWorldBody=predicted.quaternionWorldBody,
    gyroscopeBiasBodyFlu_rad_s=predicted.gyroscopeBiasBodyFlu_rad_s,
    accelerometerBiasBodyFlu_m_s2=predicted.accelerometerBiasBodyFlu_m_s2,
    covariance=restored,
    covarianceRoot=predicted.covarianceRoot,
    barometerBiasCrossCovariance=cat(1, zeros(6),
      predicted.barometerBiasCrossCovariance[7:TangentLength]),
    barometerBias_m=predicted.barometerBias_m,
    barometerBiasVariance_m2=predicted.barometerBiasVariance_m2,
    useJointBarometerBias=predicted.useJointBarometerBias,
    useSquareRootCovariance=predicted.useSquareRootCovariance);
  if predicted.useSquareRootCovariance then
    root := diagonal(cat(1,
      {sqrt(initialVariances.position_m2[axis]) for axis in 1:3},
      {sqrt(initialVariances.velocity_m2_s2[axis]) for axis in 1:3},
      zeros(TangentLength - 6)));
    root[7:TangentLength, 7:TangentLength] := LinearAlgebra.covarianceRoot(
      predicted.covarianceRoot[7:TangentLength, :]);
    reseededState := withCovarianceRoot(reseededState, root,
      reseededState.barometerBiasCrossCovariance);
  end if;
end reseed;
