within Estimation.StrapdownINS.ESKF;

function limitStateCovariance "Bound covariance without changing its representation"
  input State previous;
  input VarianceLimits limits;
  output State limited;
protected
  Real bound[TangentLength];
  Covariance root;
  Real variance;
algorithm
  if previous.useSquareRootCovariance then
    bound := cat(1, limits.position_m2, limits.velocity_m2_s2,
      limits.attitude_rad2, limits.gyroscopeBias_rad2_s2,
      limits.accelerometerBias_m2_s4);
    root := previous.covarianceRoot;
    for row in 1:TangentLength loop
      variance := root[row, :] * root[row, :];
      if variance > bound[row] then
        root[row, :] := sqrt(bound[row] / variance) * root[row, :];
      end if;
    end for;
    limited := withCovarianceRoot(previous, root);
  else
    limited := withDenseCovariance(previous,
      limitCovariance(previous.covariance, limits));
  end if;
end limitStateCovariance;
