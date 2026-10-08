within Estimation.StrapdownINS.ESKF;

function limitStateCovariance "Bound covariance without changing its representation"
  input State previous;
  input VarianceLimits limits;
  output State limited;
protected
  Real bound[TangentLength];
  Covariance root;
  Real variance;
  Real rescale[TangentLength];
algorithm
  bound := cat(1, limits.position_m2, limits.velocity_m2_s2,
    limits.attitude_rad2, limits.gyroscopeBias_rad2_s2,
    limits.accelerometerBias_m2_s4);
  rescale := ones(TangentLength);
  if previous.useSquareRootCovariance then
    root := previous.covarianceRoot;
    for row in 1:TangentLength loop
      variance := root[row, :] * root[row, :];
      if variance > bound[row] then
        rescale[row] := sqrt(bound[row] / variance);
        root[row, :] := rescale[row] * root[row, :];
      end if;
    end for;
    limited := withCovarianceRoot(previous, root,
      rescale .* previous.barometerBiasCrossCovariance);
  else
    for row in 1:TangentLength loop
      variance := previous.covariance[row, row];
      rescale[row] := if not abs(variance) < FiniteMagnitudeLimit then 0.0
        elseif variance > bound[row] then sqrt(bound[row] / variance) else 1.0;
    end for;
    limited := withDenseCovariance(previous,
      limitCovariance(previous.covariance, limits),
      rescale .* previous.barometerBiasCrossCovariance);
  end if;
end limitStateCovariance;
