within Estimation.StrapdownINS.ESKF;

function inflateStateCovariance "Inflate covariance in its selected representation"
  input State previous;
  input VarianceLimits limits;
  input Real dt;
  input Real timeConstant_s;
  output State inflated;
protected
  Covariance addendRoot;
  Covariance inflatedCovariance;
algorithm
  inflatedCovariance := inflateCovariance(previous.covariance, limits,
    dt, timeConstant_s);
  if previous.useSquareRootCovariance then
    addendRoot := zeros(TangentLength, TangentLength);
    for row in 1:TangentLength loop
      addendRoot[row, row] := sqrt(max(inflatedCovariance[row, row]
        - previous.covariance[row, row], 0.0));
    end for;
    inflated := withCovarianceRoot(previous, LinearAlgebra.covarianceRoot(
      cat(2, previous.covarianceRoot, addendRoot)));
  else
    inflated := withDenseCovariance(previous, inflatedCovariance);
  end if;
end inflateStateCovariance;
