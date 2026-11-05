within LinearAlgebra;

function transformCovariance "Covariance under a linear map"
  input Real transform[:, :];
  input Real covariance[size(transform, 2), size(transform, 2)];
  output Real transformedCovariance[size(transform, 1), size(transform, 1)];
algorithm
  transformedCovariance := SymmetricMatrix.transformDense(transform, covariance);
  annotation(Inline=false);
end transformCovariance;
