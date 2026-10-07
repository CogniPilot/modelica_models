within Estimation.StrapdownINS.UKF;

function outerProduct "Fixed-size tangent outer product"
  input Real left[TangentLength];
  input Real right[TangentLength];
  output Real product[TangentLength, TangentLength];
algorithm
  product := transpose({left}) * {right};
end outerProduct;
