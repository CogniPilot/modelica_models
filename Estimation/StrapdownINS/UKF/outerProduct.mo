within Estimation.StrapdownINS.UKF;

function outerProduct "Fixed-size tangent outer product"
  input Real left[TangentLength];
  input Real right[TangentLength];
  output Real product[TangentLength, TangentLength];
algorithm
  for row in 1:TangentLength loop
    for column in 1:TangentLength loop
      product[row, column] := left[row] * right[column];
    end for;
  end for;
end outerProduct;
