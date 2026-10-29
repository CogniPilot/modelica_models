within Estimation.StrapdownINS.ESKF;

function conjugateReset
  "Conjugate a covariance by the block-diagonal reset Jacobian diag(J, I)"
  input Real rotationJacobian[9, 9]
    "SE_2(3) right Jacobian J of the applied correction";
  input Covariance covariance;
  output Covariance conjugated;
protected
  Real rotationJacobianTransposed[9, 9];
  Real packedConjugated[div(TangentLength * (TangentLength + 1), 2)];
  Real value;
  Real rotated[9, TangentLength] "J * covariance[1:9, :]";
algorithm
  value := 0.0;
  packedConjugated := zeros(div(TangentLength * (TangentLength + 1), 2));
  rotationJacobianTransposed := transpose(rotationJacobian);
  rotated := rotationJacobian * covariance[1:9, :];
  if LinearAlgebra.SymmetricMatrix.isSymmetric(covariance) then
    for i in 1:TangentLength loop
      for j in i:TangentLength loop
        if j <= 9 then
          value := 0.0;
          for k in 1:9 loop
            value := value + rotated[i, k] * rotationJacobian[j, k];
          end for;
        elseif i <= 9 then
          value := rotated[i, j];
        else
          value := covariance[i, j];
        end if;
        packedConjugated[LinearAlgebra.SymmetricMatrix.index(i,j,TangentLength)] := value;
      end for;
    end for;
    conjugated := LinearAlgebra.SymmetricMatrix.unpack(TangentLength,packedConjugated);
  else
    conjugated[1:9, 1:9] := rotated[:, 1:9] * rotationJacobianTransposed;
    conjugated[1:9, 10:TangentLength] := rotated[:, 10:TangentLength];
    conjugated[10:TangentLength, 1:9] :=
      covariance[10:TangentLength, 1:9] * rotationJacobianTransposed;
    conjugated[10:TangentLength, 10:TangentLength] :=
      covariance[10:TangentLength, 10:TangentLength];
  end if;
end conjugateReset;
