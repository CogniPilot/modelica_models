within Estimation.StrapdownINS.ESKF;

function conjugateReset
  "Conjugate a covariance by the block-diagonal reset Jacobian diag(J, I)"
  input Real rotationJacobian[9, 9]
    "SE_2(3) right Jacobian J of the applied correction";
  input Covariance covariance;
  output Covariance conjugated;
protected
  Real rotationJacobianTransposed[9, 9];
  Real rotated[9, TangentLength] "J * covariance[1:9, :]";
algorithm
  rotationJacobianTransposed := transpose(rotationJacobian);
  rotated := rotationJacobian * covariance[1:9, :];
  conjugated[1:9, 1:9] := rotated[:, 1:9] * rotationJacobianTransposed;
  conjugated[1:9, 10:TangentLength] := rotated[:, 10:TangentLength];
  conjugated[10:TangentLength, 1:9] :=
    covariance[10:TangentLength, 1:9] * rotationJacobianTransposed;
  conjugated[10:TangentLength, 10:TangentLength] :=
    covariance[10:TangentLength, 10:TangentLength];
end conjugateReset;
