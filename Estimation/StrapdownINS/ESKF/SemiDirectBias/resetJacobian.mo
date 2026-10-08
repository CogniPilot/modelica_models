within Estimation.StrapdownINS.ESKF.SemiDirectBias;

function resetJacobian
  "Reset the full pose-bias covariance after semi-direct injection"
  input TangentVector correction;
  output Real reset[TangentLength, TangentLength];
protected
  constant Integer biasOrder[6] = {4, 5, 6, 1, 2, 3};
  Real poseAlgebra[6, 6];
  Real biasJacobian[6, 6];
  Real powerVector[6];
  Real powerDerivative[6, 6];
  Real biasCoupling[6, 6];
  Real coefficient;
algorithm
  poseAlgebra := LieGroups.SE3.Quat.small_adjoint(correction[4:9]);
  biasJacobian := LieGroups.SE3.Quat.right_jacobian(correction[4:9]);
  powerVector := cat(1, correction[13:15], correction[10:12]);
  powerDerivative := zeros(6, 6);
  biasCoupling := zeros(6, 6);
  coefficient := 1.0;
  for seriesOrder in 1:12 loop
    powerDerivative := poseAlgebra * powerDerivative
      - LieGroups.SE3.Quat.small_adjoint(powerVector);
    powerVector := poseAlgebra * powerVector;
    coefficient := -coefficient / (seriesOrder + 1.0);
    biasCoupling := biasCoupling + coefficient * powerDerivative;
  end for;
  reset := zeros(TangentLength, TangentLength);
  reset[1:9, 1:9] := LieGroups.SE23.Quat.right_jacobian(correction[1:9]);
  reset[10:15, 4:9] := biasCoupling[biasOrder, :];
  reset[10:15, 10:15] := biasJacobian[biasOrder, biasOrder];
  annotation(Inline=false);
end resetJacobian;
