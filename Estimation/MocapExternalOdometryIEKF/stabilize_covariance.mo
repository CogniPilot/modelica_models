within Estimation.MocapExternalOdometryIEKF;
function stabilize_covariance "Symmetrize covariance and floor diagonal variances"
  input Real covariance[12, 12];
  output Real stabilized[12, 12];
protected
  Real value;
  constant Real minVariance = 1.0e-12;
algorithm
  stabilized := covariance;
  for row in 1:12 loop
    for col in row + 1:12 loop
      value := 0.5 * (stabilized[row, col] + stabilized[col, row]);
      stabilized[row, col] := value;
      stabilized[col, row] := value;
    end for;
    stabilized[row, row] := max(stabilized[row, row], minVariance);
  end for;
end stabilize_covariance;
