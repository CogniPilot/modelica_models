within Estimation.StrapdownINS.ESKF;

function limitCovariance
  "Bound diagonal covariance growth to mission-envelope variance limits"
  input Covariance covariance;
  input VarianceLimits limits;
  output Covariance limited;
protected
  Real bound[TangentLength];
  Real rescale[TangentLength];
  Real scaledRow[TangentLength];
algorithm
  bound := cat(1,
    limits.position_m2,
    limits.velocity_m2_s2,
    limits.attitude_rad2,
    limits.gyroscopeBias_rad2_s2,
    limits.accelerometerBias_m2_s4);
  for row in 1:TangentLength loop
    rescale[row] := if not abs(covariance[row, row]) < FiniteMagnitudeLimit then 0.0
      elseif covariance[row, row] > bound[row]
      then sqrt(bound[row] / covariance[row, row]) else 1.0;
  end for;
  for row in 1:TangentLength loop
    scaledRow := rescale[row] * covariance[row, :];
    limited[row, :] := scaledRow .* rescale;
  end for;
  for row in 1:TangentLength loop
    limited[row, :] := if rescale[row] <= 0.0 then zeros(TangentLength)
      else limited[row, :];
    limited[:, row] := if rescale[row] <= 0.0 then zeros(TangentLength)
      else limited[:, row];
    limited[row, row] := if rescale[row] <= 0.0 then bound[row] else limited[row, row];
  end for;
end limitCovariance;
