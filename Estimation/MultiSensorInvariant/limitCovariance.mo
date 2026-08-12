within Estimation.MultiSensorInvariant;

function limitCovariance
  "Bound diagonal covariance growth to mission-envelope variance limits"
  input Estimation.MultiSensorInvariant.Covariance covariance;
  input Estimation.MultiSensorInvariant.VarianceLimits limits;
  output Estimation.MultiSensorInvariant.Covariance limited;
protected
  Real bound[TangentLength];
  Real rescale[TangentLength];
algorithm
  bound := cat(1,
    limits.position_m2,
    limits.velocity_m2_s2,
    limits.attitude_rad2,
    limits.gyroscopeBias_rad2_s2,
    limits.accelerometerBias_m2_s4);
  for i in 1:TangentLength loop
    rescale[i] := if covariance[i, i] > bound[i]
      then sqrt(bound[i] / covariance[i, i]) else 1.0;
  end for;
  // Two-sided scaling D*P*D with diagonal 0 < D <= I keeps the matrix
  // symmetric positive semi-definite and preserves every correlation
  // coefficient, unlike clamping diagonal entries in isolation. Bounding
  // the diagonal keeps long unaided propagation from integrating the
  // covariance past the dynamic range where a reduced-precision Cholesky
  // can still factor it.
  limited := diagonal(rescale) * covariance * diagonal(rescale);
end limitCovariance;
