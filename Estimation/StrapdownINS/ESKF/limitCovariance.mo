within Estimation.StrapdownINS.ESKF;

function limitCovariance
  "Bound diagonal covariance growth to mission-envelope variance limits"
  input Estimation.StrapdownINS.ESKF.Covariance covariance;
  input Estimation.StrapdownINS.ESKF.VarianceLimits limits;
  output Estimation.StrapdownINS.ESKF.Covariance limited;
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
  // D*P*D is applied row by row rather than as the matrix product
  // diagonal(rescale) * covariance * diagonal(rescale). The matrix form
  // multiplies through 15^4 + 15^3 = 54,000 products per call of which only
  // 450 are structurally non-zero, on EVERY tick, because the generated code
  // emits ((i == k) ? rescale[i] : 0.0) * covariance[k, j] and may not delete
  // a multiply by a structural zero without changing NaN/Inf/-0.0 behaviour.
  // Measured on the RDD2 estimator: 54,000 -> 450 multiplies per call and
  // -30.7 us on every tick. See dev/sparsity-eval in the rumoca tree.
  //
  // The row form keeps the surviving products in their original order, so
  // against the expanded matrix form it is exact rather than merely close.
  // (D*P)[i, j] is a sum whose only non-zero term is rescale[i]*P[i, j], and
  // the surviving term of ((D*P)*D)[i, j] is then (D*P)[i, j]*rescale[j]; the
  // deleted terms are exact zeros that shift no partial sum. The two
  // multiplies below are those two products, left to right, so nothing is
  // reassociated. Verified bit-identical through rumoca's generated C over a
  // 4000-tick mission trace at both -O2 and -Os.
  //
  // Two behaviours do change, both strictly less destructive: a NaN or an
  // infinity sitting at a structurally-zero position of the discarded outer
  // products no longer poisons the whole matrix, and a live -0.0 is no longer
  // flipped to +0.0 by summation against the deleted zero terms.
  //
  // Under OpenModelica the result moves by at most 2 ulp (measured over 400
  // random covariances, 1.2% of entries). That is not a regression against the
  // expanded product: omc already simplified the diagonal product itself, and
  // grouped it as (rescale[i]*rescale[j])*P[i, j], which is neither the
  // expanded form nor the left-to-right order Modelica specifies for a*b*c.
  // The row form below is the left-associative one.
  //
  // Written with whole-row slices rather than a scalar `limited[i, j] :=
  // rescale[i] * covariance[i, j] * rescale[j]` because rumoca rejects the
  // scalar form: reading a vector at the inner loop index inside a nest that
  // writes `x[i, j]` trips ED020 "expression shape mismatch". The slice form
  // is equivalent and compiles under both toolchains.
  for i in 1:TangentLength loop
    scaledRow := rescale[i] * covariance[i, :];
    limited[i, :] := scaledRow .* rescale;
  end for;
end limitCovariance;
