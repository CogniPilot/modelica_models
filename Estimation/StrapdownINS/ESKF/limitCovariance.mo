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
  // A non-finite diagonal entry has no finite rescale factor that lands on the
  // bound: bound/covariance is zero for an infinity, and the row product then
  // evaluates 0 * inf on the infinite entry itself, which is NaN rather than
  // the bound this function exists to impose. One unusable entry became an
  // unusable matrix. Reachability is not the argument for handling it; a
  // covariance limiter that can itself manufacture NaN is not a limiter.
  //
  // Such a row gets a zero rescale here; it is then overwritten with zeros and
  // its diagonal rebuilt at the bound below. That is the honest clamp: the variance is the largest this
  // filter is allowed to hold, and every correlation with it is dropped,
  // because a row that reached infinity carries no usable correlation. The
  // result stays symmetric positive semi-definite, being the sum of a scaled
  // congruence and a non-negative diagonal.
  //
  // The test is written as a positive finiteness proof so NaN, which fails
  // every comparison, takes the same branch as an infinity.
  for i in 1:TangentLength loop
    rescale[i] := if not abs(covariance[i, i]) < FiniteMagnitudeLimit then 0.0
      elseif covariance[i, i] > bound[i]
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
  // Whole rows expose the shared left scale and preserve the intended
  // multiplication order without constructing dense diagonal matrices.
  for i in 1:TangentLength loop
    scaledRow := rescale[i] * covariance[i, :];
    limited[i, :] := scaledRow .* rescale;
  end for;
  // Overwrite, do not scale, any row and column whose diagonal was rejected.
  // Multiplying is not enough: rescale[i] is 0.0 there, and 0.0 * inf is NaN,
  // so the products above leave NaN at exactly the off-diagonal positions of
  // an infinite row -- and a row cannot reach infinity on the diagonal of a
  // positive semi-definite matrix without its correlates being unusable too.
  // Assigning the zeros explicitly is the only way to stop a non-finite entry
  // from escaping sideways, which is the whole point of the branch above.
  //
  // The diagonal is then rebuilt at its bound, so a non-finite entry leaves
  // the filter maximally uncertain about that state rather than falsely
  // certain. A zero variance would claim the state is known exactly and would
  // drive the next gain to reject every measurement of it, which is the
  // opposite of what an unusable entry warrants. The result is symmetric and
  // positive semi-definite: it is the surviving principal submatrix, itself a
  // congruence of a covariance, bordered by zeros and a positive diagonal.
  // Written as conditional EXPRESSIONS rather than a loop inside an `if`.
  // The nested form is rejected by the galec production target (ED019), and
  // the expression form is what the rescale selection above already uses.
  for i in 1:TangentLength loop
    limited[i, :] := if rescale[i] <= 0.0 then zeros(TangentLength)
      else limited[i, :];
    limited[:, i] := if rescale[i] <= 0.0 then zeros(TangentLength)
      else limited[:, i];
    limited[i, i] := if rescale[i] <= 0.0 then bound[i] else limited[i, i];
  end for;
end limitCovariance;
