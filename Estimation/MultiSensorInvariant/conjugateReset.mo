within Estimation.MultiSensorInvariant;

function conjugateReset
  "Conjugate a covariance by the block-diagonal reset Jacobian diag(J, I)"
  input Real rotationJacobian[9, 9]
    "SE_2(3) right Jacobian J of the applied correction";
  input Estimation.MultiSensorInvariant.Covariance covariance;
  output Estimation.MultiSensorInvariant.Covariance conjugated;
protected
  Real rotationJacobianTransposed[9, 9];
  Real rotated[9, TangentLength] "J * covariance[1:9, :]";
algorithm
  // The reset Jacobian is the block diagonal
  //   Jr = cat(1, cat(2, J, zeros(9, 6)), cat(2, zeros(6, 9), identity(6)))
  // and the reset conjugates the posterior covariance as Jr * P * transpose(Jr).
  // Forming Jr and multiplying it through costs 15^4 + 15^3 = 54,000 float
  // multiplies per call, of which 2,430 are structurally non-zero: the
  // generated code emits the literal zeros() and identity() entries as a
  // `(cond ? value : 0.0)` selection inside a full-extent contraction, and it
  // may not delete a multiply by a structural zero without changing
  // NaN/Inf/-0.0 behaviour. Measured on Vehicles.Rdd2.NavigationEstimator
  // through the real rumoca compiler (adversarial re-measurement: gcov
  // multiply counts at -O0 -ffp-contract=off; pinned-core CPU-time minima of
  // nine runs): 54,000 -> 2,430 multiplies per call (22.2x), aided tick
  // 194.1 -> 99.9 us at -O2 and 364.5 -> 230.9 us at -Os.
  //
  // Writing out the four blocks of Jr * P * transpose(Jr):
  //   [1:9,   1:9  ] = (J * P[1:9, :])[:, 1:9] * transpose(J)
  //   [1:9,   10:15] = (J * P[1:9, :])[:, 10:15]
  //   [10:15, 1:9  ] = P[10:15, 1:9] * transpose(J)
  //   [10:15, 10:15] = P[10:15, 10:15]
  //
  // Nothing is reassociated. Every surviving product keeps both operands and
  // its position in the summation order. The inner contraction
  // sum(l in 1:15) Jr[i, l] * P[l, k] has non-zero terms only at l = 1:9 when
  // i <= 9 -- so the deleted terms are trailing -- and only the single term
  // l = i when i > 9; the outer contraction against transpose(Jr) behaves the
  // same way in k. Deleted terms are exact zeros that shift no partial sum,
  // and the identity block multiplies by an exact 1.0. Through rumoca's
  // generated C this is bit-identical to the dense form over a 4000-tick
  // mission trace exercising all five correction shapes, at both -O2 and -Os
  // (0 differing tokens of 4,680). In the original study's OpenModelica
  // harness it was exactly equal: 0 of 90,000 entries differed over 400
  // random covariances spanning nine decades.
  //
  // Two behaviours do change, both strictly less destructive, exactly as in
  // limitCovariance: a NaN or an infinity sitting at a structurally-zero
  // position no longer poisons the whole matrix, and a live -0.0 is no longer
  // flipped to +0.0 by summation against the deleted zero terms. That makes
  // the sparse form a distinct root rather than a drop-in, and strictly less
  // NaN-propagating --
  // a health monitor that relied on covariance NaN poisoning to notice a bad
  // entry in a structurally-zero position would no longer see it.
  //
  // THREE THINGS ABOUT THE WAY THIS IS WRITTEN ARE LOAD-BEARING; do not
  // "simplify" them without re-measuring and re-checking bit-identity.
  //
  // 1. It is a separate function rather than inline code in correctLinear.
  //    The conjugation there sits inside `if accepted then`, and rumoca
  //    rejects an element-writing `for` loop inside a conditional branch of a
  //    function with ED019 "unsupported Flat semantic owner `function
  //    conditional`". The same loop in a function called from the branch is
  //    accepted.
  //
  // 2. The four blocks are assigned as whole rows built with cat, not as the
  //    four block slices `conjugated[1:9, 1:9] := ...` etc. Slice assignments
  //    that partition the index space are fused by rumoca into a single
  //    guarded 15x15 element nest that re-inlines the producers -- including
  //    josephUpdate -- once per element. Measured: the block-slice form is
  //    49x slower than the dense form it replaces, and a row-band variant of
  //    it 96x slower.
  //
  // 3. transpose(J) is materialised into rotationJacobianTransposed and the
  //    row products are written against the whole matrix. The natural
  //    `for j ... conjugated[i, j] := rotated[i, 1:9] * rotationJacobian[j, :]`
  //    is SILENTLY MISCOMPILED by rumoca: it compacts the inner element loop
  //    to `conjugated[i, 1:9] := rotated[i, 1:9] * rotationJacobian[1:9, :]`,
  //    which raises the operand from a row vector to the whole matrix and so
  //    turns the vector-vector scalar product of MLS 10.6.4 into a
  //    vector-matrix product -- J transposed, with no diagnostic. Keeping the
  //    contraction as one row-vector-times-matrix expression leaves nothing
  //    for that rewrite to do.
  rotationJacobianTransposed := transpose(rotationJacobian);
  rotated := rotationJacobian * covariance[1:9, :];
  for i in 1:9 loop
    conjugated[i, :] := cat(1,
      rotated[i, 1:9] * rotationJacobianTransposed,
      rotated[i, 10:TangentLength]);
  end for;
  for i in 10:TangentLength loop
    conjugated[i, :] := cat(1,
      covariance[i, 1:9] * rotationJacobianTransposed,
      covariance[i, 10:TangentLength]);
  end for;
end conjugateReset;
