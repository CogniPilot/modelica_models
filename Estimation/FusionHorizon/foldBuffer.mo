within Estimation.FusionHorizon;

function foldBuffer
  "Compose a contiguous run of ring entries, then one trailing row, into one
   right factor"
  input Real ring[:, DeltaLength];
  input Integer tail(min = 1) "Index of the oldest entry in the run";
  input Integer count(min = 0) "Number of entries to fold";
  input Real trailingRow[DeltaLength] =
    cat(1, zeros(6), {1.0}, zeros(49))
    "One more factor, composed AFTER the run and coming from outside the ring.
     The caller uses it for the window accumulated since the last adopt, which
     on a release boundary is no longer part of the live delta and is not in
     the ring either: the ring the caller passes is the one from before this
     tick's store. It defaults to the identity row, which is what makes the
     extra factor invisible to a caller that has no trailing window.";
  output Estimation.FusionHorizon.Delta accumulated;
protected
  Integer bufferLength;
  Integer index;
  Real identityRow[DeltaLength];
  Real selected[DeltaLength];
  Real inWindow;
  Real isTrailing;
  Estimation.FusionHorizon.Delta identity;
  Estimation.FusionHorizon.Delta entry;
  Estimation.FusionHorizon.Delta composed;
algorithm
  // The re-base kernel, and the worst case of the whole architecture: a fold
  // walks the entire buffer. It is deliberately the fold and not the cheaper
  // peel D(t1->t2) = D(t0->t1)^-1 (x) D(t0->t2), because the peel never
  // re-anchors and its single-precision error compounds without bound over a
  // flight in exactly the quantity control reads. The fold re-derives the
  // window from stored deltas every time it runs, so the predictor's error is
  // bounded by the horizon length instead.
  //
  // The pass is DELIBERATELY branch-free and fixed-length: it always walks the
  // whole ring and selects each slot arithmetically, composing the identity
  // where the slot is outside the window. Composing with the identity is
  // exact, so the result is the same element a variable-length loop would
  // produce, and the cost of a fold does not depend on how full the buffer is.
  // That makes the worst case the measured case, and it keeps the loop inside
  // what the code generator will lower: a data-dependent trip count is not a
  // compact dependent domain and is refused.
  //
  // The trailing row rides the SAME walk, as one extra iteration whose slot is
  // selected arithmetically like every other. That is not tidiness. Composing
  // it at the call site instead adds a composition and an unpack to the
  // source, and a record-valued call is materialized once per component of the
  // record it returns: two extra call sites took the production lowering of
  // the owning block from 37 seconds to over eight minutes without finishing.
  // Here it costs one more iteration of a loop that already exists.
  bufferLength := size(ring, 1);
  // Every record-returning call is bound to a local before it is used. A
  // record-valued call written directly as an argument is re-evaluated once
  // per component of the record it returns: measured, that turned a
  // twenty-two entry fold into thirty-five folds and cost 117 million
  // instructions per re-base. Naming the intermediate is the whole fix.
  identity := Estimation.FusionHorizon.identityDelta();
  identityRow := Estimation.FusionHorizon.packDelta(identity);
  accumulated := identity;
  for k in 1:size(ring, 1) + 1 loop
    index := tail + k - 1;
    index := index - bufferLength * (if index > bufferLength then 1 else 0);
    inWindow := if k <= count then 1.0 else 0.0;
    isTrailing := if k == count + 1 then 1.0 else 0.0;
    selected := inWindow * ring[index, :] + isTrailing * trailingRow
      + (1.0 - inWindow - isTrailing) * identityRow;
    // The result goes to a fresh local and is copied back, rather than the
    // accumulator appearing on both sides of the call. A record assignment
    // whose own value is an argument is re-evaluated once per component of
    // that record: measured, thirty-five evaluations of the composition per
    // loop iteration instead of one.
    entry := Estimation.FusionHorizon.unpackDelta(selected);
    composed := Estimation.FusionHorizon.composeDelta(accumulated, entry);
    accumulated := composed;
  end for;
end foldBuffer;
