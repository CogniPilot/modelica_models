within Estimation.FusionHorizon;

function composeRows
  "Compose two packed right factors and return a packed row"
  input Real firstRow[DeltaLength];
  input Real secondRow[DeltaLength];
  output Real composedRow[DeltaLength];
protected
  Estimation.FusionHorizon.Delta first;
  Estimation.FusionHorizon.Delta second;
  Estimation.FusionHorizon.Delta composed;
algorithm
  // A FLAT-ROW FACADE over composeDelta, and it exists for the code generator
  // rather than for the algebra. A record-valued call is materialized once per
  // component of the record it returns, and a Delta has 56 of them, so every
  // Delta-returning call site in a block's algorithm multiplies the work the
  // lowering does by that factor. Measured on this block: carrying the window
  // product through Delta-valued call sites took the galec-production lowering
  // of OutputPredictor from 1.2 seconds to minutes without completing.
  //
  // Returning an ARRAY instead keeps the record inside one function body,
  // where it is evaluated once, and hands the caller a row. It is the same
  // accommodation foldBuffer and step already make, for the same defect.
  first := Estimation.FusionHorizon.unpackDelta(firstRow);
  second := Estimation.FusionHorizon.unpackDelta(secondRow);
  composed := Estimation.FusionHorizon.composeDelta(first, second);
  composedRow := Estimation.FusionHorizon.packDelta(composed);
end composeRows;
