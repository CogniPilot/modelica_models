within Estimation.FusionHorizon;

function retireRows
  "Divide the earliest packed factor out of a packed product"
  input Real firstRow[DeltaLength] "The factor being retired";
  input Real composedRow[DeltaLength] "The product it is the left factor of";
  output Real secondRow[DeltaLength];
protected
  Estimation.FusionHorizon.Delta first;
  Estimation.FusionHorizon.Delta composed;
  Estimation.FusionHorizon.Delta second;
algorithm
  // The flat-row facade over retireDelta; see composeRows for why the boundary
  // is an array and not a record.
  first := Estimation.FusionHorizon.unpackDelta(firstRow);
  composed := Estimation.FusionHorizon.unpackDelta(composedRow);
  second := Estimation.FusionHorizon.retireDelta(first, composed);
  secondRow := Estimation.FusionHorizon.packDelta(second);
end retireRows;
