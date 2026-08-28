within Estimation.FusionHorizon;

function readRow "Select one ring row without a dynamic index"
  input Real ring[:, DeltaLength];
  input Integer slot "Row to read; any value outside 1..size(ring,1) reads zero";
  output Real row[DeltaLength];
protected
  Real hit;
algorithm
  // Selecting a row is row arithmetic, not composition. Keeping the two apart
  // is what makes the buffer affordable: a fold that composes every slot to
  // reach one entry costs a full window of group products, and measured, that
  // was the dominant term. This costs one multiply-add per stored real.
  //
  // A masked walk rather than ring[slot, :] because the code generator refuses
  // a dynamic array index it cannot prove in range -- which for a ring buffer
  // is the right refusal -- while a loop variable carries its bound with it.
  row := zeros(DeltaLength);
  for k in 1:size(ring, 1) loop
    hit := if k == slot then 1.0 else 0.0;
    row := row + hit * ring[k, :];
  end for;
end readRow;
