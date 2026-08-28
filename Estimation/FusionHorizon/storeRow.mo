within Estimation.FusionHorizon;

function storeRow "Write one delta into the ring and return the ring"
  input Real ring[:, DeltaLength];
  input Integer slot
    "Row to write; any value outside 1..size(ring,1) writes nothing, which is
     how a tick that closes no window stores nothing without a branch";
  input Real row[DeltaLength];
  output Real updated[size(ring, 1), DeltaLength];
protected
  Real hit;
algorithm
  // A store is one row. It is a masked fixed-length walk rather than
  // updated[slot, :] := row because the code generator refuses a dynamic array
  // index it cannot prove in range, and because a constant-time store makes
  // the worst case the measured case.
  for k in 1:size(ring, 1) loop
    hit := if k == slot then 1.0 else 0.0;
    updated[k, :] := hit * row + (1.0 - hit) * ring[k, :];
  end for;
end storeRow;
