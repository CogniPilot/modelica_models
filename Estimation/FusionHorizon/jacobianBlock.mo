within Estimation.FusionHorizon;

function jacobianBlock "Read one 3x3 bias Jacobian out of a ring row"
  input Real row[DeltaLength];
  input Integer offset "Index just before the block's first element";
  output Real jacobian[3, 3];
algorithm
  jacobian[1, 1] := row[offset + 1];
  jacobian[1, 2] := row[offset + 2];
  jacobian[1, 3] := row[offset + 3];
  jacobian[2, 1] := row[offset + 4];
  jacobian[2, 2] := row[offset + 5];
  jacobian[2, 3] := row[offset + 6];
  jacobian[3, 1] := row[offset + 7];
  jacobian[3, 2] := row[offset + 8];
  jacobian[3, 3] := row[offset + 9];
end jacobianBlock;
