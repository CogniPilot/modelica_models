within LieGroups.SE23.Generic;
function exp_mixed "Mixed left/right SE_2(3) exponential update"
  input Element initialElement;
  input Real leftTangent[9];
  input Real rightTangent[9];
  input Real B[2, 2];
  output Element result;
protected
  Real initialColumns[3, 2];
  Real resultColumns[3, 2];
  Real leftN[3, 2];
  Real rightN[3, 2];
  Rotation.Orientation leftRotation;
  Rotation.Orientation rightRotation;
  Rotation.Orientation rightInitialRotation;
  Real rightR[3, 3];
  Real rightInitialR[3, 3];
algorithm
  initialColumns := [initialElement.velocity, initialElement.position];
  leftN := calculateN(leftTangent, B);
  rightN := calculateN(rightTangent, -B);
  leftRotation := Rotation.exp_map(leftTangent[7:9]);
  rightRotation := Rotation.exp_map(rightTangent[7:9]);
  rightInitialRotation := Rotation.product(
    rightRotation, initialElement.rotation);
  result.rotation := Rotation.product(
    rightInitialRotation, leftRotation);
  rightR := Rotation.to_Matrix(rightRotation);
  rightInitialR := Rotation.to_Matrix(rightInitialRotation);
  resultColumns := rightInitialR * leftN
    + (rightR * initialColumns + rightN) * ([1.0, 0.0; 0.0, 1.0] + B);
  result.velocity := resultColumns[:, 1];
  result.position := resultColumns[:, 2];
end exp_mixed;
