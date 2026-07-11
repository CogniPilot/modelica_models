within Tests.Assertions;
function determinant3 "Determinant of a 3x3 matrix"
  input Real A[3, 3];
  output Real determinant;
algorithm
  determinant := A[1, 1] * (A[2, 2] * A[3, 3] - A[2, 3] * A[3, 2])
    - A[1, 2] * (A[2, 1] * A[3, 3] - A[2, 3] * A[3, 1])
    + A[1, 3] * (A[2, 1] * A[3, 2] - A[2, 2] * A[3, 1]);
end determinant3;
