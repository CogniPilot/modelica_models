within Tests.LieGroupTests;
function allEulerSequencesPass
  "Round-trip all 12 Euler sequences in body-fixed and space-fixed form"
  output Boolean passed;
protected
  LieGroups.SO3.Euler.Axis sequences[12, 3];
  Real angles[3];
  Real recovered[3];
  Real R[3, 3];
  Real recoveredR[3, 3];
  Boolean bodyFixed;
algorithm
  sequences := [
    LieGroups.SO3.Euler.Axis.x, LieGroups.SO3.Euler.Axis.y, LieGroups.SO3.Euler.Axis.x;
    LieGroups.SO3.Euler.Axis.x, LieGroups.SO3.Euler.Axis.z, LieGroups.SO3.Euler.Axis.x;
    LieGroups.SO3.Euler.Axis.y, LieGroups.SO3.Euler.Axis.x, LieGroups.SO3.Euler.Axis.y;
    LieGroups.SO3.Euler.Axis.y, LieGroups.SO3.Euler.Axis.z, LieGroups.SO3.Euler.Axis.y;
    LieGroups.SO3.Euler.Axis.z, LieGroups.SO3.Euler.Axis.x, LieGroups.SO3.Euler.Axis.z;
    LieGroups.SO3.Euler.Axis.z, LieGroups.SO3.Euler.Axis.y, LieGroups.SO3.Euler.Axis.z;
    LieGroups.SO3.Euler.Axis.x, LieGroups.SO3.Euler.Axis.y, LieGroups.SO3.Euler.Axis.z;
    LieGroups.SO3.Euler.Axis.x, LieGroups.SO3.Euler.Axis.z, LieGroups.SO3.Euler.Axis.y;
    LieGroups.SO3.Euler.Axis.y, LieGroups.SO3.Euler.Axis.x, LieGroups.SO3.Euler.Axis.z;
    LieGroups.SO3.Euler.Axis.y, LieGroups.SO3.Euler.Axis.z, LieGroups.SO3.Euler.Axis.x;
    LieGroups.SO3.Euler.Axis.z, LieGroups.SO3.Euler.Axis.x, LieGroups.SO3.Euler.Axis.y;
    LieGroups.SO3.Euler.Axis.z, LieGroups.SO3.Euler.Axis.y, LieGroups.SO3.Euler.Axis.x];
  angles := {0.31, 0.67, -0.42};
  passed := true;
  for convention in 1:2 loop
    bodyFixed := convention == 1;
    for sequenceIndex in 1:12 loop
      R := LieGroups.SO3.Euler.to_Matrix(
        angles, sequences[sequenceIndex, :], bodyFixed);
      recovered := LieGroups.SO3.Euler.from_Matrix(
        R, sequences[sequenceIndex, :], bodyFixed);
      recoveredR := LieGroups.SO3.Euler.to_Matrix(
        recovered, sequences[sequenceIndex, :], bodyFixed);
      passed := passed and
        Tests.Assertions.maxAbsMatrix(recoveredR - R) < 1.0e-9;
    end for;
  end for;
end allEulerSequencesPass;
