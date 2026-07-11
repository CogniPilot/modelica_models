within LieGroups.SO3.Euler;
function to_Matrix "Euler angles to DCM for any valid axis sequence"
  input Real angles[3];
  input LieGroups.SO3.Euler.Axis sequence[3];
  input Boolean bodyFixed = true
    "True for intrinsic/body-fixed; false for extrinsic/space-fixed";
  output Real R[3, 3];
protected
  Real elemental[3, 3];
algorithm
  assert(LieGroups.SO3.Euler.validSequence(sequence),
    "Euler sequence cannot repeat adjacent axes");
  R := identity(3);
  for i in 1:3 loop
    elemental := LieGroups.SO3.Euler.rotationMatrix(sequence[i], angles[i]);
    R := if bodyFixed then R * elemental else elemental * R;
  end for;
end to_Matrix;
