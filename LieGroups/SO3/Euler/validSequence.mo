within LieGroups.SO3.Euler;
function validSequence "True for Tait-Bryan or proper Euler axis sequences"
  input LieGroups.SO3.Euler.Axis sequence[3];
  output Boolean valid;
algorithm
  valid := sequence[1] <> sequence[2] and sequence[2] <> sequence[3];
end validSequence;
