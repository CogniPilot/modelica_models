within LieGroups.SO3.Euler;
function inNonsingularChart
  "True when Euler coordinates stay away from their sequence singularity"
  input Real angles[3];
  input LieGroups.SO3.Euler.Axis sequence[3];
  input Real margin(min=0.0) = 1.0e-6
    "Lower bound on the relevant sine or cosine magnitude";
  output Boolean valid;
protected
  Boolean properEuler;
  Real singularityMeasure;
algorithm
  assert(LieGroups.SO3.Euler.validSequence(sequence),
    "Euler sequence cannot repeat adjacent axes");
  properEuler := sequence[1] == sequence[3];
  singularityMeasure := if properEuler
    then abs(sin(angles[2]))
    else abs(cos(angles[2]));
  valid := margin < 1.0 and singularityMeasure > margin;
end inNonsingularChart;
