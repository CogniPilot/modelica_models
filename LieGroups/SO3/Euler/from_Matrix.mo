within LieGroups.SO3.Euler;
function from_Matrix
  "DCM to Euler angles for any valid sequence using Shoemake's algorithm"
  input Real R[3, 3];
  input LieGroups.SO3.Euler.Axis sequence[3];
  input Boolean bodyFixed = true
    "True for intrinsic/body-fixed; false for extrinsic/space-fixed";
  output Real angles[3];
protected
  Integer i;
  Integer secondAxis;
  Integer thirdSequenceAxis;
  Integer j;
  Integer k;
  Boolean repetition;
  Boolean parity;
  Real firstAngle;
  Real secondAngle;
  Real thirdAngle;
  Real nonsingularMeasure;
  constant Real singularTolerance = 1.0e-12;
algorithm
  assert(LieGroups.SO3.Euler.validSequence(sequence),
    "Euler sequence cannot repeat adjacent axes");

  // Intrinsic a-b-c equals extrinsic c-b-a. Work with that effective
  // extrinsic sequence, then exchange the outer angles at the end.
  i := LieGroups.SO3.Euler.axisIndex(
    if bodyFixed then sequence[3] else sequence[1]);
  secondAxis := LieGroups.SO3.Euler.axisIndex(sequence[2]);
  thirdSequenceAxis := LieGroups.SO3.Euler.axisIndex(
    if bodyFixed then sequence[1] else sequence[3]);
  repetition := i == thirdSequenceAxis;

  // Cyclic sequences x-y-z, y-z-x, z-x-y have even parity.
  parity := secondAxis <> (if i < 3 then i + 1 else 1);
  j := if parity then (if i > 1 then i - 1 else 3)
       else (if i < 3 then i + 1 else 1);
  k := if parity then (if i < 3 then i + 1 else 1)
       else (if i > 1 then i - 1 else 3);

  if repetition then
    nonsingularMeasure := sqrt(R[i, j]^2 + R[i, k]^2);
    if nonsingularMeasure > singularTolerance then
      firstAngle := atan2(R[i, j], R[i, k]);
      secondAngle := atan2(nonsingularMeasure, R[i, i]);
      thirdAngle := atan2(R[j, i], -R[k, i]);
    else
      firstAngle := atan2(-R[j, k], R[j, j]);
      secondAngle := atan2(nonsingularMeasure, R[i, i]);
      thirdAngle := 0.0;
    end if;
  else
    nonsingularMeasure := sqrt(R[i, i]^2 + R[j, i]^2);
    if nonsingularMeasure > singularTolerance then
      firstAngle := atan2(R[k, j], R[k, k]);
      secondAngle := atan2(-R[k, i], nonsingularMeasure);
      thirdAngle := atan2(R[j, i], R[i, i]);
    else
      firstAngle := atan2(-R[j, k], R[j, j]);
      secondAngle := atan2(-R[k, i], nonsingularMeasure);
      thirdAngle := 0.0;
    end if;
  end if;

  if parity then
    firstAngle := -firstAngle;
    secondAngle := -secondAngle;
    thirdAngle := -thirdAngle;
  end if;
  angles := if bodyFixed
    then {thirdAngle, secondAngle, firstAngle}
    else {firstAngle, secondAngle, thirdAngle};
end from_Matrix;
