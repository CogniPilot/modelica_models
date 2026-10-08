within Vision.Registration;
// Squared whitened residual from a diagonally equilibrated Cholesky solve.
// Invalid/singular covariance refuses: no invented noise floor or inverse.
function RegistrationWhitenedResidual3
  input Real residual[3]; input Real covariance[3,3];
  input Real minimumPivot = 1e-10;
  output Boolean valid; output Real squaredResidual;
protected
  Real scale[3]; Real normalized[3,3]; Real lower[3,3]; Real whitened[3];
  Real pivot;
algorithm
  valid := minimumPivot > 0.0 and minimumPivot < 1.0;
  scale := ones(3); normalized := zeros(3,3); lower := zeros(3,3); whitened := zeros(3);
  for axis in 1:3 loop
    valid := valid and covariance[axis,axis] > 0.0 and covariance[axis,axis] < 1e100
      and abs(residual[axis]) < 1e100;
    if valid then scale[axis] := sqrt(covariance[axis,axis]); end if;
  end for;
  for row in 1:3 loop
    for column in 1:3 loop
      normalized[row,column] := covariance[row,column]/scale[row]/scale[column];
      valid := valid and abs(normalized[row,column]) < 1e100
        and abs(normalized[row,column]-covariance[column,row]/scale[row]/scale[column]) <= 1e-10;
    end for;
  end for;
  for row in 1:3 loop
    for column in 1:row loop
      if valid then
        pivot := normalized[row,column];
        for previous in 1:column-1 loop
          pivot := pivot-lower[row,previous]*lower[column,previous];
        end for;
        if row == column then
          valid := pivot >= minimumPivot and pivot < 1e100;
          if valid then lower[row,column] := sqrt(pivot); end if;
        else
          lower[row,column] := pivot/lower[column,column];
        end if;
      end if;
    end for;
    if valid then
      pivot := residual[row]/scale[row];
      for previous in 1:row-1 loop pivot := pivot-lower[row,previous]*whitened[previous]; end for;
      whitened[row] := pivot/lower[row,row];
    end if;
  end for;
  squaredResidual := whitened*whitened;
  valid := valid and squaredResidual >= 0.0 and squaredResidual < 1e100;
  if not valid then squaredResidual := 0.0; end if;
end RegistrationWhitenedResidual3;
