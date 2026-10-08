within SLAM.Localization;
// SPD solve with dimensionless diagonal equilibration, no damping/floor added
// to the matrix. A small or nonfinite pivot is a refusal, not invented noise.
function RGBDUncertaintyInverse6
  input Real A[6,6];
  input Real minimumPivot;
  output Real inverseA[6,6];
  output Boolean valid;
  output Real minimumScaledPivot;
protected
  Real scale[6]; Real L[6,6]; Real v; Real z[6]; Real x[6];
algorithm
  inverseA := zeros(6,6); L := zeros(6,6); scale := ones(6);
  minimumScaledPivot := 1.0; valid := true;
  for i in 1:6 loop
    valid := valid and A[i,i] > 0.0 and A[i,i] <= 1e100;
    scale[i] := sqrt(if A[i,i] > 0.0 and A[i,i] <= 1e100 then A[i,i] else 1.0);
    for j in 1:6 loop
      valid := valid and abs(A[i,j]) <= 1e100
        and abs(A[i,j]-A[j,i]) <= 1e-10*max(1.0,abs(A[i,j]));
    end for;
  end for;
  for i in 1:6 loop
    for j in 1:6 loop
      if j <= i then
        v := if valid then A[i,j]/scale[i]/scale[j] else 0.0;
        for k in 1:6 loop
          v := v-(if k < j then L[i,k]*L[j,k] else 0.0);
        end for;
        if j == i then
          minimumScaledPivot := min(minimumScaledPivot,v);
          valid := valid and v > minimumPivot and v <= 1e100;
          L[i,j] := sqrt(if valid then v else 1.0);
        else
          L[i,j] := if valid then v/L[j,j] else 0.0;
        end if;
      end if;
    end for;
  end for;
  for column in 1:6 loop
    z := zeros(6); x := zeros(6);
    for i in 1:6 loop
      v := if i == column then 1.0/scale[column] else 0.0;
      for k in 1:6 loop
        v := v-(if k < i then L[i,k]*z[k] else 0.0);
      end for;
      z[i] := if valid then v/L[i,i] else 0.0;
    end for;
    for index in 1:6 loop
      v := z[7-index];
      for k in 1:6 loop
        v := v-(if k > 7-index then L[k,7-index]*x[k] else 0.0);
      end for;
      x[7-index] := if valid then v/L[7-index,7-index] else 0.0;
    end for;
    for i in 1:6 loop
      inverseA[i,column] := if valid then x[i]/scale[i] else 0.0;
    end for;
  end for;
  inverseA := if valid then inverseA else zeros(6,6);
end RGBDUncertaintyInverse6;
