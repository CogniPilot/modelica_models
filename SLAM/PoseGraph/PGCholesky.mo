within SLAM.PoseGraph;
function PGCholesky
  input Real A[6,6]; input Real pivotRelative;
  output Real L[6,6]; output Boolean valid;
protected Real scale; Real value;
algorithm
  L := zeros(6,6); valid := true; scale := 0.0; value := 0.0;
  for i in 1:6 loop scale := max(scale,abs(A[i,i])); end for;
  valid := scale > 1e-12 and scale <= 1e18;
  for i in 1:6 loop
    for j in 1:6 loop
      valid := valid and abs(A[i,j]) <= 1e18
        and abs(A[i,j]-A[j,i]) <= 1e-10*max(1.0,scale);
      value := A[i,j];
      for k in 1:6 loop
        if j <= i and k < j then value := value-L[i,k]*L[j,k]; end if;
      end for;
      if j <= i then
        if i == j then
          valid := valid and value > pivotRelative*scale and value <= 1e18;
          L[i,j] := sqrt(if value > pivotRelative*scale and value <= 1e18 then value else 1.0);
        else
          L[i,j] := value/L[j,j];
        end if;
      end if;
    end for;
  end for;
end PGCholesky;
