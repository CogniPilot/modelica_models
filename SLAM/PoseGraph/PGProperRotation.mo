within SLAM.PoseGraph;
function PGProperRotation
  input Real R[3,3]; output Boolean valid;
protected Real value; Real determinant;
algorithm
  valid := true; value := 0.0;
  for i in 1:3 loop
    for j in 1:3 loop
      valid := valid and abs(R[i,j]) <= 1.000001;
      value := 0.0;
      for k in 1:3 loop value := value+R[k,i]*R[k,j]; end for;
      valid := valid and abs(value-(if i == j then 1.0 else 0.0)) <= 1e-7;
    end for;
  end for;
  determinant := R[1,1]*(R[2,2]*R[3,3]-R[2,3]*R[3,2])
    -R[1,2]*(R[2,1]*R[3,3]-R[2,3]*R[3,1])+R[1,3]*(R[2,1]*R[3,2]-R[2,2]*R[3,1]);
  valid := valid and abs(determinant-1.0) <= 1e-7;
end PGProperRotation;
