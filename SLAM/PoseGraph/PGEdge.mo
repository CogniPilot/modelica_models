within SLAM.PoseGraph;
function PGEdge
  import PGLog = SLAM.PoseGraph.PGLog;
  import PGMatVec = SLAM.PoseGraph.PGMatVec;
  import PGMultiply = SLAM.PoseGraph.PGMultiply;
  import PGSkew = SLAM.PoseGraph.PGSkew;
  import PGTranspose = SLAM.PoseGraph.PGTranspose;

  input Real pi[3]; input Real Ri[3,3]; input Real pj[3]; input Real Rj[3,3];
  input Real translation[3]; input Real measuredRotation[3,3];
  output Real residual[6]; output Real Ji[6,6]; output Real Jj[6,6]; output Boolean valid;
protected Real t[3]; Real r[3]; Real square; Real theta; Real c;
  Real S[3,3]; Real leftInverse[3,3]; Real rightInverse[3,3];
algorithm
  t := PGMatVec(PGTranspose(Ri),pj-pi);
  (r,valid) := PGLog(PGMultiply(PGMultiply(PGTranspose(measuredRotation),PGTranspose(Ri)),Rj));
  square := r[1]*r[1]+r[2]*r[2]+r[3]*r[3]; theta := sqrt(max(square,0.0));
  c := if square < 1e-8 then 1.0/12.0+square/720.0+square*square/30240.0
    else (1.0-0.5*theta*cos(0.5*theta)/max(sin(0.5*theta),1e-12))/max(square,1e-12);
  S := PGSkew(r); leftInverse := identity(3)-0.5*S+c*PGMultiply(S,S);
  rightInverse := identity(3)+0.5*S+c*PGMultiply(S,S);
  residual := zeros(6); Ji := zeros(6,6); Jj := zeros(6,6); S := PGSkew(t);
  leftInverse := -PGMultiply(leftInverse,PGTranspose(measuredRotation));
  for i in 1:3 loop
    residual[i] := t[i]-translation[i]; residual[i+3] := r[i];
    for j in 1:3 loop
      Ji[i,j] := -Ri[j,i]; Jj[i,j] := Ri[j,i]; Ji[i,j+3] := S[i,j];
      Ji[i+3,j+3] := leftInverse[i,j]; Jj[i+3,j+3] := rightInverse[i,j];
    end for;
  end for;
end PGEdge;
