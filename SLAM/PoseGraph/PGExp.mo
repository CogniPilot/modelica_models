within SLAM.PoseGraph;
function PGExp
  import PGMultiply = SLAM.PoseGraph.PGMultiply;
  import PGSkew = SLAM.PoseGraph.PGSkew;

  input Real angle[3]; output Real R[3,3];
protected Real square; Real theta; Real a; Real b; Real S[3,3];
algorithm
  square := angle[1]*angle[1]+angle[2]*angle[2]+angle[3]*angle[3]; theta := sqrt(max(square,0.0));
  a := if square < 1e-8 then 1.0-square/6.0+square*square/120.0 else sin(theta)/max(theta,1e-12);
  b := if square < 1e-8 then 0.5-square/24.0+square*square/720.0 else (1.0-cos(theta))/max(square,1e-12);
  S := PGSkew(angle); R := identity(3)+a*S+b*PGMultiply(S,S);
end PGExp;
