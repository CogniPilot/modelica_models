within SLAM.Inertial;
// A correlated, frozen reference pose is a Schmidt state, not pose truth.
// World-additive p/v; right-local body attitude; current state order p,v,theta,ba,bg.
// Registration maps reference optical points to current optical coordinates.
// Compile with RGBDRelativePose.mo, ES15PoseCorrection.mo and SPD6Solve.mo.
// These components are under numerical review; the production preset is unchanged.
// Sequential validation only: tolerance jitter never enters retained covariance.
// The function owns factor ordering instead of a cyclic equation-form factor graph.
function SLAMCovariancePSDCheck
  input Real covariance[:,:];
  input Real relativeTolerance;
  output Real valid;
protected
  Real scale;
  Real pivot;
  Real diagonalScale;
  Real diagonalSum;
  Real offDiagonalSum;
  Real lower[size(covariance,1),size(covariance,1)];
algorithm
  diagonalScale := 0.0;
  diagonalSum := 0.0;
  offDiagonalSum := 0.0;
  for i in 1:size(covariance,1) loop
    diagonalScale := diagonalScale+abs(covariance[i,i]);
  end for;
  scale := max(1.0,diagonalScale);
  lower := zeros(size(covariance,1),size(covariance,1));
  pivot := 0.0;
  valid := if size(covariance,1) == size(covariance,2) then 1.0 else 0.0;
  for i in 1:size(covariance,1) loop
    for j in 1:size(covariance,1) loop
      offDiagonalSum := 0.0;
      diagonalSum := 0.0;
      // Ordered finite accumulation; no array-valued comprehension temporary.
      for k in 1:size(covariance,1) loop
        offDiagonalSum := offDiagonalSum+(if k < j then lower[i,k]*lower[j,k] else 0.0);
        diagonalSum := diagonalSum+(if k < i then lower[i,k]^2 else 0.0);
      end for;
      if not (abs(covariance[i,j]) <= 1e12 and
          abs(covariance[i,j]-covariance[j,i]) <= relativeTolerance*scale) then
        valid := 0.0;
      end if;
      if j < i then
        lower[i,j] := (covariance[i,j]-offDiagonalSum)
          /max(lower[j,j],1e-150);
      elseif j == i then
        pivot := covariance[i,i]+relativeTolerance*scale
          -diagonalSum;
        if not (pivot > 0.0 and pivot <= 1e12) then valid := 0.0; end if;
        lower[i,j] := sqrt(if pivot > 0.0 then pivot else 1.0);
      end if;
    end for;
  end for;
end SLAMCovariancePSDCheck;
