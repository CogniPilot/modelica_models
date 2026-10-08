within SLAM.PoseGraph;
function PGPCG
  import PGDampedFactor = SLAM.PoseGraph.PGDampedFactor;
  import PGDot = SLAM.PoseGraph.PGDot;
  import PGNormalProduct = SLAM.PoseGraph.PGNormalProduct;
  import PGPrecondition = SLAM.PoseGraph.PGPrecondition;

  input Real gradient[:,6]; input Real blocks[size(gradient,1),6,6]; input Real nodeMask[size(gradient,1)];
  input Real edgeMask[:]; input Integer source[size(edgeMask,1)]; input Integer target[size(edgeMask,1)];
  input Real Ji[size(edgeMask,1),6,6]; input Real Jj[size(edgeMask,1),6,6]; input Real information[size(edgeMask,1),6,6];
  input Real damping; input Real tolerance; input Integer maximumPCG;
  output Real delta[size(gradient,1),6]; output Real iterations; output Real initialNorm; output Boolean valid;
protected Real factors[size(gradient,1),6,6]; Real diagonal[size(gradient,1),6]; Real rhs[size(gradient,1),6];
  Real localFactor[6,6]; Real localDiagonal[6];
  Real z[size(gradient,1),6]; Real direction[size(gradient,1),6]; Real product[size(gradient,1),6];
  Real rho; Real nextRho; Real denominator; Real alpha; Real beta; Real residualNorm; Boolean factorValid; Boolean running;
algorithm
  factors := zeros(size(gradient,1),6,6); diagonal := zeros(size(gradient,1),6); delta := zeros(size(gradient,1),6);
  localFactor := zeros(6,6); localDiagonal := zeros(6);
  rhs := -gradient; z := zeros(size(gradient,1),6); direction := zeros(size(gradient,1),6); product := zeros(size(gradient,1),6);
  valid := true; factorValid := false; iterations := 0.0; rho := 0.0; nextRho := 0.0; denominator := 0.0;
  alpha := 0.0; beta := 0.0; residualNorm := 0.0; running := false;
  for node in 1:size(gradient,1) loop
    if node > 1 and nodeMask[node] == 1.0 then
      (localFactor,localDiagonal,factorValid) := PGDampedFactor(blocks[node,:,:],damping);
      factors[node,:,:] := localFactor; diagonal[node,:] := localDiagonal;
      valid := valid and factorValid;
    end if;
  end for;
  z := PGPrecondition(rhs,factors,nodeMask); direction := z;
  rho := PGDot(rhs,z); initialNorm := PGDot(rhs,rhs);
  valid := valid and rho >= 0.0 and rho <= 1e100 and initialNorm <= 1e100;
  running := valid and initialNorm > 1e-18;
  for pcg in 1:96 loop
    if running and pcg <= maximumPCG then
      iterations := iterations+1.0;
      product := PGNormalProduct(direction,nodeMask,edgeMask,source,target,Ji,Jj,information,diagonal,damping);
      denominator := PGDot(direction,product); valid := denominator > 0.0 and denominator <= 1e100 and rho > 0.0;
      if valid then
        alpha := rho/denominator; delta := delta+alpha*direction; rhs := rhs-alpha*product; residualNorm := PGDot(rhs,rhs);
        valid := residualNorm >= 0.0 and residualNorm <= 1e100;
        running := valid and residualNorm > tolerance*tolerance*initialNorm;
        if running then
          z := PGPrecondition(rhs,factors,nodeMask); nextRho := PGDot(rhs,z);
          valid := nextRho > 0.0 and nextRho <= 1e100; beta := nextRho/max(rho,1e-300);
          direction := z+beta*direction; rho := nextRho; running := running and valid;
        end if;
      else running := false;
      end if;
    end if;
  end for;
end PGPCG;
