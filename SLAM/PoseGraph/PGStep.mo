within SLAM.PoseGraph;
function PGStep
  import PGGraphCost = SLAM.PoseGraph.PGGraphCost;
  import PGLinearize = SLAM.PoseGraph.PGLinearize;
  import PGPCG = SLAM.PoseGraph.PGPCG;
  import PGRetract = SLAM.PoseGraph.PGRetract;

  input Real p[:,3]; input Real R[size(p,1),3,3]; input Real nodeMask[size(p,1)]; input Real edgeMask[:];
  input Integer source[size(edgeMask,1)]; input Integer target[size(edgeMask,1)];
  input Real translation[size(edgeMask,1),3]; input Real measuredRotation[size(edgeMask,1),3,3]; input Real information[size(edgeMask,1),6,6];
  input Real currentCost; input Real damping; input Real maximumPositionStep; input Real maximumAngleStep;
  input Real pcgTolerance; input Integer maximumPCG; input Integer maximumBacktracks;
  output Real nextP[size(p,1),3]; output Real nextR[size(p,1),3,3]; output Real nextCost; output Real nextDamping;
  output Real accepted; output Real iterations; output Boolean running;
protected Real Ji[size(edgeMask,1),6,6]; Real Jj[size(edgeMask,1),6,6]; Real gradient[size(p,1),6]; Real blocks[size(p,1),6,6];
  Real delta[size(p,1),6]; Real initialNorm; Real scale; Real positionNorm; Real angleNorm;
  Real trialP[size(p,1),3]; Real trialR[size(p,1),3,3]; Real trialCost; Boolean pcgValid; Boolean trialValid;
algorithm
  nextP := p; nextR := R; nextCost := currentCost; nextDamping := min(1e6,damping*10.0); accepted := 0.0;
  trialP := p; trialR := R; trialCost := currentCost; scale := 1.0; positionNorm := 0.0; angleNorm := 0.0; trialValid := false;
  (Ji,Jj,gradient,blocks) := PGLinearize(p,R,edgeMask,source,target,translation,measuredRotation,information);
  (delta,iterations,initialNorm,pcgValid) := PGPCG(gradient,blocks,nodeMask,edgeMask,source,target,Ji,Jj,information,damping,pcgTolerance,maximumPCG);
  running := initialNorm > 1e-18;
  for node in 1:size(p,1) loop
    positionNorm := sqrt(max(delta[node,1:3]*delta[node,1:3],0.0)); angleNorm := sqrt(max(delta[node,4:6]*delta[node,4:6],0.0));
    scale := min(scale,min(maximumPositionStep/max(positionNorm,1e-12),maximumAngleStep/max(angleNorm,1e-12)));
  end for;
  for backtrack in 1:12 loop
    if pcgValid and running and accepted == 0.0 and backtrack <= maximumBacktracks then
      (trialP,trialR) := PGRetract(p,R,nodeMask,delta,scale);
      (trialCost,trialValid) := PGGraphCost(trialP,trialR,edgeMask,source,target,translation,measuredRotation,information);
      if trialValid and trialCost < currentCost-1e-12*max(1.0,currentCost) then
        nextP := trialP; nextR := trialR; nextCost := trialCost; nextDamping := max(1e-9,damping*0.3); accepted := 1.0;
      else scale := scale*0.5;
      end if;
    end if;
  end for;
end PGStep;
