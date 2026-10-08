within SLAM.PoseGraph;
function PGRun
  import PGStep = SLAM.PoseGraph.PGStep;

  input Real p[:,3]; input Real R[size(p,1),3,3]; input Real nodeMask[size(p,1)]; input Real edgeMask[:];
  input Integer source[size(edgeMask,1)]; input Integer target[size(edgeMask,1)];
  input Real translation[size(edgeMask,1),3]; input Real measuredRotation[size(edgeMask,1),3,3]; input Real information[size(edgeMask,1),6,6];
  input Real initialCost; input Real initialDamping; input Real maximumPositionStep; input Real maximumAngleStep; input Real pcgTolerance;
  input Integer maximumIterations; input Integer maximumPCG; input Integer maximumBacktracks;
  output Real nextP[size(p,1),3]; output Real nextR[size(p,1),3,3]; output Real cost; output Real acceptedIterations; output Real pcgIterations;
protected Real damping; Real accepted; Real iterations; Boolean running;
algorithm
  nextP := p; nextR := R; cost := initialCost; acceptedIterations := 0.0; pcgIterations := 0.0;
  damping := initialDamping; accepted := 0.0; iterations := 0.0; running := true;
  for iteration in 1:16 loop
    if running and iteration <= maximumIterations then
      (nextP,nextR,cost,damping,accepted,iterations,running) := PGStep(nextP,nextR,nodeMask,edgeMask,source,target,
        translation,measuredRotation,information,cost,damping,maximumPositionStep,maximumAngleStep,pcgTolerance,maximumPCG,maximumBacktracks);
      acceptedIterations := acceptedIterations+accepted; pcgIterations := pcgIterations+iterations;
    end if;
  end for;
end PGRun;
