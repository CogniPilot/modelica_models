within SLAM.PoseGraph;
function OptimizeModelicaPoseGraph
  import PGGraphCost = SLAM.PoseGraph.PGGraphCost;
  import PGRun = SLAM.PoseGraph.PGRun;
  import PGValidateGraph = SLAM.PoseGraph.PGValidateGraph;

  input Real position[:,3]; input Real rotation[size(position,1),3,3]; input Real nodeMask[size(position,1)];
  input Real edgeMask[:]; input Real fromNode[size(edgeMask,1)]; input Real toNode[size(edgeMask,1)];
  input Real translation[size(edgeMask,1),3]; input Real measuredRotation[size(edgeMask,1),3,3]; input Real information[size(edgeMask,1),6,6];
  input Integer maximumIterations; input Integer maximumPCG; input Integer maximumBacktracks;
  input Real initialDamping; input Real maximumPositionStep; input Real maximumAngleStep; input Real pcgTolerance;
  output Real nextPosition[size(position,1),3]; output Real nextRotation[size(position,1),3,3]; output Real status;
  output Real costBefore; output Real costAfter; output Real acceptedIterations; output Real pcgIterations; output Real activeNodes; output Real activeEdges;
protected Integer source[size(edgeMask,1)]; Integer target[size(edgeMask,1)]; Boolean valid; Boolean costValid;
algorithm
  nextPosition := position; nextRotation := rotation; costBefore := 0.0; costAfter := 0.0; acceptedIterations := 0.0; pcgIterations := 0.0;
  (source,target,status,activeNodes,activeEdges) := PGValidateGraph(position,rotation,nodeMask,edgeMask,fromNode,toNode,translation,measuredRotation,information);
  valid := status == 1.0 and maximumIterations >= 1 and maximumIterations <= 16 and maximumPCG >= 1 and maximumPCG <= 96
    and maximumBacktracks >= 1 and maximumBacktracks <= 12 and initialDamping >= 1e-9 and initialDamping <= 1e3
    and maximumPositionStep > 0.0 and maximumPositionStep <= 2.0 and maximumAngleStep > 0.0 and maximumAngleStep <= 0.5
    and pcgTolerance >= 1e-8 and pcgTolerance <= 0.1;
  status := if status == 1.0 and not valid then -1.0 else status; costValid := false;
  if valid then
    (costBefore,costValid) := PGGraphCost(position,rotation,edgeMask,source,target,translation,measuredRotation,information);
    status := if costValid then 1.0 else -4.0; costAfter := costBefore;
    if costValid then
      (nextPosition,nextRotation,costAfter,acceptedIterations,pcgIterations) := PGRun(position,rotation,nodeMask,edgeMask,source,target,
        translation,measuredRotation,information,costBefore,initialDamping,maximumPositionStep,maximumAngleStep,pcgTolerance,maximumIterations,maximumPCG,maximumBacktracks);
      status := if acceptedIterations > 0.0 then 2.0 else 1.0;
    end if;
  end if;
end OptimizeModelicaPoseGraph;
