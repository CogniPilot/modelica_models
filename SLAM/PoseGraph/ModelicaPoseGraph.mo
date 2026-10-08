within SLAM.PoseGraph;
model ModelicaPoseGraph
  import OptimizeModelicaPoseGraph = SLAM.PoseGraph.OptimizeModelicaPoseGraph;

  parameter Integer nodeCapacity = 128; parameter Integer edgeCapacity = 256;
  parameter Integer maximumIterations = 8; parameter Integer maximumPCG = 48; parameter Integer maximumBacktracks = 8;
  parameter Real initialDamping = 0.001; parameter Real maximumPositionStep = 0.5;
  parameter Real maximumAngleStep = 0.2; parameter Real pcgTolerance = 1e-5;
  input Real position[nodeCapacity,3] = zeros(nodeCapacity,3);
  input Real rotation[nodeCapacity,3,3] = zeros(nodeCapacity,3,3);
  input Real nodeMask[nodeCapacity] = zeros(nodeCapacity); input Real edgeMask[edgeCapacity] = zeros(edgeCapacity);
  input Real fromNode[edgeCapacity] = ones(edgeCapacity); input Real toNode[edgeCapacity] = ones(edgeCapacity);
  input Real translation[edgeCapacity,3] = zeros(edgeCapacity,3);
  input Real measuredRotation[edgeCapacity,3,3] = zeros(edgeCapacity,3,3);
  input Real information[edgeCapacity,6,6] = zeros(edgeCapacity,6,6);
  output Real nextPosition[nodeCapacity,3]; output Real nextRotation[nodeCapacity,3,3];
  output Real status; output Real costBefore; output Real costAfter; output Real acceptedIterations;
  output Real pcgIterations; output Real activeNodes; output Real activeEdges;
equation
  (nextPosition,nextRotation,status,costBefore,costAfter,acceptedIterations,pcgIterations,activeNodes,activeEdges) =
    OptimizeModelicaPoseGraph(position,rotation,nodeMask,edgeMask,fromNode,toNode,translation,measuredRotation,information,
      maximumIterations,maximumPCG,maximumBacktracks,initialDamping,maximumPositionStep,maximumAngleStep,pcgTolerance);
end ModelicaPoseGraph;
