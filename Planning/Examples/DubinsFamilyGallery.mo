within Planning.Examples;
model DubinsFamilyGallery "All six feasible families for a common boundary condition"
  Planning.Examples.DubinsFamilySample lsl(
    pathType=Planning.Dubins.PathType.LSL);
  Planning.Examples.DubinsFamilySample rsr(
    pathType=Planning.Dubins.PathType.RSR);
  Planning.Examples.DubinsFamilySample lsr(
    pathType=Planning.Dubins.PathType.LSR);
  Planning.Examples.DubinsFamilySample rsl(
    pathType=Planning.Dubins.PathType.RSL);
  Planning.Examples.DubinsFamilySample rlr(
    pathType=Planning.Dubins.PathType.RLR);
  Planning.Examples.DubinsFamilySample lrl(
    pathType=Planning.Dubins.PathType.LRL);
  annotation(experiment(StartTime=0.0, StopTime=1.0, Interval=0.0025));
end DubinsFamilyGallery;
