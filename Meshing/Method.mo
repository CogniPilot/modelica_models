within Meshing;
type Method=enumeration(
  snappedCartesian "Locally refined Cartesian mesh with bounded wall snapping",
  thinWallCartesian "Also capture thin solids missed by cell centres",
  levelSetCut "Clip a tetrahedral background at a signed-distance wall");
