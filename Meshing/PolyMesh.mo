within Meshing;
record PolyMesh "Indexed polyhedral cells with oriented faces and named boundary groups"
  Integer nPoints;
  Integer nFaces;
  Integer nCorners;
  Integer nCells;
  Real points[:,3](each unit="m");
  Integer first[:] "Face f uses corners[first[f]:first[f+1]-1]";
  Integer corners[:];
  Integer owner[:];
  Integer neighbour[:] "Zero on a boundary";
  Integer patch[:] "Zero internal. Generated Cartesian meshes use 1/2 x min/max, 3/4 y, 5/6 z and 7 body";
end PolyMesh;
