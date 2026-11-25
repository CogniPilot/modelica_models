within Meshing;
impure function saveMesh "Atomically save a complete PolyMesh as portable MATLAB v4 matrices"
  input String path;
  input PolyMesh mesh;
protected
  impure function writeMatrices
    input String path;
    input Integer np,nf,nk,nc;
    input Real points[np,3];
    input Integer first[nf+1],owner[nf],other[nf],patch[nf],corners[nk];
    external "C" mesh_write_mat(path,np,nf,nk,nc,points,first,owner,other,patch,corners)
      annotation(Include="#include \"meshing_cache.h\"",IncludeDirectory="modelica://Meshing/Resources/Include");
  end writeMatrices;
  String tmp;
algorithm
  assert(mesh.first[mesh.nFaces+1]==mesh.nCorners+1,"Invalid PolyMesh sentinel");
  tmp:=Cache.temporary(path);
  writeMatrices(tmp,mesh.nPoints,mesh.nFaces,mesh.nCorners,mesh.nCells,mesh.points,
    mesh.first,mesh.owner,mesh.neighbour,mesh.patch,mesh.corners);
  Cache.publish(tmp,path);
end saveMesh;
