within Meshing;
function loftHexMesh "Build a conforming all-hex block through an ordered stack of quadrilateral grids"
  input Real sections[:,:,:,3](each unit="m") "Coordinates indexed by chordwise, transverse, spanwise node";
  output PolyMesh mesh;
protected
  Integer n[3]={size(sections,1)-1,size(sections,2)-1,size(sections,3)-1};
  Integer ids[n[1],n[2],n[3]],index[3];
  Real volumes[:];
algorithm
  assert(min(n)>0,"Each direction of a loft needs at least two nodes");
  for k in 1:n[3] loop
    for j in 1:n[2] loop
      for i in 1:n[1] loop
        ids[i,j,k]:=i+n[1]*((j-1)+n[2]*(k-1));
      end for;
    end for;
  end for;
  mesh:=buildTopology(ids,fill(1,n[1],n[2],n[3]),product(n),zeros(3),n);
  for p in 1:mesh.nPoints loop
    for d in 1:3 loop
      index[d]:=1+integer(floor(mesh.points[p,d]+0.5));
    end for;
    mesh.points[p,:]:=sections[index[1],index[2],index[3],:];
  end for;
  volumes:=cellVolumes(mesh);
  assert(min(volumes)>0,"Loft generated an inverted cell");
  annotation(Documentation(info="<html><p>Supply corresponding quadrilateral sections in right-handed order.
Their positions may follow a bent or twisted span. Set the normal spacing of the section nodes
to prescribe wall layers. Patches 1 to 6 are the low and high boundaries of the three
index directions. Use patchNames on export to give these physical names. The caller
supplies the block layout and open or capped boundary conditions. Periodic seam
identification, automatic blocking, and high-order curved elements require further work.</p></html>"));
end loftHexMesh;
