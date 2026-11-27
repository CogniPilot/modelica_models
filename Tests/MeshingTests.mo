within Tests;
model MeshingTests "Verify conforming refinement, body patch extraction and wall-layer volume conservation"
  function check
    output Boolean passed;
  protected
    Real points[8,3]={{1,1,1},{2,1,1},{2,2,1},{1,2,1},{1,1,2},{2,1,2},{2,2,2},{1,2,2}};
    Integer faces[12,3]={{1,3,2},{1,4,3},{5,6,7},{5,7,8},{1,2,6},{1,6,5},{2,3,7},{2,7,6},{3,4,8},{3,8,7},{4,1,5},{4,5,8}};
    Real planePhi[3,3,3],distances[7,7,7],reversed[7,7,7];
    Real triangles[12,3,3],plate[12,3,3],pair[24,3,3],v[:];
    Meshing.PolyMesh m,moved;
    Real limited[:,:];
    Real sections[3,3,4,3],angle;
    Real swept[:],change[:],vEnd[:];
    Integer walls=0,base, gapLabels[5,5,5], repaired[5,5,5];
    Boolean crease[12];
    Real sourceArea,wallArea,sourceVolume,wallVolume;
  algorithm
    // A slanted cut through the unit cube leaves exactly half of its volume.
    for i in 1:3 loop
      for j in 1:3 loop
        for k in 1:3 loop planePhi[i,j,k]:=(i+j+k-6)/2.0;end for;
      end for;
    end for;
    m:=Meshing.levelSetMesh({0,0.5,1},{0,0.5,1},{0,0.5,1},planePhi);
    v:=Meshing.cellVolumes(m,true);
    assert(abs(sum(v)-0.5)<1e-12 and Meshing.bodyComponentCount(m)==1,"Linear interface with exact-zero nodes");
    // The resolved fluid lane between two plates is already manifold.
    gapLabels:=fill(0,5,5,5);
    gapLabels[2,:,:]:=fill(-1,5,5);
    gapLabels[4,:,:]:=fill(-1,5,5);
    repaired:=Meshing.regularizeSurface(gapLabels);
    assert(min(repaired[3,:,:])==0,"Topology repair closed a resolved one-cell channel");
    for i in 1:12 loop
      for j in 1:3 loop
        triangles[i,j,:]:=points[faces[i,j],:];
      end for;
    end for;
    distances:=Meshing.surfaceDistanceGrid(triangles,{0,0,0},{3,3,3},{6,6,6});
    for f in 1:12 loop plate[f,:,:]:=triangles[13-f,:,:];end for;
    reversed:=Meshing.surfaceDistanceGrid(plate,{0,0,0},{3,3,3},{6,6,6});
    assert(abs(distances[4,4,4]+0.5)+abs(distances[6,4,4]-0.5)+abs(distances[3,4,4])<1e-12,
      "Incorrect signed distance to the cube");
    for k in 1:7 loop
      for j in 1:7 loop
        for i in 1:7 loop
          assert(abs(distances[i,j,k]-reversed[i,j,k])<1e-12,"Facet order changed the nearest signed distance");
        end for;
      end for;
    end for;
    // Both faces of this thin plate lie between adjacent cell centres.
    plate:=triangles;
    plate[:,:,1]:=1.08 .+ 0.08*(triangles[:,:,1] .- 1.0);
    m:=Meshing.generate({0,0,0},{4,4,4},{8,8,8},0,plate,0,captureThinSolids=true);
    v:=Meshing.cellVolumes(m,true);
    assert(abs(sum(v)-63.5)<1e-8,"The intersected cell layer must capture the thin plate");
    assert(Meshing.bodyComponentCount(m)==1,"The thin plate must remain one connected shell");
    moved:=m;
    moved.points:=Meshing.stretchFarField(m,{0.5,0.5,0.5},{3,3,3},{-1,-1,-1},{5,5,5});
    v:=Meshing.cellVolumes(moved,true);
    assert(abs(sum(v)-215.5)<1e-8,"Far-field expansion changed the near-body volume");
    pair[1:12,:,:]:=plate;
    pair[13:24,:,:]:=plate;
    pair[13:24,:,1]:=pair[13:24,:,1] .+ 1.5;
    m:=Meshing.generate({0,0,0},{4,4,4},{8,8,8},0,pair,0,captureThinSolids=true);
    assert(Meshing.bodyComponentCount(m)==2,"Distinct thin plates must remain separate");
    m:=Meshing.surfacePrismMesh(points,faces,{0.01,0.025,0.05});
    (sourceArea,wallArea,sourceVolume,wallVolume):=Meshing.bodySurfaceMetrics(m,triangles);
    assert(m.nCells==36 and abs(sourceArea-wallArea)+abs(sourceVolume-wallVolume)<1e-12,
      "Prism layers must retain the original cube wall and enclose one cell per face per layer");
    crease:=Meshing.surfaceFeatureFaces(triangles,Modelica.Constants.pi/4);
    assert(sum(if value then 1 else 0 for value in crease)==12,"Each cube triangle borders a crease");
    crease:=Meshing.surfaceFeatureFaces(triangles,0.51*Modelica.Constants.pi);
    assert(sum(if value then 1 else 0 for value in crease)==0,"Coplanar and right-angle edges lie below this cutoff");
    m:=Meshing.generate({0,0,0},{4,4,4},{8,8,8},1,triangles,0);
    v:=Meshing.cellVolumes(m,checkFacePyramids=true);
    assert(abs(sum(v)-63)<1e-8 and min(v)>0,"Refinement changed the unit cube volume");
    for f in 1:m.nFaces loop
      if m.patch[f]==7 then
        walls:=walls+1;
      end if;
    end for;
    assert(walls==96 and max(v)>7.9*min(v),"Incorrect body patch or refinement");
    (sourceArea,wallArea,sourceVolume,wallVolume):=Meshing.bodySurfaceMetrics(m,triangles);
    assert(abs(sourceArea-6)+abs(wallArea-6)+abs(sourceVolume-1)+abs(wallVolume-1)<1e-10,
      "Translated cube must retain its surface area and enclosed solid volume");
    base:=m.nCells;
    m:=Meshing.addWallLayers(m,2,0.04,1.25);
    v:=Meshing.cellVolumes(m,checkFacePyramids=true);
    assert(m.nCells==base+2*walls and min(v)>0 and abs(sum(v)-63)<1e-8,"Layer topology failed to conserve volume");
    m:=Meshing.generate({0,0,0},{4,4,4},{4,4,4},1,fill(0.0,0,3,3),0,
      {{{1.5,1.5,1.5},{2.5,2.5,2.5}}},{1});
    v:=Meshing.cellVolumes(m,checkFacePyramids=true);
    assert(m.nCells>64 and max(v)>8*min(v) and abs(sum(v)-64)<1e-8,
      "Volume refinement should preserve volume and leave coarse cells outside the region");
    assert(Meshing.bodyComponentCount(m)==0,"The empty mesh should have no solid wall");
    // A nonuniform bending displacement warps the faces as well as translating them.
    moved:=m;
    for i in 1:m.nPoints loop
      moved.points[i,1]:=m.points[i,1]+0.2;
      moved.points[i,3]:=m.points[i,3]+0.02*m.points[i,1]*m.points[i,2]*(1+m.points[i,3]);
    end for;
    swept:=Meshing.faceSweptVolumes(m,moved.points);
    change:=zeros(m.nCells);
    for f in 1:m.nFaces loop
      change[m.owner[f]]:=change[m.owner[f]]+swept[f];
      if m.neighbour[f]>0 then
        change[m.neighbour[f]]:=change[m.neighbour[f]]-swept[f];
      end if;
    end for;
    vEnd:=Meshing.cellVolumes(moved);
    assert(max(abs(vEnd-v-change))<1e-10 and max(abs(change))>0.001,
      "Swept faces must recover each cell's volume change under bending");
    // An inverted end cell must backtrack without moving the far end of the box.
    m:=Meshing.generate({0,0,0},{3,1,1},{3,1,1},0,fill(0.0,0,3,3),0);
    moved:=m;
    for i in 1:m.nPoints loop
      if m.points[i,1]==0 then moved.points[i,1]:=1.5;
      elseif m.points[i,1]==3 then moved.points[i,1]:=3.1;end if;
    end for;
    limited:=Meshing.limitHexSnap(m,moved.points);
    for i in 1:m.nPoints loop
      if m.points[i,1]==0 then assert(limited[i,1]>0 and limited[i,1]<1,"Invalid local snap");
      elseif m.points[i,1]==3 then assert(abs(limited[i,1]-3.1)<1e-12,"A distant cell lost its requested snap");end if;
    end for;
    for k in 1:4 loop
      angle:=(k-1)*0.08;
      for j in 1:3 loop
        for i in 1:3 loop
          sections[i,j,k,:]:={(i-1)*cos(angle)-(j-1)*sin(angle),
            (i-1)*sin(angle)+(j-1)*cos(angle),0.5*(k-1)};
        end for;
      end for;
    end for;
    m:=Meshing.loftHexMesh(sections);
    v:=Meshing.cellVolumes(m,checkFacePyramids=true);
    assert(m.nCells==12 and m.nPoints==36 and min(v)>0.49 and max(v)<0.51,
      "Twisted section loft must keep the prescribed block topology and spacing");
    m:=Meshing.fromTetrahedra({{0,0,0},{1,0,0},{0,1,0},{0,0,1},{0,0,-1}},
      {{1,2,3,4},{1,2,3,5}},{{1,1,1,0},{2,2,2,0}});
    v:=Meshing.cellVolumes(m,checkFacePyramids=true);
    assert(m.nFaces==7 and sum(m.neighbour)==2 and max(abs(v-fill(1/6,2)))<1e-12 and sum(m.patch)==9,
      "Imported tetrahedra must share one face and retain external patch IDs and volumes");
    passed:=true;
  end check;
initial algorithm
  assert(check(),"Volume meshing regression");
end MeshingTests;
