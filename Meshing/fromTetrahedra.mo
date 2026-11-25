within Meshing;
function fromTetrahedra "Import a conforming first-order tetrahedral mesh as oriented finite-volume faces"
  input Real points[:,3](each unit="m");
  input Integer tetrahedra[:,4] "One-based node indices, either winding";
  input Integer boundaryPatch[size(tetrahedra,1),4]=fill(7,size(tetrahedra,1),4)
    "Patch for the face opposite each local node; ignored for shared faces";
  output PolyMesh mesh;
protected
  Integer cells=size(tetrahedra,1),capacity=4*cells;
  Integer head[max(1,capacity)]=fill(0,max(1,capacity)),link[capacity];
  Integer corners[capacity,3],lowKey[capacity],highKey[capacity],midKey[capacity];
  Integer owner[capacity],neighbour[capacity]=fill(0,capacity),patch[capacity];
  Integer count=0,a,b,c,temp,lo,mid,hi,bucket,face,opposite;
  Real inwardTest;
algorithm
  assert(cells>0 and min(tetrahedra)>=1 and max(tetrahedra)<=size(points,1),"Invalid tetrahedral connectivity");
  for t in 1:cells loop
    for side in 1:4 loop
      opposite:=tetrahedra[t,side];
      a:=tetrahedra[t,mod(side,4)+1];
      b:=tetrahedra[t,mod(side+1,4)+1];
      c:=tetrahedra[t,mod(side+2,4)+1];
      inwardTest:=cross(points[b,:]-points[a,:],points[c,:]-points[a,:])*(points[opposite,:]-points[a,:]);
      assert(abs(inwardTest)>0,"Zero-volume tetrahedron "+String(t));
      if inwardTest>0 then
        temp:=b;
        b:=c;
        c:=temp;
      end if;
      lo:=min({a,b,c});
      hi:=max({a,b,c});
      mid:=a+b+c-lo-hi;
      bucket:=1+mod(31*mod(31*lo+mid,capacity)+hi,capacity);
      face:=head[bucket];
      while face>0 loop
        if lowKey[face]==lo and midKey[face]==mid and highKey[face]==hi then break;end if;
        face:=link[face];
      end while;
      if face==0 then
        count:=count+1;
        lowKey[count]:=lo;
        midKey[count]:=mid;
        highKey[count]:=hi;
        corners[count,:]:={a,b,c};
        owner[count]:=t;
        patch[count]:=boundaryPatch[t,side];
        link[count]:=head[bucket];
        head[bucket]:=count;
      else
        assert(neighbour[face]==0,"More than two tetrahedra share a face");
        assert((a==corners[face,1] and b==corners[face,3]) or
               (a==corners[face,2] and b==corners[face,1]) or
               (a==corners[face,3] and b==corners[face,2]),"Overlapping tetrahedra share a face");
        neighbour[face]:=t;
        patch[face]:=0;
      end if;
    end for;
  end for;
  mesh:=PolyMesh(size(points,1),count,3*count,cells,points,1:3:(3*count+1),fill(0,3*count),
    fill(0,count),fill(0,count),fill(0,count));
  for i in 1:count loop
    mesh.owner[i]:=owner[i];
    mesh.neighbour[i]:=neighbour[i];
    mesh.patch[i]:=patch[i];
    assert(neighbour[i]>0 or patch[i]>0,"Exterior tetrahedral face needs a positive patch ID");
    for j in 1:3 loop
      mesh.corners[3*(i-1)+j]:=corners[i,j];
    end for;
  end for;
  annotation(Documentation(info="<html><p>For use with indexed tetrahedra from generators such as
Gmsh HXT, CGAL Mesh_3 or fTetWild. Remap external node tags to contiguous one-based
indices and provide linear elements. Coincident nodes must share an index. Preserves supplied
boundary identifiers. CAD import and execution of the external generator are caller responsibilities.</p></html>"));
end fromTetrahedra;
