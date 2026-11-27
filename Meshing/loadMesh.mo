within Meshing;
impure function loadMesh "Load coordinates, cell ownership, boundary patches and polygon connectivity without regenerating the mesh"
  import Modelica.Utilities.Streams;
  input String path;
  output PolyMesh mesh;
protected
  Real header[1,5]=Streams.readRealMatrix(path,"header",1,5);
  Integer np=integer(header[1,2]),nf=integer(header[1,3]),nk=integer(header[1,4]),nc=integer(header[1,5]);
  Real faces[:,:],corners[:,:];
  Integer first,owner,other,patch,point;
algorithm
  assert(header[1,2]==np and header[1,3]==nf and header[1,4]==nk and header[1,5]==nc,
    "Nonintegral mesh dimensions");
  assert(header[1,1]==1 and np>=4 and nf>=4 and nk>=3*nf and nc>=1,"Invalid or unsupported mesh header");
  assert(max(abs(Streams.readMatrixSize(path,"points")-{np,3}))==0 and max(abs(Streams.readMatrixSize(path,"faces")-{nf,4}))==0
    and max(abs(Streams.readMatrixSize(path,"corners")-{nk,1}))==0,"Mesh dimensions disagree with the header");
  mesh:=PolyMesh(np,nf,nk,nc,Streams.readRealMatrix(path,"points",np,3),
    fill(0,nf+1),fill(0,nk),fill(0,nf),fill(0,nf),fill(0,nf));
  faces:=Streams.readRealMatrix(path,"faces",nf,4);
  for f in 1:nf loop
    first:=integer(faces[f,1]);
    owner:=integer(faces[f,2]);
    other:=integer(faces[f,3]);
    patch:=integer(faces[f,4]);
    assert(faces[f,1]==first and faces[f,2]==owner and faces[f,3]==other and faces[f,4]==patch,"Nonintegral face record");
    assert(if f==1 then first==1 else first>=mesh.first[f-1]+3,"Invalid polygon offset");
    assert(first<=nk-2 and owner>=1 and owner<=nc and other>=0 and other<=nc and owner<>other and
      (if other==0 then patch>0 else patch==0),"Invalid cell ownership or boundary patch");
    mesh.first[f]:=first;
    mesh.owner[f]:=owner;
    mesh.neighbour[f]:=other;
    mesh.patch[f]:=patch;
  end for;
  mesh.first[nf+1]:=nk+1;
  corners:=Streams.readRealMatrix(path,"corners",nk,1);
  for j in 1:nk loop
    point:=integer(corners[j,1]);
    assert(point>=1 and point<=np and corners[j,1]==point,"Invalid corner index");
    mesh.corners[j]:=point;
  end for;
end loadMesh;
