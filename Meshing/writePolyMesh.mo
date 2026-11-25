within Meshing;
impure function writePolyMesh "Write a native OpenFOAM ASCII polyMesh to a new or empty directory"
  input String directory;
  input PolyMesh mesh;
  input String patchNames[:]={"xmin","xmax","ymin","ymax","zmin","zmax","body"};
  input String patchTypes[size(patchNames,1)]={if i==7 then "wall" else "patch" for i in 1:size(patchNames,1)};
protected
  import S=Modelica.Utilities.Streams;
  import F=Modelica.Utilities.Files;
  Integer groups=size(patchNames,1)+1;
  String objects[5]={"points","faces","owner","neighbour","boundary"};
  String types[5]={"vectorField","faceList","labelList","labelList","polyBoundaryMesh"};
  String paths[5],line;
  Integer sizes[groups]=fill(0,groups),startFace[groups],cursor[groups],order[mesh.nFaces]=fill(0,mesh.nFaces),slot,nPatches=0;
  Integer bucket[mesh.nCells]=fill(0,mesh.nCells),j,key;
algorithm
  assert(min(mesh.patch)>=0 and max(mesh.patch)<groups,"Supply a name and type for each boundary patch ID");
  F.createDirectory(directory);
  for i in 1:5 loop
    paths[i]:=directory+"/"+objects[i];
    assert(not F.exist(paths[i]),"Mesh object already exists: "+paths[i]);
  end for;
  for f in 1:mesh.nFaces loop
    slot:=mesh.patch[f]+1;
    sizes[slot]:=sizes[slot]+1;
    if slot==1 then
      bucket[mesh.owner[f]]:=bucket[mesh.owner[f]]+1;
    end if;
  end for;
  startFace[1]:=0;
  for i in 2:groups loop
    startFace[i]:=startFace[i-1]+sizes[i-1];
    if sizes[i]>0 then
      nPatches:=nPatches+1;
    end if;
  end for;
  // Internal faces are upper triangular, sorted by (owner, neighbour).
  j:=0;
  for c in 1:mesh.nCells loop
    key:=bucket[c];
    bucket[c]:=j;
    j:=j+key;
  end for;
  cursor:=startFace;
  for f in 1:mesh.nFaces loop
    slot:=mesh.patch[f]+1;
    if slot==1 then
      key:=mesh.owner[f];
      bucket[key]:=bucket[key]+1;
      j:=bucket[key];
      while j>1 loop
        if order[j-1]==0 then break;end if;
        if mesh.owner[order[j-1]]<>key or mesh.neighbour[order[j-1]]<=mesh.neighbour[f] then break;end if;
        order[j]:=order[j-1];
        j:=j-1;
      end while;
      order[j]:=f;
    else
      cursor[slot]:=cursor[slot]+1;
      order[cursor[slot]]:=f;
    end if;
  end for;
  for i in 1:5 loop
    S.print("FoamFile { version 2.0; format ascii; class "+types[i]+"; object "+objects[i]+"; }",paths[i]);
    S.print(String(if i==1 then mesh.nPoints elseif i==4 then sizes[1] elseif i==5 then nPatches else mesh.nFaces)+"\n(",paths[i]);
  end for;
  for i in 1:mesh.nPoints loop
    S.print("("+String(mesh.points[i,1],significantDigits=16)+" "+String(mesh.points[i,2],significantDigits=16)+" "+String(mesh.points[i,3],significantDigits=16)+")",paths[1]);
  end for;
  for i in 1:mesh.nFaces loop
    slot:=order[i];line:=String(mesh.first[slot+1]-mesh.first[slot])+"(";
    for j in mesh.first[slot]:mesh.first[slot+1]-1 loop
      line:=line+String(mesh.corners[j]-1)+" ";
    end for;
    S.print(line+")",paths[2]);S.print(String(mesh.owner[slot]-1),paths[3]);
    if mesh.neighbour[slot]>0 then S.print(String(mesh.neighbour[slot]-1),paths[4]);end if;
  end for;
  for i in 1:groups-1 loop
    if sizes[i+1]>0 then
      S.print(patchNames[i]+" { type "+patchTypes[i]+"; nFaces "+String(sizes[i+1])+"; startFace "+String(startFace[i+1])+"; }",paths[5]);
    end if;
  end for;
  for i in 1:5 loop S.print(")",paths[i]);S.close(paths[i]);end for;
end writePolyMesh;
