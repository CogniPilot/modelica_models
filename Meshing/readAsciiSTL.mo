within Meshing;
impure function readAsciiSTL "Read an ASCII STL facet list and convert coordinates to metres"
  input String path;
  input Real metresPerUnit=1;
  output Real triangles[:,3,3](each unit="m");
protected
  import Strings=Modelica.Utilities.Strings;
  String lines[:]=Modelica.Utilities.Streams.readFile(path);
  String keyword;
  Integer facets=0,p,row=0,corner=0;
  Boolean inFacet=false;
algorithm
  assert(metresPerUnit>0,"STL length scale must be positive");
  for line in 1:size(lines,1) loop
    (p,keyword):=Strings.Advanced.scanIdentifier(lines[line]);
    if keyword=="facet" then facets:=facets+1;end if;
  end for;
  assert(facets>0,"Expected facets in ASCII STL "+path);
  triangles:=zeros(facets,3,3);
  for line in 1:size(lines,1) loop
    (p,keyword):=Strings.Advanced.scanIdentifier(lines[line]);
    if keyword=="facet" then
      assert(not inFacet,"Nested facet in "+path);
      row:=row+1;
      corner:=0;
      inFacet:=true;
    elseif keyword=="vertex" then
      assert(inFacet and corner<3,"Unexpected vertex at line "+String(line));
      corner:=corner+1;
      for d in 1:3 loop
        (triangles[row,corner,d],p):=Strings.scanReal(lines[line],p);
        triangles[row,corner,d]:=triangles[row,corner,d]*metresPerUnit;
      end for;
    elseif keyword=="endfacet" then
      assert(inFacet and corner==3,"Incomplete triangle at line "+String(line));
      inFacet:=false;
    end if;
  end for;
  assert(not inFacet,"Unterminated facet in "+path);
end readAsciiSTL;
