within Meshing;
function levelSetMesh "Conform a rectilinear tetrahedral background to the zero surface of a nodal field"
  input Real x[:](each unit="m"),y[:](each unit="m"),z[:](each unit="m");
  input Real phi[size(x,1),size(y,1),size(z,1)] "Positive outside the solid";
  output PolyMesh mesh;
protected
  Integer nx=size(x,1),ny=size(y,1),nz=size(z,1),np=nx*ny*nz,nt=6*(nx-1)*(ny-1)*(nz-1);
  Integer step[3]={1,nx,nx*ny};
  Integer permutations[6,3]={{1,2,3},{1,3,2},{2,1,3},{2,3,1},{3,1,2},{3,2,1}};
  Integer tets[nt,4],patch[nt,4]=fill(0,nt,4),corner,t=0,d;
  Real points[np,3],field[np];
algorithm
  assert(min({nx,ny,nz})>=2 and min(x[2:nx]-x[1:nx-1])>0 and
    min(y[2:ny]-y[1:ny-1])>0 and min(z[2:nz]-z[1:nz-1])>0,"Grid axes must increase strictly");
  for k in 1:nz loop
    for j in 1:ny loop
      for i in 1:nx loop
        corner:=i+nx*((j-1)+ny*(k-1));
        points[corner,:]:={x[i],y[j],z[k]};field[corner]:=phi[i,j,k];
        if i<nx and j<ny and k<nz then
          for p in 1:6 loop
            t:=t+1;tets[t,1]:=corner;
            for side in 1:3 loop
              tets[t,side+1]:=tets[t,side]+step[permutations[p,side]];
            end for;
            // Only the faces opposite the two ends can lie on a box boundary.
            d:=permutations[p,1];
            if (d==1 and i==nx-1) or (d==2 and j==ny-1) or (d==3 and k==nz-1) then patch[t,1]:=2*d;end if;
            d:=permutations[p,3];
            if (d==1 and i==1) or (d==2 and j==1) or (d==3 and k==1) then patch[t,4]:=2*d-1;end if;
          end for;
        end if;
      end for;
    end for;
  end for;
  mesh:=clipTetrahedra(points,tets,field,patch);
end levelSetMesh;
