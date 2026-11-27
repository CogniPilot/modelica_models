within Meshing;
function surfaceDistanceGrid "Sample signed distance to a closed triangle surface in a narrow band on a regular grid"
  input Real triangles[:,3,3](each unit="m");
  input Real lower[3](each unit="m"),upper[3](each unit="m");
  input Integer cells[3](each min=1);
  output Real phi[cells[1]+1,cells[2]+1,cells[3]+1](each unit="m") "Positive fluid. Distances saturate outside the band";
protected
  Integer n[3]=cells+{1,1,1};
  Real h[3]=(upper-lower)./cells,band=2*sqrt(h*h),eps=1e-10*min(h);
  Integer events[cells[1]+2,cells[2]+1,cells[3]+1]=fill(0,cells[1]+2,cells[2]+1,cells[3]+1);
  Integer lo[3],hi[3],first,parity;
  Real a[3],b[3],c[3],ab[3],ac[3],normal[3],p[3],ds,den,t;
  Real yy,zz,u,v,xhit,aa,bb,cc,bc2,dx,dy,dz,da,db,ap2,bp2,tc,td,te,q,r;
algorithm
  assert(min(cells)>=1 and min(h)>0,"Grid must have positive lengths and cell counts");
  phi:=fill(band^2,n[1],n[2],n[3]);
  for f in 1:size(triangles,1) loop
    a:=triangles[f,1,:];b:=triangles[f,2,:];c:=triangles[f,3,:];
    ab:=b-a;ac:=c-a;normal:=cross(ab,ac);den:=normal*normal;
    aa:=ab*ab;bb:=ab*ac;cc:=ac*ac;bc2:=(ac-ab)*(ac-ab);
    assert(den>0,"Degenerate surface triangle");
    for d in 1:3 loop
      assert(min(triangles[f,:,d])>lower[d] and max(triangles[f,:,d])<upper[d],"Body must lie inside grid");
      lo[d]:=max(1,1+integer(ceil((min(triangles[f,:,d])-lower[d]-band)/h[d])));
      hi[d]:=min(n[d],1+integer(floor((max(triangles[f,:,d])-lower[d]+band)/h[d])));
    end for;
    for k in lo[3]:hi[3] loop
      p[3]:=lower[3]+(k-1)*h[3];
      for j in lo[2]:hi[2] loop
        p[2]:=lower[2]+(j-1)*h[2];
        // An infinitesimal lateral shift avoids rays through shared CAD vertices.
        yy:=p[2]+0.37*eps-a[2];zz:=p[3]+0.71*eps-a[3];
        if normal[1]<>0 then
          u:=(yy*ac[3]-zz*ac[2])/normal[1];
          v:=(ab[2]*zz-ab[3]*yy)/normal[1];
          if u>=0 and v>=0 and u+v<1 then
            xhit:=a[1]+u*ab[1]+v*ac[1];
            first:=min(n[1]+1,max(1,2+integer(floor((xhit-lower[1])/h[1]))));
            events[first,j,k]:=events[first,j,k]+1;
          end if;
        end if;
        for i in lo[1]:hi[1] loop
          p[1]:=lower[1]+(i-1)*h[1];
          dx:=p[1]-a[1];dy:=p[2]-a[2];dz:=p[3]-a[3];
          t:=dx*normal[1]+dy*normal[2]+dz*normal[3];
          ds:=t*t/den;
          if ds<phi[i,j,k] then
            da:=dx*ab[1]+dy*ab[2]+dz*ab[3];db:=dx*ac[1]+dy*ac[2]+dz*ac[3];
            q:=cc*da-bb*db;r:=aa*db-bb*da;
            if q<0 or r<0 or q+r>den then
              // Squared distances to the three clamped edge projections.
              ap2:=dx*dx+dy*dy+dz*dz;
              tc:=min(1,max(0,da/aa));td:=min(1,max(0,db/cc));
              bp2:=ap2-2*da+aa;
              t:=db-da-bb+aa;te:=min(1,max(0,t/bc2));
              ds:=max(0,min({ap2-tc*(2*da-tc*aa),ap2-td*(2*db-td*cc),bp2-te*(2*t-te*bc2)}));
            end if;
            phi[i,j,k]:=min(phi[i,j,k],ds);
          end if;
        end for;
      end for;
    end for;
  end for;
  for k in 1:n[3] loop
    for j in 1:n[2] loop
      parity:=0;
      for i in 1:n[1] loop
        parity:=mod(parity+events[i,j,k],2);
        phi[i,j,k]:=if phi[i,j,k]<=eps^2 then 0 else (1-2*parity)*sqrt(phi[i,j,k]);
      end for;
      assert(mod(parity+events[n[1]+1,j,k],2)==0,"Unpaired surface crossing");
    end for;
  end for;
end surfaceDistanceGrid;
