within Meshing;
function markSurfaceCells "Capture under-resolved solid walls using triangle–cell intersections"
  input Integer labels[:,:,:];
  input Real triangles[:,3,3](each unit="m"),lower[3](each unit="m"),h[3](each unit="m");
  output Integer marked[size(labels,1),size(labels,2),size(labels,3)];
protected
  Integer normalAxis,step[3],minus[3],plus[3];
  Integer n[3]=size(labels),lo[3],hi[3],count;
  Real verts[3,3],edges[3,3],basis[3,3]=identity(3),axis[3],axes[10,3];
  Real low[10],high[10],radius,proj[3],centre[3],value;
  Boolean overlap;
algorithm
  marked:=labels;
  for t in 1:size(triangles,1) loop
    for j in 1:3 loop verts[j,:]:=(triangles[t,j,:]-lower)./h;end for;
    for k in 1:3 loop
      lo[k]:=max(1,1+integer(floor(min(verts[:,k])+1e-9)));
      hi[k]:=min(n[k],integer(ceil(max(verts[:,k])-1e-9)));
      edges[k,:]:=verts[mod(k,3)+1,:]-verts[k,:];
    end for;
    // A strictly interior overlap excludes a triangle lying on a grid plane.
    // The ordinary solid-centre test already handles that aligned boundary.
    if min(hi-lo)>=0 then
      count:=1;
      axes[1,:]:=cross(edges[1,:],edges[2,:]);
      // In grid coordinates, the dominant normal identifies the two cell centres
      // across this wall. Keep resolved surfaces on the original classification.
      normalAxis:=1;
      for q in 2:3 loop
        if abs(axes[1,q])>abs(axes[1,normalAxis]) then normalAxis:=q;end if;
      end for;
      step:={0,0,0};step[normalAxis]:=1;
      for e in 1:3 loop
        for k in 1:3 loop
          axis:=cross(edges[e,:],basis[k,:]);
          if axis*axis>0 then count:=count+1;axes[count,:]:=axis;end if;
        end for;
      end for;
      for k in 1:count loop
        axis:=axes[k,:];
        proj:=verts*axis;
        radius:=(0.5-1e-9)*sum(abs(axis));
        low[k]:=min(proj)-radius;
        high[k]:=max(proj)+radius;
      end for;
      for k in lo[3]:hi[3] loop
        for j in lo[2]:hi[2] loop
          for i in lo[1]:hi[1] loop
            if marked[i,j,k]>=0 then
              centre:={i-0.5,j-0.5,k-0.5};
              overlap:=true;
              for q in 1:count loop
                value:=axes[q,:]*centre;
                if value<=low[q] or value>=high[q] then overlap:=false;break;end if;
              end for;
              if overlap then
                minus:={max(1,i-step[1]),max(1,j-step[2]),max(1,k-step[3])};
                plus:={min(n[1],i+step[1]),min(n[2],j+step[2]),min(n[3],k+step[3])};
                if labels[minus[1],minus[2],minus[3]]>=0 and labels[plus[1],plus[2],plus[3]]>=0 then
                  marked[i,j,k]:=-1;
                end if;
              end if;
            end if;
          end for;
        end for;
      end for;
    end if;
  end for;
end markSurfaceCells;
