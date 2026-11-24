within Meshing;
function regularizeSurface "Resolve voxel contacts that would produce a nonmanifold wall or a one-cell fluid slit"
  input Integer labels[:,:,:] "Negative inside the solid, zero in fluid";
  output Integer repaired[size(labels,1),size(labels,2),size(labels,3)];
protected
  Integer offsets[8,3]={{0,0,0},{1,0,0},{0,1,0},{1,1,0},{0,0,1},{1,0,1},{0,1,1},{1,1,1}};
  Integer quads[6,4]={{1,2,4,3},{5,6,8,7},{1,2,6,5},{3,4,8,7},{1,3,7,5},{2,4,8,6}};
  Integer n[3]=size(labels),pos[3],choice,score,best,solid,other,queue[8],head,tail;
  Boolean inside[8],seen[8],bad,changed=true;
algorithm
  repaired:=labels;
  while changed loop
    changed:=false;
    for k in 1:n[3]-1 loop
      for j in 1:n[2]-1 loop
        for i in 1:n[1]-1 loop
          solid:=0;
          for q in 1:8 loop
            inside[q]:=repaired[i+offsets[q,1],j+offsets[q,2],k+offsets[q,3]]<0;
            if inside[q] then
              solid:=solid+1;
            end if;
          end for;
          if solid>=2 and solid<=6 then
            bad:=false;
            for f in 1:6 loop
              if inside[quads[f,1]]==inside[quads[f,3]] and inside[quads[f,2]]==inside[quads[f,4]]
                  and inside[quads[f,1]]<>inside[quads[f,2]] then
                    bad:=true;
                    break;
                  end if;
            end for;
            // A vertex with two disconnected fans is also nonmanifold, even
            // when every incident edge is shared by exactly two wall faces.
            for phase in 0:1 loop
              if bad then break;end if;
              seen:=fill(false,8);
              head:=1;
              tail:=0;
              for q in 1:8 loop
                if inside[q]==(phase==1) then
                  queue[1]:=q;
                  tail:=1;
                  seen[q]:=true;
                  break;
                end if;
              end for;
              while head<=tail loop
                for step in {1,2,4} loop
                  other:=if mod(div(queue[head]-1,step),2)==0 then
                    queue[head]+step else queue[head]-step;
                  if not seen[other] and inside[other]==(phase==1) then
                    tail:=tail+1;
                    queue[tail]:=other;
                    seen[other]:=true;
                  end if;
                end for;
                head:=head+1;
              end while;
              bad:=tail<>(if phase==1 then
                solid else 8-solid);
            end for;
            if bad then
              best:=-1;
              choice:=1;
              for q in 1:8 loop
                if not inside[q] then
                  score:=0;
                  for step in {1,2,4} loop
                    other:=if mod(div(q-1,step),2)==0 then
                      q+step else q-step;
                    if inside[other] then
                      score:=score+1;
                    end if;
                  end for;
                  if score>best then
                    best:=score;
                    choice:=q;
                  end if;
                end if;
              end for;
              pos:={i,j,k}+offsets[choice,:];
              repaired[pos[1],pos[2],pos[3]]:=-1;
              changed:=true;
            end if;
          end if;
        end for;
      end for;
    end for;
    for k in 2:n[3]-1 loop
      for j in 2:n[2]-1 loop
        for i in 2:n[1]-1 loop
          if repaired[i,j,k]==0 and ((repaired[i-1,j,k]<0 and repaired[i+1,j,k]<0)
              or (repaired[i,j-1,k]<0 and repaired[i,j+1,k]<0)
              or (repaired[i,j,k-1]<0 and repaired[i,j,k+1]<0)) then
            repaired[i,j,k]:=-1;
            changed:=true;
          end if;
        end for;
      end for;
    end for;
  end while;
end regularizeSurface;
