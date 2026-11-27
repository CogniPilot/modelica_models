within Marine.Sailing;
function stayPanelContact "Frictionless finite stay against a four-column triangulated sail, in common local axes"
  input Real wire[3,3](each unit="m") "Pin, moving midpoint and hounds";
  input Real vertex[:,3](each unit="m") "Row-major sail mesh, luff to leech";
  input Real clearance(unit="m")=0.005 "Wire radius plus cloth envelope";
  input Real stiffness(unit="N/m")=800 "Effective normal compliance of the loaded cloth and cable";
  output Real sailLoad[size(vertex,1),3](each unit="N");
  output Real stayLoad[3,3](each unit="N") "Equivalent loads at the three cable nodes";
protected
  Integer ids[3],winner[3],first,next;
  Real tri[3,3],bestWeights[3],start[3],axis[3],face[3],edge[3],rel[3],delta[3],n[3],bestN[3];
  Real a,b,c,d,e,det,u,v,gap,best,bestU,signedDistance,normalSpeed,bary[3],reaction[3],point[3];
algorithm
  sailLoad:=zeros(size(vertex,1),3);
  stayLoad:=zeros(3,3);
  for half in 1:2 loop
    start:=wire[half,:];
    axis:=wire[half+1,:]-start;
    a:=axis*axis;
    best:=clearance;
    bestWeights:=zeros(3);bestN:=zeros(3);winner:={1,1,1};bestU:=0;
    for row in 1:div(size(vertex,1),4)-1 loop
      for col in 1:3 loop
        first:=(row-1)*4+col;
        for side in 1:2 loop
          ids:=if side==1 then {first,first+1,first+5} else {first,first+5,first+4};
          tri:=vertex[ids,:];
          if a>1e-16 and min({min(max(tri[:,k]),max(wire[half:half+1,k]))-
              max(min(tri[:,k]),min(wire[half:half+1,k])) for k in 1:3}) >= -clearance then
            // Closest approach occurs at an edge, a segment endpoint, or a face crossing.
            for j in 1:3 loop
              next:=mod(j,3)+1;
              edge:=tri[next,:]-tri[j,:];
              rel:=start-tri[j,:];
              b:=axis*edge;c:=edge*edge;d:=axis*rel;e:=edge*rel;
              det:=a*c-b*b;
              u:=if det>1e-14*a*c then min(1,max(0,(b*e-c*d)/det)) else 0.5;
              v:=min(1,max(0,(b*u+e)/max(c,1e-16)));
              u:=min(1,max(0,(b*v-d)/a));
              delta:=rel+u*axis-v*edge;
              gap:=sqrt(delta*delta);
              if gap<best then
                best:=gap;bestU:=u;winner:=ids;bestWeights:=zeros(3);
                bestWeights[j]:=1-v;bestWeights[next]:=v;
                bestN:=delta/max(1e-12,gap);
              end if;
            end for;
            face:=cross(tri[2,:]-tri[1,:],tri[3,:]-tri[1,:]);
            det:=face*face;
            if det>1e-20 then
              n:=face/sqrt(det);
              signedDistance:=(start-tri[1,:])*n;normalSpeed:=axis*n;
              u:=if abs(normalSpeed)>1e-12 then max(0,min(1,-signedDistance/normalSpeed)) else 0.5;
              // Test both endpoints as well, including lines parallel to the face.
              for sampleIndex in 1:3 loop
                v:=if sampleIndex==1 then u else sampleIndex-2;
                gap:=signedDistance+v*normalSpeed;
                // An interior intersection lies on the face by construction. All such
                // crossings tie at zero distance, in the declared triangle order.
                if sampleIndex==1 and abs(normalSpeed)>1e-12 and u>0 and u<1 then gap:=0; end if;
                point:=start+v*axis-gap*n;
                for j in 1:3 loop
                  bary[j]:=cross(tri[mod(j,3)+1,:]-point,tri[mod(j+1,3)+1,:]-point)*face/det;
                end for;
                if min(bary)>=-1e-9 and abs(gap)<best then
                  best:=abs(gap);bestU:=v;winner:=ids;bestWeights:=bary;
                  bestN:=(if abs(gap)>1e-10 then sign(gap) elseif signedDistance<0 then -1 else 1)*n;
                end if;
              end for;
              if best<1e-12 and bestN*bestN<0.1 then
                bestN:=(if signedDistance<0 then -1 else 1)*n;
              end if;
            end if;
          end if;
        end for;
      end for;
    end for;
    if best<clearance then
      reaction:=stiffness*(clearance-best)*bestN;
      stayLoad[half,:]:=stayLoad[half,:]+(1-bestU)*reaction;
      stayLoad[half+1,:]:=stayLoad[half+1,:]+bestU*reaction;
      for j in 1:3 loop
        sailLoad[winner[j],:]:=sailLoad[winner[j],:]-bestWeights[j]*reaction;
      end for;
    end if;
  end for;
end stayPanelContact;
