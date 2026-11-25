within Marine.Environment;
model SwimmerWake "Moving free-surface patch driven by displaced swimmer volume"
  constant Integer n=64;
  parameter Real cell(unit="m")=.42;
  input Real sources[12,5] "ENU east, north, displaced volume, kernel width, waterline contact";
  input Real clock, restart, impact[5] "Impact time, ENU, downward entry speed";
  discrete Real prior[12,5](each start=0,each fixed=true);
  discrete Real stamp(start=-100,fixed=true),lastImpact(start=-100,fixed=true);
  discrete output Real center[2](each start=0,each fixed=true);
  discrete output Real active(start=0,fixed=true);
  discrete output Real field[3,n,n](each start=0,each fixed=true) "Elevation (m), vertical velocity (m/s), foam coverage";

  // Keep the inner writes row-sized in the generated FMI solver.
  function stampRow
    input Real priorRow[n],x,y,r2,scale;
    input Integer j,channel;
    output Real row[n];
  protected
    Real w;
  algorithm
    row:=priorRow;
    for i in max(2,integer(ceil(x-sqrt(r2)))):min(n-1,integer(floor(x+sqrt(r2)))) loop
      w:=max(0,1-((i-x)^2+(j-y)^2)/r2)^2;
      row[i]:=if channel==1 then row[i]+scale*w else min(.92,row[i]+scale*w);
    end for;
  end stampRow;

  function impactRow
    input Real priorRow[n],x,y,cell,scale;
    input Integer j,channel;
    output Real row[n];
  protected
    Real d2;
  algorithm
    row:=priorRow;
    for i in 2:n-1 loop
      d2:=((x-i)^2+(y-j)^2)*cell^2;
      if d2<36 then
        row[i]:=if channel==1 then row[i]+scale*(.27*exp(-(sqrt(d2)-2.8)^2/.62)-.16*exp(-d2/3))
                else max(row[i],scale*.9*exp(-d2/13));
      end if;
    end for;
  end impactRow;
  function carryRow
    input Real old[n],shift;
    input Boolean enabled;
    output Real row[n];
  protected
    Integer dx;
  algorithm
    dx:=integer(shift);
    row:={if enabled and i>1 and i<n and i+dx>1 and i+dx<n then old[max(1,min(n,i+dx))] else 0 for i in 1:n};
  end carryRow;

  function assembledRow
    input Real old[n],stamps[24,5],impactCell[3],cell;
    input Integer dx,j,channel;
    input Boolean carry;
    output Real row[n];
  algorithm
    row:=carryRow(old,dx,carry);
    if channel==2 or j==1 or j==n then return;end if;
    for k in 1:24 loop
      if abs(j-stamps[k,2])<stamps[k,3] and stamps[k,if channel==1 then 4 else 5]<>0 then
        row:=stampRow(row,stamps[k,1],stamps[k,2],stamps[k,3]^2,stamps[k,if channel==1 then 4 else 5],j,channel);
      end if;
    end for;
    if impactCell[3]>0 and abs(j-impactCell[2])*cell<6 then
      row:=impactRow(row,impactCell[1],impactCell[2],cell,impactCell[3],j,channel);
    end if;
  end assembledRow;

  function velocityRow
    input Real velocity[n],before[n],here[n],after[n],h,rate;
    input Integer j;
    output Real row[n];
  algorithm
    row:={if min(i,j)>1 and max(i,j)<n then
      (velocity[i]+rate*(here[max(1,i-1)]+here[min(n,i+1)]+before[i]+after[i]-4*here[i]))/
      (1+h*(.55+max(0,6-min(min(i,j),min(n-i+1,n-j+1))))) else 0 for i in 1:n};
  end velocityRow;

  function advancePatch
    input Real old[3,n,n],before[12,5],now[12,5],origin[2],previousOrigin[2],cell,dt,remaining,entry[5],trigger;
    output Real grid[3,n,n];
    output Real life;
  protected
    Integer dx,dy,steps;
    Real x,y,radius,speed,change,foam,wet,h,stamps[24,5],impactCell[3],height[n,n],velocity[n,n],foamField[n,n];
  algorithm
    grid:=zeros(3,n,n);
    life:=max(0,remaining-dt);
    if dt<=0 or dt>.6 then life:=0;return;end if;
    dx:=integer(floor((origin[1]-previousOrigin[1])/cell+.5));
    dy:=integer(floor((origin[2]-previousOrigin[2])/cell+.5));
    if abs(dx)>=n or abs(dy)>=n then life:=0;return;end if;
    stamps:=zeros(24,5);
    for k in 1:12 loop
      if max(now[k,3],before[k,3])>.0003 then
        speed:=sqrt((now[k,1]-before[k,1])^2+(now[k,2]-before[k,2])^2)/dt;
        change:=abs(now[k,3]-before[k,3])/dt;
        foam:=now[k,5]*min(1,.32*speed+change)*min(1,now[k,3]/max(.00001,.12*now[k,4]^2));
        if foam>.015 or change>.012 then life:=5;end if;
        if life>0 then
          for pass in 1:2 loop
            x:=((if pass==1 then before[k,1] else now[k,1])-origin[1])/cell+(n+1)*.5;
            y:=((if pass==1 then before[k,2] else now[k,2])-origin[2])/cell+(n+1)*.5;
            wet:=if pass==1 then -before[k,3] else now[k,3];
            radius:=max(1.5,2.3*now[k,4]/cell);
            stamps[2*(k-1)+pass,:]:={x,y,radius,1.65*wet/(3.14159265*radius^2*cell^2),
                                      if pass==2 then foam*dt*3.5 else 0};
          end for;
        end if;
      end if;
    end for;
    if trigger>.5 then life:=8;end if;
    if life<=0 then return;end if;
    impactCell:={(entry[2]-origin[1])/cell+(n+1)*.5,(entry[3]-origin[2])/cell+(n+1)*.5,if trigger>.5 then min(1,entry[5]/4) else 0};
    height:=zeros(n,n);velocity:=zeros(n,n);foamField:=zeros(n,n);
    for j in 2:n-1 loop
      height[j,:]:=assembledRow(old[1,max(1,min(n,j+dy)),:],stamps,impactCell,cell,dx,j,1,remaining>dt and j+dy>1 and j+dy<n);
      velocity[j,:]:=carryRow(old[2,max(1,min(n,j+dy)),:],dx,remaining>dt and j+dy>1 and j+dy<n);
      foamField[j,:]:=assembledRow(old[3,max(1,min(n,j+dy)),:],stamps,impactCell,cell,dx,j,3,remaining>dt and j+dy>1 and j+dy<n);
    end for;
    steps:=max(1,integer(ceil(dt*sqrt(2*9.81*1.6/cell)/.82)));h:=dt/steps;
    for step in 1:16 loop
      if step>steps then break;end if;
      for j in 2:n-1 loop
        velocity[j,:]:=velocityRow(velocity[j,:],height[j-1,:],height[j,:],height[j+1,:],h,h*9.81*1.6/cell,j);
      end for;
      height:=height+h*velocity;
    end for;
    grid[1,:,:]:=height;
    grid[2,:,:]:=velocity;
    grid[3,:,:]:=foamField*exp(-.91*dt);
  end advancePatch;
equation
  when sample(0,.1) then
    center={cell*floor(sources[1,k]/cell+.5) for k in 1:2};
    (field,active)=advancePatch(pre(field),pre(prior),sources,center,pre(center),cell,
      if restart>.5 then 0 else clock-pre(stamp),pre(active),impact,
      if impact[5]>1.2 and clock>=impact[1] and clock-impact[1]<.6 and abs(impact[1]-pre(lastImpact))>.00001 then 1 else 0);
    prior=sources;stamp=clock;lastImpact=impact[1];
  end when;
end SwimmerWake;
