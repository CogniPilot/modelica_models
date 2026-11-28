within Marine.Sailing;
package SailingTactics "Fleet course decisions for the Hobie 16"
  function wrap
    input Real angle;
    output Real bounded;
  algorithm
    bounded:=atan2(sin(angle),cos(angle));
  end wrap;

  function mustKeepClear "Common open-water right-of-way cases"
    input Real own[2],heading,peer[9],upwind;
    output Boolean yield;
  protected
    Real side,otherSide,relative[2],length;
  algorithm
    side:=sin(upwind-heading);
    otherSide:=sin(upwind-peer[4]);
    relative:=own-peer[2:3];
    length:=if peer[9]>1.5 then 7.6 elseif peer[9]>.5 then 5.04 else 2.4;
    yield:=peer[9]>1.5 or (if side*otherSide<0 then side>0
      elseif relative*{cos(peer[4]),sin(peer[4])}< -.5*(5.04+length) then true
      elseif relative*{cos(heading),sin(heading)}>.5*(5.04+length) then false
      else relative*{cos(upwind),sin(upwind)}>.3);
  end mustKeepClear;

  function polar "Conservative close-hauled and downwind speed estimate"
    input Real heading,upwind,wind;
    output Real speed;
  protected Real off;
  algorithm
    off:=abs(wrap(heading-upwind));
    speed:=min(1.7,max(0,wind)/11)*max(0,min(1,(off-.50)/.45))*
      (if off<1.38 then 3.8+.9*(off-.94) elseif off<2.49 then 5.25 else 5.25-2.3*(off-2.49));
  end polar;

  function chooseCourse "Choose highest progress with safe crossing room and clean wind"
    input Real own[2],heading,speed,goal[2],upwind,wind,noGo,prior;
    input Real traffic[:,9];
    output Real course;
  protected
    Real request,bearings[15],h,off,s,v[2],rel[2],dv[2],at,gap,room,cost,best,err;
    Real initialGap,after,shadow,across,weight;
    Boolean yield;
  algorithm
    request:=atan2(goal[2]-own[2],goal[1]-own[1]);
    bearings:={request,upwind+noGo,upwind-noGo,upwind+2.55,upwind-2.55,prior,heading,
      request+.23,request-.23,request+.48,request-.48,heading+.65,heading-.65,heading+1.25,heading-1.25};
    best:=-1e8;course:=prior;
    for k in 1:size(bearings,1) loop
      off:=wrap(bearings[k]-upwind);
      h:=upwind+(if abs(off)<noGo then (if off>=0 then noGo else -noGo) else off);
      err:=abs(wrap(h-heading));
      s:=max(.45,.64*polar(h,upwind,wind)+.36*abs(speed))*(1-.18*min(1,err));
      v:=s*{cos(h),sin(h)};
      // Velocity made good to the target, turn cost, and a modest penalty for
      // changing tack unless a lift or the approach to the mark pays for it.
      cost:=s*cos(request-h)-.48*err-.32*abs(wrap(h-prior))
        -(if sin(h-upwind)*sin(heading-upwind)<0 and abs(off)<1.6 then .60 else 0)
        -(if sin(upwind-h)>.12 and abs(off)<1.6 then .11 else 0);
      for i in 1:size(traffic,1) loop
        if traffic[i,1]>.5 then
          rel:=traffic[i,2:3]-own;
          initialGap:=sqrt(rel*rel);
          if initialGap<75 then
            dv:=traffic[i,5:6]-v;
            at:=max(0,min(12,-rel*dv/max(.04,dv*dv)));
            gap:=sqrt((rel+at*dv)*(rel+at*dv));
            room:=if traffic[i,9]>1.5 then 7.6 else 5.6;
            yield:=mustKeepClear(own,heading,traffic[i,:],upwind);
            if gap<room and (at>.08 or rel*dv<0) then
              // Stand-on boats still manoeuvre to avoid contact when it becomes
              // clear the give-way boat has not acted.
              weight:=if yield then 18 elseif at<4.5 then 20 else 7;
              cost:=cost-weight*((room-gap)/room)^2*(1-at/16);
            end if;
            // When ducking a starboard boat, prefer a pass astern of it.
            if yield and at>0 and gap<room+2 then
              after:=(rel+at*dv)*{cos(traffic[i,4]),sin(traffic[i,4])};
              if after<0 then cost:=cost-1.1*(1-at/14);end if;
            end if;
            shadow:=rel*{cos(upwind),sin(upwind)};
            across:=abs(rel*{-sin(upwind),cos(upwind)});
            if shadow>1 and shadow<28 and across<3+.19*shadow then
              cost:=cost-.85*(1-shadow/28)*max(0,cos(h-upwind));
            end if;
          end if;
        end if;
      end for;
      if cost>best then best:=cost;course:=h;end if;
    end for;
  end chooseCourse;

  function newBreach "Detect a new give-way incident or contact with a course mark"
    input Real own[2],heading,upwind,traffic[:,9],contact[size(traffic,1)],previous[size(traffic,1)],mark,previousMark;
    output Real penalty;
  algorithm
    penalty:=if mark>1 and previousMark<=1 then 1 else 0;
    for i in 1:size(traffic,1) loop
      if traffic[i,1]>.5 and traffic[i,9]<1.5 and contact[i]>10 and previous[i]<=10 and mustKeepClear(own,heading,traffic[i,:],upwind) then
        penalty:=1;
      end if;
    end for;
  end newBreach;

  function roomToTurn
    input Real own[2],traffic[:,9],marks[:,2];
    output Boolean clear;
  protected Real rel[2];
  algorithm
    clear:=true;
    for i in 1:size(traffic,1) loop
      rel:=own-traffic[i,2:3];
      if traffic[i,1]>.5 and rel*rel<144 then clear:=false;end if;
    end for;
    for i in 1:size(marks,1) loop
      rel:=own-marks[i,:];
      if rel*rel<100 then clear:=false;end if;
    end for;
  end roomToTurn;
  function penaltyEscape "Sail away from nearby marks and hulls before beginning a penalty turn"
    input Real own[2],heading,upwind,traffic[:,9],marks[:,2];
    output Real target[2];
  protected
    Real escape[2],delta[2],distance;
  algorithm
    escape:=-.25*{cos(upwind),sin(upwind)};
    for i in 1:size(marks,1) loop
      delta:=own-marks[i,:];distance:=max(.1,sqrt(delta*delta));
      escape:=escape+max(0,1-distance/15)*delta/distance;
    end for;
    for i in 1:size(traffic,1) loop
      if traffic[i,1]>.5 then
        delta:=own-traffic[i,2:3];distance:=max(.1,sqrt(delta*delta));
        escape:=escape+max(0,1-distance/17)*delta/distance;
      end if;
    end for;
    distance:=sqrt(escape*escape);
    target:=own+16*(if distance>.04 then escape/distance else {-sin(heading),cos(heading)});
  end penaltyEscape;
end SailingTactics;
