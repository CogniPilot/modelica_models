within Marine.Environment;
model MarineWind "Advected gusts, directional shifts and short-wave response on a periodic 288 m patch"
  constant Integer n=48;
  parameter Real period=288,dt=.25,seed=1;
  input Real meanEast,meanNorth,east,north;
  output Real strikeBearing,strikeFlash,thunderArrival "Atmospheric signal at the observer";
  input Real phaseOffset=0 "Room phase minus local simulation time [s]";
  input Real advectionAnchor[5]=zeros(5) "Phase advance [s], peer east/north travel and local east/north travel at receipt [m]";
  output Real gustEast,gustNorth "Room-aligned advection distances [m]";
  Real weatherTime;
  Real stormAge,stormRange,stormPhase;
  Real travelE(start=0,fixed=true),travelN(start=0,fixed=true);
  discrete output Real field[n,n,3](each start=0,each fixed=true) "East, north [m/s] and short-wave roughness proxy [m/s]";
  input Real droneEast,droneNorth;
  output Real droneWind[3];
  output Real localEast,localNorth;
  function gustCell "Wind vector and roughness for one independent grid cell"
    input Real eastOffset[8],northOffset[8],previous,speed,direction,c,s,meanShift,width[8],lengths[8],turn[8],gain[8],t,rise,fall;
    output Real cell[3];
  protected
    Real amplitude,shift,ex,ny,along,across,w,theta,u,relax;
  algorithm
    amplitude:=.975;shift:=meanShift;
    for g in 1:8 loop
      ex:=eastOffset[g];
      ny:=northOffset[g];
      along:=ex*c+ny*s;across:=-ex*s+ny*c;
      // Compact smooth footprint, with zero slope at the perimeter.
      w:=max(0,1-(along/lengths[g])^2-(across/width[g])^2)^3;
      amplitude:=amplitude+gain[g]*w;shift:=shift+turn[g]*w;
    end for;
    theta:=direction+shift;u:=speed*amplitude;
    cell[1]:=u*cos(theta);cell[2]:=u*sin(theta);
    relax:=if u>previous then rise else fall;
    cell[3]:=if t<.01 then u else previous+relax*(u-previous);
  end gustCell;
  function fieldStep
    input Real prev[n,n,3],baseE,baseN,travelE,travelN,t,seed,period,dt;
    output Real tile[n,n,3];
  protected
    Real speed,direction,meanShift,c,s,anchorE,anchorN,along,across,hand,coordinate;
    Integer member,groupIndex;
    // A broad gust, two offset lobes and a trailing lull travel together.
    constant Real alongOffset[4]={0,-.108,.105,-.18},crossOffset[4]={0,.031,.018,-.09};
    constant Real halfLength[4]={.235,.145,.123,.195},halfWidth[4]={.117,.076,.093,.124};
    constant Real peak[4]={.27,.20,.17,-.23};
    Real cx[8],cy[8],turn[8],gain[8],width[8],lengths[8],eastOffsets[n,8],northOffsets[n,8],rise,fall;
  algorithm
    speed:=sqrt(baseE^2+baseN^2);direction:=atan2(baseN,baseE);
    c:=cos(direction);s:=sin(direction);
    meanShift:=.12*sin(t/37+seed)+.075*sin(t/21+.3*seed);
    for g in 1:8 loop
      member:=1+mod(g-1,4);groupIndex:=div(g-1,4);hand:=1-2*groupIndex;
      anchorE:=travelE+period*(.09+.46*groupIndex+.07*sin(seed+groupIndex));
      anchorN:=travelN+period*(.16+.42*groupIndex+.05*cos(seed-groupIndex));
      along:=period*(alongOffset[member]+.018*sin(t/39+g+seed));
      across:=period*hand*(crossOffset[member]+.026*cos(.031*t+.9*g+seed));
      cx[g]:=anchorE+along*c-across*s;cy[g]:=anchorN+along*s+across*c;
      width[g]:=period*halfWidth[member]*(1+.15*sin(t/46+g));
      lengths[g]:=period*halfLength[member]*(1+.12*cos(t/53+g));
      turn[g]:=.17*hand*cos(.7*member+seed);
      gain[g]:=peak[member]*(.9+.1*sin(t/34+g+seed));
    end for;
    // A periodic displacement along either axis is independent of the other.
    for i in 1:n loop
      coordinate:=(i-1)*period/n;
      for g in 1:8 loop
        eastOffsets[i,g]:=mod(coordinate-cx[g]+period/2,period)-period/2;
        northOffsets[i,g]:=mod(coordinate-cy[g]+period/2,period)-period/2;
      end for;
    end for;
    rise:=1-exp(-dt/1.4);
    fall:=1-exp(-dt/3.5);
    tile:={gustCell(eastOffsets[i,:],northOffsets[j,:],prev[j,i,3],speed,direction,c,s,
      meanShift,width,lengths,turn,gain,t,rise,fall) for i in 1:n, j in 1:n};
  end fieldStep;
  function sampleWind
    input Real grid[n,n,3],east,north,period;
    output Real wind[3];
  protected
    Real x,y,a,b;
    Integer i,j,k,l;
  algorithm
    x:=mod(east,period)*n/period;y:=mod(north,period)*n/period;
    i:=min(n,integer(floor(x))+1);j:=min(n,integer(floor(y))+1);
    k:=mod(i,n)+1;l:=mod(j,n)+1;a:=x-i+1;b:=y-j+1;
    for axis in 1:2 loop
      wind[axis]:=(1-b)*((1-a)*grid[j,i,axis]+a*grid[j,k,axis])+b*((1-a)*grid[l,i,axis]+a*grid[l,k,axis]);
    end for;
    wind[3]:=0;
  end sampleWind;
  Real boatWind[3];
equation
  weatherTime=time+phaseOffset;
  gustEast=travelE+advectionAnchor[2]-advectionAnchor[4]+.74*meanEast*advectionAnchor[1];
  gustNorth=travelN+advectionAnchor[3]-advectionAnchor[5]+.74*meanNorth*advectionAnchor[1];
  stormAge=mod(weatherTime,14);
  stormPhase=floor(weatherTime/14);
  strikeBearing=atan2(meanNorth,meanEast)+3.14159+.71*sin(stormPhase*13.58+seed);
  stormRange=850+550*(1+sin(7.13*stormPhase+seed));
  strikeFlash=if stormAge<.5 then 0 else exp(-13*(stormAge-.5))+
    (if stormAge<.71 then 0 else .65*exp(-18*(stormAge-.71)));
  thunderArrival=if stormAge>=.5+stormRange/343 then stormPhase else stormPhase-1;
  der(travelE)=.74*meanEast;der(travelN)=.74*meanNorth;
  when sample(0,dt) then
    field=fieldStep(pre(field),meanEast,meanNorth,gustEast,gustNorth,weatherTime,seed,period,dt);
  end when;
  boatWind=sampleWind(field,east,north,period);
  localEast=boatWind[1];localNorth=boatWind[2];
  droneWind=sampleWind(field,droneEast,droneNorth,period);
end MarineWind;
