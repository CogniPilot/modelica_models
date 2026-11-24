within Marine.Environment;
model MarineWind "Advected gusts, directional shifts and short-wave response on a periodic 96 m patch"
  constant Integer n=48;
  parameter Real period=96,dt=.25,seed=1;
  input Real meanEast,meanNorth,east,north;
  output Real strikeBearing,strikeFlash,thunderArrival "Atmospheric signal at the observer";
  Real stormAge,stormRange,stormPhase;
  Real travelE(start=0,fixed=true),travelN(start=0,fixed=true);
  discrete output Real field[n,n,3](each start=0,each fixed=true) "East, north [m/s] and short-wave roughness proxy [m/s]";
  input Real droneEast,droneNorth;
  output Real droneWind[3];
  output Real localEast,localNorth;
  function cell "Wind vector and evolving roughness at one grid point"
    input Integer i,j;
    input Real prior,speed,direction,meanShift,cx[8],cy[8],turn[8],gain[8],width[8],lengths[8],period,dt,t;
    output Real value[3];
  protected
    Real amplitude,shift,ex,ny,along,across,w,theta,u,relax;
  algorithm
    amplitude:=1;shift:=meanShift;
    for g in 1:8 loop
      ex:=mod((i-1)*period/n-cx[g]+period/2,period)-period/2;
      ny:=mod((j-1)*period/n-cy[g]+period/2,period)-period/2;
      along:=ex*cos(direction)+ny*sin(direction);across:=-ex*sin(direction)+ny*cos(direction);
      w:=exp(-.5*((along/lengths[g])^2+(across/width[g])^2));
      amplitude:=amplitude+gain[g]*w;shift:=shift+turn[g]*w;
    end for;
    theta:=direction+shift;u:=speed*amplitude;
    relax:=if u>prior then 1.4 else 3.5;
    value:={u*cos(theta),u*sin(theta),if t<.01 then u else prior+(1-exp(-dt/relax))*(u-prior)};
  end cell;
  function fieldStep
    input Real prev[n,n,3],baseE,baseN,travelE,travelN,t,seed,period,dt;
    output Real tile[n,n,3];
  protected
    Real speed,direction,meanShift;
    Real cx[8],cy[8],turn[8],gain[8],width[8],lengths[8];
  algorithm
    speed:=sqrt(baseE^2+baseN^2);direction:=atan2(baseN,baseE);
    meanShift:=.12*sin(t/37+seed)+.075*sin(t/21+.3*seed);
    for g in 1:8 loop
      cx[g]:=mod(59.17*g+7.3*seed+travelE+.6*g*sin(.027*t+g),period);
      cy[g]:=mod(37.61*g+3.6*seed+travelN+.4*g*cos(.043*t+g),period);
      width[g]:=3.5+mod(g*1.45,4);lengths[g]:=6+mod(g*4.3,10);
      turn[g]:=.30*sin(2.3*g+seed);
      gain[g]:=(if mod(g,4)==0 then -.30 else .48)*(.72+.28*sin(t/(7+g)+g+seed));
    end for;
    tile:={cell(i,j,prev[j,i,3],speed,direction,meanShift,cx,cy,turn,gain,width,lengths,period,dt,t)
      for i in 1:n, j in 1:n};
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
  stormAge=mod(time,14);
  stormPhase=floor(time/14);
  strikeBearing=atan2(meanNorth,meanEast)+3.14159+.71*sin(stormPhase*13.58+seed);
  stormRange=850+550*(1+sin(7.13*stormPhase+seed));
  strikeFlash=if stormAge<.5 then 0 else exp(-13*(stormAge-.5))+
    (if stormAge<.71 then 0 else .65*exp(-18*(stormAge-.71)));
  thunderArrival=if stormAge>=.5+stormRange/343 then stormPhase else stormPhase-1;
  der(travelE)=.74*meanEast;der(travelN)=.74*meanNorth;
  when sample(0,dt) then
    field=fieldStep(pre(field),meanEast,meanNorth,travelE,travelN,time,seed,period,dt);
  end when;
  boatWind=sampleWind(field,east,north,period);
  localEast=boatWind[1];localNorth=boatWind[2];
  droneWind=sampleWind(field,droneEast,droneNorth,period);
end MarineWind;
