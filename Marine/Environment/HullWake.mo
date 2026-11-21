within Marine.Environment;
model HullWake "Local water-relative, forced and damped linear surface-wave patch"
  constant Integer n=48;
  parameter Real dx=.30, dt=.05,c=1.35 "Effective wave speed for the short hull-wave band [m/s]";
  input Real east,north,heading,surge,sway,currentEast,currentNorth;
  input Real sectionVolume[2,6],stations[7],separation,hullHeel,hullPitch;
  discrete Real blockage[n,n](each start=0,each fixed=true);
  Real driftE(start=0,fixed=true),driftN(start=0,fixed=true);
  discrete Integer cellE(start=0,fixed=true),cellN(start=0,fixed=true);
  discrete output Real eta[n,n](each start=0,each fixed=true);
  discrete Real rate[n,n](each start=0,each fixed=true);
  output Real originE,originN,level,pitch,roll;
  function stepField
    input Real h[n,n],v[n,n],previousBlock[n,n],vol[2,6],xs[7],sep,roll,pitch,spacing,step,speed,x,y,bearing;
    input Boolean starting;
    input Integer shiftX,shiftY;
    output Real next[n,n],velocity[n,n],occupancy[n,n];
  protected
    Integer ox,oy;
    Real e,f,along,across,lap,drag,ax,ay,edge;
  algorithm
    next:=zeros(n,n);velocity:=zeros(n,n);occupancy:=zeros(n,n);
    // Deposit each section's displaced volume on the horizontal grid with
    // normalized bilinear weights, then smooth over one cell on each side.
    for hull in 1:2 loop
      for section in 1:6 loop
        along:=.5*(xs[section]+xs[section+1])*cos(pitch);
        across:=(if hull==1 then 1 else -1)*sep*cos(roll);
        e:=(x+along*cos(bearing)-across*sin(bearing))/spacing;
        f:=(y+along*sin(bearing)+across*cos(bearing))/spacing;
        ox:=integer(floor(e));oy:=integer(floor(f));
        for jj in 0:3 loop
          for ii in 0:3 loop
            ax:=if ii==0 then .25*(1-e+ox) elseif ii==1 then .5-.25*(e-ox) elseif ii==2 then .25+.25*(e-ox) else .25*(e-ox);
            ay:=if jj==0 then .25*(1-f+oy) elseif jj==1 then .5-.25*(f-oy) elseif jj==2 then .25+.25*(f-oy) else .25*(f-oy);
            if ox+ii>=1 and ox+ii<=n and oy+jj>=1 and oy+jj<=n then
              occupancy[oy+jj,ox+ii]:=occupancy[oy+jj,ox+ii]+ax*ay*vol[hull,section]/spacing^2;
            end if;
          end for;
        end for;
      end for;
    end for;
    for j in 2:n-1 loop
      for i in 2:n-1 loop
        ox:=max(2,min(n-1,i+shiftX));oy:=max(2,min(n-1,j+shiftY));
        if ox==i+shiftX and oy==j+shiftY then
          lap:=(h[oy,ox-1]+h[oy,ox+1]+h[oy-1,ox]+h[oy+1,ox]-4*h[oy,ox])/spacing^2;
          edge:=min(min(i-1,n-i),min(j-1,n-j));
          drag:=.30+4*max(0,1-edge/7)^2;
          velocity[j,i]:=(v[oy,ox]+step*(speed^2*lap))/(1+step*drag);
          next[j,i]:=h[oy,ox]+step*velocity[j,i]+(if starting then 0 else occupancy[j,i]-previousBlock[oy,ox]);
        end if;
      end for;
    end for;
  end stepField;
  function sampleHeight
    input Real h[n,n],x,y,spacing;
    output Real z;
  protected
    Integer i,j;
    Real a,b;
  algorithm
    i:=max(1,min(n-1,integer(floor(x/spacing))+1));
    j:=max(1,min(n-1,integer(floor(y/spacing))+1));
    a:=max(0,min(1,x/spacing-i+1));b:=max(0,min(1,y/spacing-j+1));
    z:=(1-b)*((1-a)*h[j,i]+a*h[j,i+1])+b*((1-a)*h[j+1,i]+a*h[j+1,i+1]);
  end sampleHeight;
protected
  Real x,y,sectionX[2,6],sectionY[2,6],volumeX[2,6],volumeY[2,6];
public
  output Real waveForceEast,waveForceNorth,waveYaw;

equation
  der(driftE)=currentEast;der(driftN)=currentNorth;
  originE=cellE*dx+driftE;originN=cellN*dx+driftN;
  x=east-originE;y=north-originN;
  when sample(0,dt) then
    cellE=integer(floor((east-driftE)/dx))-div(n,2);
    cellN=integer(floor((north-driftN)/dx))-div(n,2);
    (eta,rate,blockage)=stepField(pre(eta),pre(rate),pre(blockage),sectionVolume,stations,separation,hullHeel,hullPitch,dx,dt,c,east-driftE-cellE*dx,north-driftN-cellN*dx,
       heading,time<dt/2,cellE-pre(cellE),cellN-pre(cellN));
  end when;
  for hull in 1:2 loop
    for panel in 1:6 loop
      sectionX[hull,panel]=.5*(stations[panel]+stations[panel+1])*cos(hullPitch)*cos(heading)-(3-2*hull)*separation*cos(hullHeel)*sin(heading);
      sectionY[hull,panel]=.5*(stations[panel]+stations[panel+1])*cos(hullPitch)*sin(heading)+(3-2*hull)*separation*cos(hullHeel)*cos(heading);
      volumeX[hull,panel]=-1025*9.80665*sectionVolume[hull,panel]*(sampleHeight(eta,x+sectionX[hull,panel]+dx,y+sectionY[hull,panel],dx)-sampleHeight(eta,x+sectionX[hull,panel]-dx,y+sectionY[hull,panel],dx))/(2*dx);
      volumeY[hull,panel]=-1025*9.80665*sectionVolume[hull,panel]*(sampleHeight(eta,x+sectionX[hull,panel],y+sectionY[hull,panel]+dx,dx)-sampleHeight(eta,x+sectionX[hull,panel],y+sectionY[hull,panel]-dx,dx))/(2*dx);
    end for;
  end for;
  waveForceEast=sum(volumeX);waveForceNorth=sum(volumeY);waveYaw=sum(sectionX.*volumeY-sectionY.*volumeX);
  level=sampleHeight(eta,x,y,dx);
  pitch=atan((sampleHeight(eta,x-.9*cos(heading),y-.9*sin(heading),dx)-
              sampleHeight(eta,x+.9*cos(heading),y+.9*sin(heading),dx))/1.8);
  roll=atan((sampleHeight(eta,x-.32*sin(heading),y+.32*cos(heading),dx)-
             sampleHeight(eta,x+.32*sin(heading),y-.32*cos(heading),dx))/.64);
end HullWake;
