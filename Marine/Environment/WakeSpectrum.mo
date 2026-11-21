within Marine.Environment;
model WakeSpectrum "World-referenced periodic gravity-wave field with a moving displacement source"
  constant Integer n=64;
  constant Real pi=3.141592653589793;
  parameter Real period=64,dt=.1,depth=14,decay=.04;
  input Real east,north,heading,heel,pitch,stations[7],separation,volume[2,6],beamHalf[7];
  input Real currentEast,currentNorth,windEast,windNorth,seaAmplitude;
  Real driftE(start=0,fixed=true),driftN(start=0,fixed=true);
  discrete Real modesR[n,n](each start=0,each fixed=true),modesI[n,n](each start=0,each fixed=true);
  discrete Real displacementR[n,n](each start=0,each fixed=true),displacementI[n,n](each start=0,each fixed=true);
  discrete output Real height[n,n](each start=0,each fixed=true);
  discrete output Real waveLoad[3](each start=0,each fixed=true) "Wave resistance east, north [N] and yaw [Nm]";
  discrete output Real incident[3](each start=0,each fixed=true) "Wind-wave elevation, pitch and roll at hull";
  discrete output Real sectionHeight[2,6](each start=0,each fixed=true) "Metres at each pontoon strip on the displayed FFT grid";
  function sampleSections
    input Real tile[n,n],x,y,bearing,roll,pitch,xs[7],sep,L;
    output Real z[2,6];
  protected Real a,b,u,v,along,across;
    Integer i,j,ii,jj;
  algorithm
    for hull in 1:2 loop
      for panel in 1:6 loop
        across:=(3-2*hull)*sep*cos(roll);
        along:=.5*(xs[panel]+xs[panel+1])*cos(pitch)+(3-2*hull)*sep*sin(roll)*sin(pitch);
        u:=mod((x+along*cos(bearing)-across*sin(bearing))*n/L,n);
        v:=mod((y+along*sin(bearing)+across*cos(bearing))*n/L,n);
        i:=integer(floor(u))+1;j:=integer(floor(v))+1;ii:=mod(i,n)+1;jj:=mod(j,n)+1;
        a:=u-floor(u);b:=v-floor(v);
        z[hull,panel]:=(1-b)*((1-a)*tile[j,i]+a*tile[j,ii])+b*((1-a)*tile[jj,i]+a*tile[jj,ii]);
      end for;
    end for;
  end sampleSections;
  function reverseIndex "One-based index in radix-two digit-reversed order"
    input Integer index,size;
    output Integer reversed;
  protected
    Integer bits,remaining;
  algorithm
    bits:=index-1; remaining:=size; reversed:=0;
    while remaining>1 loop
      reversed:=2*reversed+mod(bits,2);
      bits:=div(bits,2); remaining:=div(remaining,2);
    end while;
    reversed:=1+reversed;
  end reverseIndex;

  function transformLine "One gather per butterfly stage"
    input Real sourceR[n],sourceI[n],cosine[n],sine[n];
    input Integer order[n];
    output Real a[n],b[n];
  protected
    Real oldR[n],oldI[n];
    Integer halfSpan,lo[n],hi[n],phase[n];
  algorithm
    a:={sourceR[order[j]] for j in 1:n};
    b:={sourceI[order[j]] for j in 1:n};
    halfSpan:=1;
    while halfSpan<n loop
      oldR:=a; oldI:=b;
      lo:={1+2*halfSpan*div(j,2*halfSpan)+mod(j,halfSpan) for j in 0:n-1};
      hi:=lo+fill(halfSpan,n);
      phase:={1+mod(j,2*halfSpan)*div(n,2*halfSpan) for j in 0:n-1};
      a:={oldR[lo[j]]+oldR[hi[j]]*cosine[phase[j]]-oldI[hi[j]]*sine[phase[j]] for j in 1:n};
      b:={oldI[lo[j]]+oldR[hi[j]]*sine[phase[j]]+oldI[hi[j]]*cosine[phase[j]] for j in 1:n};
      halfSpan:=2*halfSpan;
    end while;
  end transformLine;

  function transform "Separable two-dimensional FFT. The inverse divides by n squared."
    input Real realPart[n,n],imagPart[n,n];
    input Boolean inverse;
    output Real re[n,n],im[n,n];
  protected
    Real rowsR[n,n],rowsI[n,n],lineR[n],lineI[n];
    Real normalization,cosine[n],sine[n];
    Integer order[n];
  algorithm
    order:={reverseIndex(j,n) for j in 1:n};
    cosine:={cos(2*pi*j/n) for j in 0:n-1};
    sine:={(if inverse then 1 else -1)*sin(2*pi*j/n) for j in 0:n-1};
    normalization:=if inverse then n*n else 1;
    for row in 1:n loop
      (lineR,lineI):=transformLine(realPart[row,:],imagPart[row,:],cosine,sine,order);
      rowsR[row,:]:=lineR; rowsI[row,:]:=lineI;
    end for;
    for column in 1:n loop
      (lineR,lineI):=transformLine(rowsR[:,column],rowsI[:,column],cosine,sine,order);
      re[:,column]:=lineR/normalization; im[:,column]:=lineI/normalization;
    end for;
  end transform;
  function step
    input Real prevR[n,n],prevI[n,n],oldR[n,n],oldI[n,n],vol[2,6],xs[7],beams[7],sep,x,y,bearing,heel,pitch,ue,un,sea,L,h,dt,loss,t;
    input Boolean first;
    output Real wr[n,n],wi[n,n],displacedR[n,n],displacedI[n,n],tile[n,n],probe[3],load[3];
  protected
    Real rowR[n],rowI[n],windR[n],windI[n],sourceR[n],sourceI[n],dx,kx,ky,k,omega,c,s,atten,along,across,ex,ny,a,b,amplitude,phase,slope;
    Integer mx,my,strip;
    Real km,peak,windSpeed,directional,seed,trial;
    Real sr,si,mr,mi,stripR,stripI,momentArm,crossTerm,forceScale;
    Real cx[12,n],sx[12,n],cy[12,n],sy[12,n],ka,kb,length,width,draft,weight;
    Real re[n,n],im[n,n],unused[n,n];
  algorithm
    windSpeed:=sqrt(ue*ue+un*un);peak:=max(.19,1.4/max(2,windSpeed));
    dx:=L/n;probe:=zeros(3);load:=zeros(3);forceScale:=1025*9.80665*dx^2/n^2;
    // Fourier transform of six immersed strips on each hull. Separable phase
    // tables preserve the exact subcell locations and avoid a forward FFT.
    cx:=zeros(12,n);sx:=zeros(12,n);cy:=zeros(12,n);sy:=zeros(12,n);
    for hull in 1:2 loop
      for panel in 1:6 loop
        strip:=(hull-1)*6+panel;
        along:=.5*(xs[panel]+xs[panel+1])*cos(pitch);
        across:=(3-2*hull)*sep*cos(heel);
        ex:=x+along*cos(bearing)-across*sin(bearing);
        ny:=y+along*sin(bearing)+across*cos(bearing);
        for i in 1:n loop
          k:=2*pi*(if i<=div(n,2) then i-1 else i-1-n)/L;
          cx[strip,i]:=cos(k*ex);sx[strip,i]:=sin(k*ex);
          cy[strip,i]:=cos(k*ny);sy[strip,i]:=sin(k*ny);
        end for;
      end for;
    end for;
    re:=zeros(n,n);im:=zeros(n,n);displacedR:=oldR;displacedI:=oldI;
    wr:=prevR;wi:=prevI;
    for j in 1:n loop
      sourceR:=zeros(n);sourceI:=zeros(n);
      rowR:=prevR[j,:];rowI:=prevI[j,:];windR:=zeros(n);windI:=zeros(n);
      for i in 1:n loop
        mx:=if i<=div(n,2) then i-1 else i-1-n;
        my:=if j<=div(n,2) then j-1 else j-1-n;
        kx:=2*pi*mx/L;ky:=2*pi*my/L;k:=sqrt(kx^2+ky^2);
        sr:=0;si:=0;mr:=0;mi:=0;
        ka:=kx*cos(bearing)+ky*sin(bearing);kb:=-kx*sin(bearing)+ky*cos(bearing);
        for hull in 1:2 loop
          for panel in 1:6 loop
            strip:=(hull-1)*6+panel;
            length:=max(.001,xs[panel+1]-xs[panel]);
            width:=max(.001,beams[panel]+beams[panel+1]);
            draft:=vol[hull,panel]/(length*width);
            // Match each strip's volume and rectangular second moments.
            // Submerged sources radiate less energy at short wavelengths.
            weight:=vol[hull,panel]/dx^2*exp(-.5*k*draft-((ka*length)^2+(kb*width)^2)/24);
            stripR:=weight*(cx[strip,i]*cy[strip,j]-sx[strip,i]*sy[strip,j]);
            stripI:=-weight*(sx[strip,i]*cy[strip,j]+cx[strip,i]*sy[strip,j]);
            sr:=sr+stripR;si:=si+stripI;
            along:=.5*(xs[panel]+xs[panel+1])*cos(pitch);
            across:=(3-2*hull)*sep*cos(heel);
            momentArm:=along*kb-across*ka;
            mr:=mr+momentArm*stripR;mi:=mi+momentArm*stripI;
          end for;
        end for;
        // The signed wavevector is ambiguous on a Nyquist row or column.
        // Omit it so a symmetric tow cannot acquire a spurious lateral load.
        if abs(mx)==div(n,2) or abs(my)==div(n,2) then
          sr:=0;si:=0;mr:=0;mi:=0;
        end if;
        sourceR[i]:=sr;sourceI[i]:=si;
        omega:=sqrt(9.80665*k*tanh(k*h));c:=cos(omega*dt);s:=sin(omega*dt);
        atten:=exp(-dt*loss*(1+.08*k^2));
        // Complex travelling waves encode elevation and its quadrature.
        // A real spatial displacement increment excites both travel directions.
        rowR[i]:=atten*(c*prevR[j,i]+s*prevI[j,i])+(if first or k==0 then 0 else sourceR[i]-oldR[j,i]);
        rowI[i]:=atten*(c*prevI[j,i]-s*prevR[j,i])+(if first or k==0 then 0 else sourceI[i]-oldI[j,i]);

        // Integral of -rho*g*displacement*grad(elevation), evaluated by
        // Parseval on the same modes used to reconstruct the displayed wake.
        crossTerm:=forceScale*(rowI[i]*sr-rowR[i]*si);
        load[1]:=load[1]+kx*crossTerm;load[2]:=load[2]+ky*crossTerm;
        load[3]:=load[3]+forceScale*(rowI[i]*mr-rowR[i]*mi);
        // Directional gravity-wave spectrum. Independent reproducible phase and
        // Rayleigh amplitude per travelling mode break up parallel crest lines.
        if k>0 and abs(mx)<div(n,2) and abs(my)<div(n,2) then
          km:=k/peak;
          directional:=max(0,(kx*ue+ky*un)/(k*max(.01,windSpeed)));
          seed:=mod(sin(mx*127.1+my*311.7)*43758.5453,1);
          trial:=mod(sin(mx*269.5+my*183.3)*43758.5453,1);
          amplitude:=(.013*min(16,windSpeed)+sea*.85)*2*pi/(L*peak)
            *exp(-.625/km^2-.035*k^2)/km^2*directional^2*sqrt(-2*log(max(.0001,trial)));
          phase:=2*pi*seed-omega*t;
          windR[i]:=n*n*amplitude*cos(phase);
          windI[i]:=n*n*amplitude*sin(phase);
          phase:=phase+kx*x+ky*y;slope:=-amplitude*sin(phase);
          probe[1]:=probe[1]+amplitude*cos(phase);
          probe[2]:=probe[2]-slope*(kx*cos(bearing)+ky*sin(bearing));
          probe[3]:=probe[3]+slope*(-kx*sin(bearing)+ky*cos(bearing));
        end if;
      end for;
      displacedR[j,:]:=sourceR;displacedI[j,:]:=sourceI;
      wr[j,:]:=rowR;wi[j,:]:=rowI;re[j,:]:=rowR+windR;im[j,:]:=rowI+windI;
    end for;
    probe[2]:=atan(probe[2]);probe[3]:=atan(probe[3]);
    (tile,unused):=transform(re,im,true);
  end step;
equation
  der(driftE)=currentEast;der(driftN)=currentNorth;
  when sample(0,dt) then
    (modesR,modesI,displacementR,displacementI,height,incident,waveLoad)=step(pre(modesR),pre(modesI),pre(displacementR),pre(displacementI),volume,stations,beamHalf,separation,east-driftE,north-driftN,heading,heel,pitch,windEast,windNorth,seaAmplitude,period,depth,dt,decay,time,time<dt/2);
    sectionHeight=sampleSections(height,east-driftE,north-driftN,heading,heel,pitch,stations,separation,period);
  end when;
end WakeSpectrum;
