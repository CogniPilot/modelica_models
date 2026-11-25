within Marine.Environment;
model WakeSpectrum "World-referenced periodic gravity-wave field with a moving displacement source"
  constant Integer n=64;
  constant Real pi=3.141592653589793;
  input Real omitWake=0 "Low performance preset: solve incident waves only";
  discrete Boolean wakeActive(start=false,fixed=true);
  input Real seaClockOffset=0 "Seconds added to local time for incident sea phase";
  parameter Real period=64,dt=.1,depth=14,decay=.04;
  parameter Boolean sharedDrift=false "Use a common clock and constant surface current for the spectral frame";
  parameter Real edgeDamping=0 "Peak damping rate [1/s] for outgoing waves at the edge of a hull-following, calm-background calculation window";
  parameter Boolean localHull=true "Compute radiation and reaction for an immersed local hull";
  parameter Boolean separatePeerSurface=false "Recover the incoming peer wake separately when the wind sea is supplied externally";
  input Real east,north,heading,heel,pitch,stations[7],separation,volume[2,6],beamHalf[7];
  input Real currentEast,currentNorth,windEast,windNorth,seaAmplitude;
  input Real peers[4,38]=zeros(4,38) "Source token, sample time, then east/north/immersed volume for twelve strips";
  discrete Real priorPeers[4,38](each start=0,each fixed=true),priorDrift[4,2](each start=0,each fixed=true);
  discrete output Real sourceStrips[12,3](each start=0,each fixed=true);
  Real driftE(start=0,fixed=not sharedDrift),driftN(start=0,fixed=not sharedDrift);
  discrete Real modesR[n,n](each start=0,each fixed=true),modesI[n,n](each start=0,each fixed=true);
  discrete Real peerModesR[n,n](each start=0,each fixed=true),peerModesI[n,n](each start=0,each fixed=true);
  discrete Boolean peerWakeActive(start=false,fixed=true),receivedPeer(start=false,fixed=true);
  discrete Boolean radiationActive(start=false,fixed=true) "A local or incoming disturbance can be present in the modes";
  discrete Real peerForcingR[n,n](each start=0,each fixed=true);
  discrete Real peerForcingI[n,n](each start=0,each fixed=true);
  discrete Real displacementR[n,n](each start=0,each fixed=true),displacementI[n,n](each start=0,each fixed=true);
  discrete output Real height[n,n](each start=0,each fixed=true);
  discrete output Real wakeHeight[n,n](each start=0,each fixed=true) "Hull-only elevation for surface shading [m]";
  discrete output Real peerHeight[n,n](each start=0,each fixed=true) "Incoming peer-only elevation for the local hull [m]";
  discrete output Real waveLoad[3](each start=0,each fixed=true) "Wave resistance east, north [N] and yaw [Nm]";
  discrete output Real incident[3](each start=0,each fixed=true) "Wind-wave elevation, pitch and roll at hull";
  discrete output Real sectionHeight[2,6](each start=0,each fixed=true) "Incoming FFT elevation at each pontoon strip [m]";
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
  function surfaceElevation "Real elevation from complex travelling-wave amplitudes"
    input Real realPart[n,n],imagPart[n,n];
    output Real z[n,n];
  protected
    constant Integer order[n]={reverseIndex(i,n) for i in 1:n};
    constant Integer opposite[n]={1+mod(1-i,n) for i in 1:n};
    constant Real cosine[n]={cos(2*pi*i/n) for i in 0:n-1};
    constant Real sine[n]={sin(2*pi*i/n) for i in 0:n-1};
    constant Integer folded[n]={min(i,opposite[i]) for i in 1:n};
    constant Real signIm[n]={if i<=div(n,2)+1 then 1 else -1 for i in 1:n};
    Real rowsR[div(n,2)+1,n],rowsI[div(n,2)+1,n],lineR[n],lineI[n];
    Integer partner;
  algorithm
    // Project onto the Hermitian spectrum of the real elevation. After the
    // east transform, conjugate north rows can be reconstructed exactly.
    for row in 1:div(n,2)+1 loop
      partner:=opposite[row];
      (lineR,lineI):=transformLine(
        {.5*(realPart[row,i]+realPart[partner,opposite[i]]) for i in 1:n},
        {.5*(imagPart[row,i]-imagPart[partner,opposite[i]]) for i in 1:n},
        cosine,sine,order);
      rowsR[row,:]:=lineR;rowsI[row,:]:=lineI;
    end for;
    // The inverse transform of A+iB returns two real water columns together.
    for col in 1:2:n loop
      (lineR,lineI):=transformLine(
        {rowsR[folded[i],col]-signIm[i]*rowsI[folded[i],col+1] for i in 1:n},
        {signIm[i]*rowsI[folded[i],col]+rowsR[folded[i],col+1] for i in 1:n},cosine,sine,order);
      z[:,col]:=lineR/(n*n);z[:,col+1]:=lineI/(n*n);
    end for;
  end surfaceElevation;

  function hullSource
    input Real x,y,bearing,heel,pitch,xs[7],sep,vol[2,6];
    output Real strips[12,3];
  protected
    Real along,across;
    Integer r;
  algorithm
    strips:=zeros(12,3);
    for hull in 1:2 loop
      across:=(3-2*hull)*sep*cos(heel);
      for panel in 1:6 loop
        r:=(hull-1)*6+panel;
        along:=.5*(xs[panel]+xs[panel+1])*cos(pitch)+(3-2*hull)*sep*sin(heel)*sin(pitch);
        strips[r,:]:={x+along*cos(bearing)-across*sin(bearing),y+along*sin(bearing)+across*cos(bearing),vol[hull,panel]};
      end for;
    end for;
  end hullSource;

  function peerForcing "Deposit the change in displaced volume on the periodic grid, then transform all peers together"
    input Real peers[4,38],old[4,38],driftE,driftN,oldDrift[4,2],east,north,L;
    input Boolean active;
    output Real re[n,n],im[n,n];
    output Boolean changed;
  protected
    Real grid[n,n],scale=n/L,u,v,fx,fy,z;
    Integer col,row,nextCol,nextRow,k;
  algorithm
    grid:=zeros(n,n);changed:=false;
    for peer in 1:4 loop
      if active and peers[peer,1]>0 and peers[peer,1]==old[peer,1] and peers[peer,2]>old[peer,2]
          and peers[peer,2]-old[peer,2]<1 and
          (peers[peer,3]-east)^2+(peers[peer,4]-north)^2<(.45*L)^2 and
          (old[peer,3]-east)^2+(old[peer,4]-north)^2<(.45*L)^2 then
        changed:=true;
        for strip in 1:12 loop
          k:=3+3*(strip-1);
          for endpoint in 1:2 loop
            u:=scale*(if endpoint==1 then peers[peer,k]-driftE else old[peer,k]-oldDrift[peer,1]);
            v:=scale*(if endpoint==1 then peers[peer,k+1]-driftN else old[peer,k+1]-oldDrift[peer,2]);
            z:=scale^2*(if endpoint==1 then peers[peer,k+2] else -old[peer,k+2]);
            col:=1+mod(integer(floor(u)),n);row:=1+mod(integer(floor(v)),n);
            nextCol:=1+mod(col,n);nextRow:=1+mod(row,n);
            fx:=u-floor(u);fy:=v-floor(v);
            grid[row,col]:=grid[row,col]+z*(1-fx)*(1-fy);
            grid[nextRow,col]:=grid[nextRow,col]+z*(1-fx)*fy;
            grid[row,nextCol]:=grid[row,nextCol]+z*fx*(1-fy);
            grid[nextRow,nextCol]:=grid[nextRow,nextCol]+z*fx*fy;
          end for;
        end for;
      end if;
    end for;
    if changed then
      (re,im):=transform(grid,zeros(n,n),false);
    else
      re:=zeros(n,n);im:=zeros(n,n);
    end if;
  end peerForcing;

  function advancePeerModes
    input Real prevR[n,n],prevI[n,n],driveR[n,n],driveI[n,n],L,h,dt,loss;
    output Real nextR[n,n],nextI[n,n];
    output Boolean active;
  protected
    Real rowR[n],rowI[n],kx,ky,k,phase,c,s,atten,r,q,peak;
    Integer mx,my;
  algorithm
    peak:=0;
    for j in 1:n loop
      rowR:=zeros(n);rowI:=zeros(n);
      my:=if j<=div(n,2) then j-1 else j-1-n;
      ky:=2*pi*my/L;
      for i in 1:n loop
        mx:=if i<=div(n,2) then i-1 else i-1-n;
        if (mx<>0 or my<>0) and abs(mx)<div(n,2) and abs(my)<div(n,2) then
          kx:=2*pi*mx/L;k:=sqrt(kx^2+ky^2);
          phase:=sqrt(9.80665*k*tanh(k*h))*dt;
          c:=cos(phase);s:=sin(phase);atten:=exp(-dt*loss*(1+.08*k^2));
          r:=atten*(c*prevR[j,i]+s*prevI[j,i])+driveR[j,i];
          q:=atten*(c*prevI[j,i]-s*prevR[j,i])+driveI[j,i];
          rowR[i]:=r;rowI[i]:=q;
          peak:=max(peak,max(abs(r),abs(q)));
        end if;
      end for;
      nextR[j,:]:=rowR;nextI[j,:]:=rowI;
    end for;
    active:=peak>1e-7;
    if not active then nextR:=zeros(n,n);nextI:=zeros(n,n);end if;
  end advancePeerModes;

  function peerResponse
    input Real previousR[n,n],previousI[n,n],forcingR[n,n],forcingI[n,n],L,depth,dt,decay;
    input Boolean needed;
    output Real nextR[n,n],nextI[n,n],height[n,n];
    output Boolean active;
  protected
    Real quadrature[n,n];
  algorithm
    if needed then
      (nextR,nextI,active):=advancePeerModes(previousR,previousI,forcingR,forcingI,L,depth,dt,decay);
      if active then
        (height,quadrature):=transform(nextR,nextI,true);
      else
        height:=zeros(n,n);
      end if;
    else
      nextR:=zeros(n,n);nextI:=zeros(n,n);height:=zeros(n,n);active:=false;
    end if;
  end peerResponse;

  function edgeGain "One-dimensional periodic absorbing layer centered on the hull"
    input Real center,L,dt,loss;
    output Real gain[n];
  protected
    Real dx=L/n;
    Real distance,blend;
  algorithm
    for i in 1:n loop
      distance:=abs(mod((i-1)*dx-center+.5*L,L)-.5*L);
      blend:=max(0,min(1,(distance/L-.30)/.20));
      gain[i]:=exp(-dt*loss*blend^2*(3-2*blend));
    end for;
  end edgeGain;

  function step
    input Real prevR[n,n],prevI[n,n],oldR[n,n],oldI[n,n],vol[2,6],xs[7],beams[7],sep,x,y,bearing,heel,pitch,ue,un,sea,L,h,dt,loss,t;
    input Boolean first;
    input Real peerR[n,n],peerI[n,n];
    input Boolean active=true,localHull=true;
    input Real edgeDamping=0;
    output Real wr[n,n],wi[n,n],displacedR[n,n],displacedI[n,n],tile[n,n],probe[3],load[3],wake[n,n];
  protected
    Real rowR[n],rowI[n],windR[n],windI[n],sourceR[n],sourceI[n],dx,kx,ky,k,omega,c,s,atten,along,across,ex,ny,a,b,amplitude,phase,slope;
    Integer mx,my,wetStrips,oppositeRow,opposite[n];
    Real km,peak,windSpeed,directional,seed,trial,gainX[n],gainY[n],rowGain[n];
    Real sr,si,mr,mi,stripR,stripI,momentArm,crossTerm,forceScale;
    Real cx[12,n],sx[12,n],cy[12,n],sy[12,n],ka,kb,weight;
    Real armAlong[12],armAcross[12],stripLength[12],stripWidth[12],stripDraft[12],sectionVolume[12];
    Real waveNumber[n],cosHeading,sinHeading,cosPitch,sinPitch,cosHeel,sinHeel;
    Real re[n,n],im[n,n],packedR[n,n],packedI[n,n],unused[n,n];
  algorithm
    // OceanRace supplies incident waves separately. With wake radiation off
    // and no wind here, every spectral mode and the inverse transform are zero.
    if not active and ue==0 and un==0 then
      wr:=zeros(n,n);wi:=zeros(n,n);displacedR:=zeros(n,n);displacedI:=zeros(n,n);
      tile:=zeros(n,n);probe:=zeros(3);load:=zeros(3);wake:=zeros(n,n);
      return;
    end if;
    windSpeed:=sqrt(ue*ue+un*un);peak:=max(.19,1.4/max(2,windSpeed));
    dx:=L/n;probe:=zeros(3);load:=zeros(3);forceScale:=1025*9.80665*dx^2/n^2;
    // A dry hull strip gives exactly zero forcing and reaction. Pack the
    // wet strips once in hull order before evaluating the spectral modes.
    wetStrips:=0;
    waveNumber:={2*pi*(if i<=div(n,2) then i-1 else i-1-n)/L for i in 1:n};
    cosHeading:=cos(bearing);sinHeading:=sin(bearing);
    cosPitch:=cos(pitch);sinPitch:=sin(pitch);cosHeel:=cos(heel);sinHeel:=sin(heel);
    armAlong:=zeros(12);armAcross:=zeros(12);stripLength:=zeros(12);stripWidth:=zeros(12);
    stripDraft:=zeros(12);sectionVolume:=zeros(12);
    cx:=zeros(12,n);sx:=zeros(12,n);cy:=zeros(12,n);sy:=zeros(12,n);
    if active and localHull then
      for hull in 1:2 loop
        for panel in 1:6 loop
          if vol[hull,panel]<>0 then
            wetStrips:=wetStrips+1;
            along:=.5*(xs[panel]+xs[panel+1])*cosPitch+(3-2*hull)*sep*sinHeel*sinPitch;
            across:=(3-2*hull)*sep*cosHeel;
            armAlong[wetStrips]:=along;armAcross[wetStrips]:=across;
            stripLength[wetStrips]:=max(.001,xs[panel+1]-xs[panel]);
            stripWidth[wetStrips]:=max(.001,beams[panel]+beams[panel+1]);
            stripDraft[wetStrips]:=vol[hull,panel]/(stripLength[wetStrips]*stripWidth[wetStrips]);
            sectionVolume[wetStrips]:=vol[hull,panel];
            ex:=x+along*cosHeading-across*sinHeading;
            ny:=y+along*sinHeading+across*cosHeading;
            cx[wetStrips,:]:={cos(waveNumber[i]*ex) for i in 1:n};
            sx[wetStrips,:]:={sin(waveNumber[i]*ex) for i in 1:n};
            cy[wetStrips,:]:={cos(waveNumber[i]*ny) for i in 1:n};
            sy[wetStrips,:]:={sin(waveNumber[i]*ny) for i in 1:n};
          end if;
        end for;
      end for;
    end if;
    re:=zeros(n,n);im:=zeros(n,n);displacedR:=oldR;displacedI:=oldI;
    wr:=prevR;wi:=prevI;
    for j in 1:n loop
      sourceR:=zeros(n);sourceI:=zeros(n);
      rowR:=prevR[j,:];rowI:=prevI[j,:];windR:=zeros(n);windI:=zeros(n);
      for i in 1:n loop
        mx:=if i<=div(n,2) then i-1 else i-1-n;
        my:=if j<=div(n,2) then j-1 else j-1-n;
        kx:=waveNumber[i];ky:=waveNumber[j];k:=sqrt(kx^2+ky^2);
        sr:=0;si:=0;mr:=0;mi:=0;
        omega:=sqrt(9.80665*k*tanh(k*h));
        if active then
          // A Nyquist component has no signed direction on this even grid.
          if localHull and abs(mx)<div(n,2) and abs(my)<div(n,2) then
            ka:=kx*cosHeading+ky*sinHeading;kb:=-kx*sinHeading+ky*cosHeading;
            for strip in 1:wetStrips loop
              // Match each strip's volume and rectangular second moments.
              // Submerged sources radiate less energy at short wavelengths.
              weight:=sectionVolume[strip]/dx^2*exp(-.5*k*stripDraft[strip]-((ka*stripLength[strip])^2+(kb*stripWidth[strip])^2)/24);
              stripR:=weight*(cx[strip,i]*cy[strip,j]-sx[strip,i]*sy[strip,j]);
              stripI:=-weight*(sx[strip,i]*cy[strip,j]+cx[strip,i]*sy[strip,j]);
              sr:=sr+stripR;si:=si+stripI;
              momentArm:=armAlong[strip]*kb-armAcross[strip]*ka;
              mr:=mr+momentArm*stripR;mi:=mi+momentArm*stripI;
            end for;
          end if;
          sourceR[i]:=sr;sourceI[i]:=si;
          c:=cos(omega*dt);s:=sin(omega*dt);
          atten:=exp(-dt*loss*(1+.08*k^2));
          // Complex travelling waves encode elevation and its quadrature.
          // A real spatial displacement increment excites both travel directions.
          rowR[i]:=atten*(c*prevR[j,i]+s*prevI[j,i])+(if first or k==0 then 0 else sourceR[i]-oldR[j,i])+(if k>0 and abs(mx)<div(n,2) and abs(my)<div(n,2) then peerR[j,i] else 0);
          rowI[i]:=atten*(c*prevI[j,i]-s*prevR[j,i])+(if first or k==0 then 0 else sourceI[i]-oldI[j,i])+(if k>0 and abs(mx)<div(n,2) and abs(my)<div(n,2) then peerI[j,i] else 0);

          if localHull then
            // Integral of -rho*g*displacement*grad(elevation), evaluated by
            // Parseval on the same modes used to reconstruct the displayed wake.
            crossTerm:=forceScale*(rowI[i]*sr-rowR[i]*si);
            load[1]:=load[1]+kx*crossTerm;load[2]:=load[2]+ky*crossTerm;
            load[3]:=load[3]+forceScale*(rowI[i]*mr-rowR[i]*mi);
          end if;
        else
          rowR[i]:=0;rowI[i]:=0;
        end if;
        // Directional gravity-wave spectrum. Independent reproducible phase and
        // Rayleigh amplitude per travelling mode break up parallel crest lines.
        if abs(mx)<div(n,2) and abs(my)<div(n,2) and kx*ue+ky*un>0 then
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
    if edgeDamping>0 and active then
      // Split-step absorbing layer on both quadratures of the travelling field.
      // Distance is wrapped relative to the hull so the layer also works as
      // the hull crosses the FFT origin. Preserve the hull forcing history.
      (tile,unused):=transform(wr,wi,true);
      // Damping decreases monotonically with distance. The more distant
      // coordinate supplies the smaller gain for this square window.
      gainX:=edgeGain(x,L,dt,edgeDamping);
      gainY:=edgeGain(y,L,dt,edgeDamping);
      for j in 1:n loop
        rowGain:={min(gainX[i],gainY[j]) for i in 1:n};
        tile[j,:]:=rowGain .* tile[j,:];
        unused[j,:]:=rowGain .* unused[j,:];
      end for;
      (wr,wi):=transform(tile,unused,false);
      wake:=tile;
      return;
    end if;
    if ue==0 and un==0 then
      // The coupled boats receive wind waves from Wreck.Ocean. Here their
      // total and hull-only elevations are the same real inverse transform.
      tile:=surfaceElevation(wr,wi);
      wake:=tile;
      return;
    end if;
    packedR:=re;
    packedI:=im;
    if active then
      // The Hermitian part of each spectrum reconstructs a real field.
      // Put the total sea in the real channel and the hull wake in the
      // imaginary channel to recover both fields in one inverse FFT.
      opposite:={1+mod(1-i,n) for i in 1:n};
      for j in 1:n loop
        oppositeRow:=opposite[j];
        packedR[j,:]:={.5*(re[j,i]+re[oppositeRow,opposite[i]]-wi[j,i]+wi[oppositeRow,opposite[i]]) for i in 1:n};
        packedI[j,:]:={.5*(im[j,i]-im[oppositeRow,opposite[i]]+wr[j,i]+wr[oppositeRow,opposite[i]]) for i in 1:n};
      end for;
    end if;
    (tile,unused):=transform(packedR,packedI,true);
    wake:=if active then unused else zeros(n,n);
  end step;
equation
  assert(edgeDamping<=0 or (windEast==0 and windNorth==0),
    "A local wake window requires separately supplied incident waves");
  assert(not separatePeerSurface or (windEast==0 and windNorth==0),
    "Separate peer wake requires the incident sea to be supplied by a separate model");
  if sharedDrift then
    driftE=currentEast*(time+seaClockOffset);
    driftN=currentNorth*(time+seaClockOffset);
  else
    der(driftE)=currentEast;
    der(driftN)=currentNorth;
  end if;
  when sample(0,dt) then
    wakeActive=omitWake<.5;
    sourceStrips=hullSource(east,north,heading,heel,pitch,stations,separation,volume);
    priorPeers=peers;
    for peer in 1:4 loop
      for axis in 1:2 loop
        priorDrift[peer,axis]=if abs(peers[peer,1]-pre(priorPeers[peer,1]))>.5 or peers[peer,2]>pre(priorPeers[peer,2])
          then (if axis==1 then driftE else driftN) else pre(priorDrift[peer,axis]);
      end for;
    end for;
    // Both responses use the same forward transform of the received sources.
    (peerForcingR,peerForcingI,receivedPeer)=peerForcing(peers,pre(priorPeers),driftE,driftN,
      pre(priorDrift),east,north,period,wakeActive);
    // A vehicle without a hull has zero radiation until a peer deposits a wake.
    // Keep evolving incoming waves after the peer has left until wakes are reset.
    radiationActive=wakeActive and (localHull or receivedPeer or pre(radiationActive));
    (modesR,modesI,displacementR,displacementI,height,incident,waveLoad,wakeHeight)=step(
      pre(modesR),pre(modesI),pre(displacementR),pre(displacementI),volume,stations,beamHalf,separation,
      east-driftE,north-driftN,heading,heel,pitch,windEast,windNorth,seaAmplitude,
      period,depth,dt,decay,time+seaClockOffset,time<dt/2 or not pre(wakeActive),
      peerForcingR,peerForcingI,radiationActive,localHull,edgeDamping);
    // Retain the total radiation for rendering and wave pressure. Only the
    // independently incident peer response is fed back into local buoyancy.
    (peerModesR,peerModesI,peerHeight,peerWakeActive)=peerResponse(pre(peerModesR),pre(peerModesI),
      peerForcingR,peerForcingI,period,depth,dt,decay,
      separatePeerSurface and wakeActive and (pre(peerWakeActive) or receivedPeer));
    sectionHeight=sampleSections(if separatePeerSurface then peerHeight else height,
      east-driftE,north-driftN,heading,heel,pitch,stations,separation,period);
  end when;
end WakeSpectrum;
