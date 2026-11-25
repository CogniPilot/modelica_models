within Marine.Environment;
package SpectralSea "Directional, finite-depth Fourier sea on a periodic patch"
  constant Real g=Modelica.Constants.g_n, pi=Modelica.Constants.pi;
  constant Integer N=32;
  constant Real length=64 "Periodic tile size [m]";
  constant Real swellWavelength=30.48 "100 ft independent swell wavelength [m]";
  constant Real swellK=2*pi/swellWavelength "Swell wavenumber [rad/m]";
  constant Real longSwellWavelength=2*swellWavelength "Wavelength of the long-swell preset [m]";
  constant Real longSwellPreset=1.5;
  constant Real swellBearing=1.04159 "Swell propagation direction from east [rad]";
  constant Real nearshoreDepth=13.7 "Default still-water depth [m]";

  function seaPreset
    input Real state,wind;
    output Real windHeight "Target significant wind-wave height [m]";
    output Real peakPeriod "Wind-sea spectral peak period [s]";
    output Real swellAmplitude "Amplitude of the independent long swell [m]";
  protected
    Real smoothHeight,mid,steep;
  algorithm
    smoothHeight:=.08*min(1,max(0,wind)/11);
    mid:=min(1,max(0,state/.45));
    steep:=min(1,max(0,(state-.45)/.65));
    windHeight:=smoothHeight*(1-mid)+.62*mid+.53*steep;
    peakPeriod:=2.3+1.35*mid+1.15*steep;
    swellAmplitude:=.325*mid+.275*steep;
    if state>=longSwellPreset then
      // Move the directional spectrum toward longer, crossing waves while
      // keeping the shorter wind sea below the regular Swell preset.
      windHeight:=.47*min(1.7,max(0,wind)/11);
      peakPeriod:=5.2;
      swellAmplitude:=1.70;
    end if;
  end seaPreset;

  function swellWaveNumber
    input Real state;
    output Real k;
  algorithm
    k:=if state>=longSwellPreset then 2*pi/longSwellWavelength else swellK;
  end swellWaveNumber;

  function swellHeight
    input Real state;
    output Real amplitude;
  protected
    Real windHeight,period;
  algorithm
    (windHeight,period,amplitude):=seaPreset(state,11);
  end swellHeight;

  function bitReversedIndices "Bit-reversed order for a power-of-two transform length"
    input Integer size;
    output Integer indices[size];
  algorithm
    indices:=fill(1,size);
    for k in 1:size-1 loop
      // Shifting k right removes the bit that becomes the high output bit.
      indices[k+1]:=1+div(indices[div(k,2)+1]-1,2)+mod(k,2)*div(size,2);
    end for;
  end bitReversedIndices;
  function inverseLine "Unnormalised inverse radix-two transform of Fourier series coefficients"
    input Real re[N],im[N];
    output Real x[N],y[N];
  protected
    constant Integer stages=integer(log(N)/log(2)+.5);
    constant Integer shuffled[N]=bitReversedIndices(N);
    constant Real twiddleCos[div(N,2)]={cos(2*pi*k/N) for k in 0:div(N,2)-1};
    constant Real twiddleSin[div(N,2)]={sin(2*pi*k/N) for k in 0:div(N,2)-1};

    Integer j,root,span,half,blocks,offset,base,left,right;
    Real c,s,r,i;
  algorithm
    x:=zeros(N);y:=zeros(N);
    for n in 1:N loop
      j:=shuffled[n];x[n]:=re[j];y[n]:=im[j];
    end for;
    span:=2;
    for stage in 1:stages loop
      half:=div(span,2);
      // Every radix-two stage performs N/2 independent butterflies.
      blocks:=div(N,span);
      for butterfly in 0:div(N,2)-1 loop
        offset:=div(butterfly,blocks);
        base:=mod(butterfly,blocks)*span;
        root:=offset*blocks+1;
        c:=twiddleCos[root]; s:=twiddleSin[root];
        left:=base+offset+1; right:=left+half;
        r:=c*x[right]-s*y[right]; i:=s*x[right]+c*y[right];
        x[right]:=x[left]-r; y[right]:=y[left]-i;
        x[left]:=x[left]+r; y[left]:=y[left]+i;
      end for;
      span:=2*span;
    end for;
  end inverseLine;

  function spectrumColumn "Construct one pair of conjugate columns in a real directional sea"
    input Integer ix;
    input Real t,cb,sb,depth,level,pos[3],peak;
    output Real realColumn[N],imagColumn[N],conjugateReal[N],conjugateImag[N],probe[5],slope[2],variance;
  protected
    Real dk=2*pi/length,kx,ky,along,k,omega,f,groupSpeed,sigma,energy,amp,phaseSeed,amplitudeSeed,phase,ct,st,cp,sp,ratio;
    Integer mx,my,sx,sy,otherY,signOfMode;
  algorithm
    realColumn:=zeros(N);imagColumn:=zeros(N);conjugateReal:=zeros(N);conjugateImag:=zeros(N);
    probe:=zeros(5);slope:=zeros(2);variance:=0;mx:=ix-1;
    // Each antipodal pair has exactly one downwind member. Keep that member's
    // original seed and orientation when forming the paired Fourier amplitudes.
    for iy in 1:N loop
      my:=if iy-1<N/2 then iy-1 else iy-1-N;
      kx:=dk*mx;ky:=dk*my;along:=kx*cb+ky*sb;
      if abs(my)<N/2 and (mx<>0 or my>0) and along<>0 then
        signOfMode:=if along>0 then 1 else -1;
        kx:=signOfMode*kx;ky:=signOfMode*ky;
        sx:=mod(signOfMode*mx,N)+1;sy:=mod(signOfMode*my,N)+1;otherY:=mod(-my,N)+1;
        k:=sqrt(kx*kx+ky*ky);omega:=sqrt(g*k*tanh(k*depth));f:=omega/(2*pi);
        groupSpeed:=g*(tanh(k*depth)+k*depth/cosh(k*depth)^2)/(2*omega);
        sigma:=if f<peak then .07 else .09;
        energy:=f^(-5)*exp(-1.25*(peak/f)^4)*3.3^exp(-.5*((f-peak)/(peak*sigma))^2)
          *(abs(along)/k)^4*groupSpeed/(2*pi*k)*dk^2;
        phaseSeed:=mod(sin(sx*127.1+sy*311.7)*43758.5453,1);
        amplitudeSeed:=max(1e-6,mod(sin(sx*269.5+sy*183.3)*19341.235,1));
        amp:=sqrt(max(0,-2*energy*log(amplitudeSeed)));variance:=variance+.5*amp^2;
        phase:=2*pi*phaseSeed-omega*t;ct:=.5*amp*cos(phase);st:=signOfMode*.5*amp*sin(phase);
        realColumn[iy]:=ct;imagColumn[iy]:=st;
        if mx==0 then
          realColumn[otherY]:=ct;imagColumn[otherY]:=-st;
        else
          conjugateReal[otherY]:=ct;conjugateImag[otherY]:=-st;
        end if;
        phase:=kx*pos[1]+ky*pos[2]+phase;cp:=cos(phase);sp:=sin(phase);
        ratio:=cosh(k*level)/sinh(k*depth);
        probe[1]:=probe[1]+amp*cp;probe[2]:=probe[2]+amp*omega*(kx/k)*ratio*cp;
        probe[3]:=probe[3]+amp*omega*(ky/k)*ratio*cp;
        probe[4]:=probe[4]+amp*omega*sinh(k*level)/sinh(k*depth)*sp;
        probe[5]:=probe[5]+amp*cosh(k*level)/cosh(k*depth)*cp;
        slope[1]:=slope[1]-amp*sp*kx;slope[2]:=slope[2]-amp*sp*ky;
      end if;
    end for;
  end spectrumColumn;

  function field
    input Real t(unit="s"),wind(unit="m/s"),bearing(unit="rad"),fetch(unit="m"),waterDepth(unit="m"),tide(unit="m"),pos[3](each unit="m");
    input Real seaState=-1 "Nonnegative values select calibrated sea presets";
    output Real height[N,N] "Surface relative to tide, rows north, columns east";
    output Real probe[5] "Surface elevation, local wave velocity ENU, dynamic pressure head";
    output Real slope[2] "Surface derivative east and north";
  protected
    Real re[N,N],im[N,N],columnA[N],columnB[N],tmpR[N],tmpI[N];
    Real dk=2*pi/length;
    Real kx,ky,k,omega,peak,hs,amp,phase,atProbe,total=0,scale,level,period,unusedSwell;
    Real columnRe[N],columnIm[N],oppositeRe[N],oppositeIm[N],partProbe[5],partSlope[2],partVariance;
  algorithm
    re:=zeros(N,N); im:=zeros(N,N); probe:=zeros(5);slope:=zeros(2);
    level:=max(0,min(waterDepth,waterDepth+pos[3]-tide));
    peak:=1/max(1.2,7.54*max(0.1,wind)/g*tanh(0.077*(g*fetch/max(0.1,wind)^2)^0.25));
    hs:=0.283*wind^2/g*tanh(0.0125*(g*fetch/max(0.1,wind)^2)^0.42);
    if seaState>=0 then
      (hs,period,unusedSwell):=seaPreset(seaState,wind);
      peak:=1/period;
    end if;
    for ix in 1:div(N,2) loop
      (columnRe,columnIm,oppositeRe,oppositeIm,partProbe,partSlope,partVariance):=spectrumColumn(ix,t,cos(bearing),sin(bearing),waterDepth,level,pos,peak);
      re[:,ix]:=columnRe;im[:,ix]:=columnIm;
      if ix>1 then
        re[:,N+2-ix]:=oppositeRe;im[:,N+2-ix]:=oppositeIm;
      end if;
      probe:=probe+partProbe;slope:=slope+partSlope;total:=total+partVariance;
    end for;
    scale:=hs/(4*sqrt(max(1e-18,total)));
    probe:=probe*scale;
    slope:=slope*scale;
    // Standalone callers without presets retain the original background sea.
    kx:=dk; ky:=-dk; k:=sqrt(2)*dk; omega:=sqrt(g*k*tanh(k*waterDepth));
    phase:=0.7-omega*t; amp:=if seaState<0 then 0.32 else 0;
    re:=re*scale; im:=im*scale;
    re[N,2]:=re[N,2]+0.5*amp*cos(phase); im[N,2]:=im[N,2]+0.5*amp*sin(phase);
    re[2,N]:=re[2,N]+0.5*amp*cos(phase); im[2,N]:=im[2,N]-0.5*amp*sin(phase);
    atProbe:=kx*pos[1]+ky*pos[2]+phase;
    probe:=probe+amp*{cos(atProbe),omega*kx/k*cosh(k*level)/sinh(k*waterDepth)*cos(atProbe),
      omega*ky/k*cosh(k*level)/sinh(k*waterDepth)*cos(atProbe),
      omega*sinh(k*level)/sinh(k*waterDepth)*sin(atProbe),cosh(k*level)/cosh(k*waterDepth)*cos(atProbe)};
    slope:=slope-amp*sin(atProbe)*{kx,ky};
    // Positive/negative north wavenumbers are conjugates after the east
    // transform. DC is real and the Nyquist modes are identically zero.
    for row in 1:div(N,2) loop
      (tmpR,tmpI):=inverseLine(re[row,:],im[row,:]);
      re[row,:]:=tmpR;im[row,:]:=tmpI;
      if row>1 then re[N+2-row,:]:=tmpR;im[N+2-row,:]:=-tmpI;end if;
    end for;
    // Pack two real spatial columns into the real and imaginary parts of one
    // complex inverse transform: F(a + i*b) = F(a) + i*F(b).
    for col in 1:2:N loop
      (columnA,columnB):=inverseLine(re[:,col]-im[:,col+1],im[:,col]+re[:,col+1]);
      height[:,col]:=columnA;height[:,col+1]:=columnB;
    end for;
  end field;
end SpectralSea;
