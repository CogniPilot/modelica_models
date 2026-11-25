within Marine.Wreck;
package Perception "Acoustic/optical acquisition and pressure- and gravity-aided keyframe SLAM"
  constant Integer rays=1017, width=36, height=24, keys=48, stride=180;
  constant Real cameraFx=18/tan(40*pi/180),cameraFy=12/tan(32*pi/180);
  constant Real dvlMount[3]={0,0,-.15} "Assumed transducer face below the body origin, FLU";
  function directions
    input Real focal=cameraFx;
    output Real d[rays,3];
  protected
    Real a,b,l,u,v;
    Integer row,col;
  algorithm
    for i in 1:rays loop
      if i<=4 then
        a:=if i==1 then 0 elseif i==2 then pi elseif i==3 then pi/2 else -pi/2;
        d[i,:]:={sin(pi/8)*cos(a),sin(pi/8)*sin(a),-cos(pi/8)};
      elseif i<=153 then
        if i<=53 then a:=(i-29)*ImagingSonar.halfFan/24; b:=0;
        else row:=div(i-54,25);col:=mod(i-54,25);a:=(col-12)*pi/36;
          b:=(if row==0 then -24 elseif row==1 then -12 elseif row==2 then 12 else 24)*pi/180;
        end if;
        d[i,:]:={cos(b)*cos(a),cos(b)*sin(a),sin(b)};
      else
        u:=(17.5-mod(i-154,width))/focal;v:=(11.5-div(i-154,width))/cameraFy;l:=sqrt(1+u*u+v*v);
        d[i,:]:={1/l,u/l,v/l};
      end if;
    end for;
  end directions;

  function slabInterval "Parametric ray interval inside a rectangular box; hi < lo means miss"
    input Real origin[3],direction[3],lower[3],upper[3],parallelTolerance;
    output Real lo,hi;
  protected
    Real a,b;
  algorithm
    lo:=-1e6;hi:=1e6;
    for k in 1:3 loop
      if abs(direction[k])<parallelTolerance then
        if origin[k]<lower[k] or origin[k]>upper[k] then hi:=-1e6;end if;
      else
        a:=(lower[k]-origin[k])/direction[k];b:=(upper[k]-origin[k])/direction[k];
        lo:=max(lo,min(a,b));hi:=min(hi,max(a,b));
      end if;
      if hi<lo then break;end if;
    end for;
  end slabInterval;

  function intersectsBounds "Conservative subtree test over the sensor near and far planes"
    input Real o[3],d[3],low[3],high[3],limit;
    output Boolean hit;
  protected Real entry,exitDistance;
  algorithm
    (entry,exitDistance):=slabInterval(o,d,low,high,1e-15);
    hit:=exitDistance>=max(.08,entry) and entry<=limit;
  end intersectsBounds;

  function plateRange "Closest entry or exit of an oriented solid; ranges are in metres"
    input Real o[3],d[3],center[3],halfExtent[3],worldToBox[3,3],limit;
    output Real distance;
  protected Real entry,exitDistance,hit;
  algorithm
    (entry,exitDistance):=slabInterval(worldToBox*(o-center),worldToBox*d,-halfExtent,halfExtent,1e-9);
    hit:=if entry>=.08 then entry else exitDistance;
    distance:=if exitDistance>=entry and hit>=.08 then min(limit,hit) else limit;
  end plateRange;

  function renderDepth "First returns along the four DVL, 149 sonar and 864 pinhole-camera rays"
    input Real pos[3],T[3,3],d[rays,3];
    input Boolean camera=true,sonar=true;
    output Real depth[rays];
  protected Real eye[3],limit,hit;
  algorithm
    eye:=pos+transpose(T)*{.62,0,.09};
    depth:=zeros(rays);
    for i in 1:(if camera then rays elseif sonar then 153 else 4) loop
      if i<=4 or i>153 then
      limit:=if i<=4 then 50/cos(pi/8) elseif i<=153 then 18 else 8;
      hit:=WreckSolids.beamRange(if i<=4 then pos+transpose(T)*dvlMount else eye,transpose(T)*d[i,:],limit+1);
      depth[i]:=if hit<=limit then hit else 0;
      end if;
    end for;
  end renderDepth;

  function noise "Repeatable zero-mean instrument noise, independently keyed by sample and channel"
    input Real t;input Integer channel;output Real n;
  algorithm
    n:=2*(mod(43758.5453*sin(12.9898*channel+78.233*(t+1)),1)-.5);
  end noise;

  function instruments
    input Real t,pos[3],T[3,3],v[3],rates[3],accel[3],pressure,depth[rays],d[rays,3];
    input Boolean camera=true;
    output Real raw[195] "149 sonar ranges, four DVL ranges and radial velocities, pressure, inertial measurements";
    output Real image[864];
  protected
    Real p[3],r,texture,rayWorld[3],eye[3],signalLevel;
  algorithm
    raw:=zeros(195);eye:=pos+transpose(T)*{.62,0,.09};
    for i in 1:153 loop
      r:=depth[i];
      raw[i]:=if r>.08 and noise(t,1000+i)>(if i<5 then -.976 else -.93) then max(.08,r+noise(t,i)*(.012+.0015*r)) else 0;
    end for;
    for i in 1:4 loop
      // Bottom-track Doppler measures transducer velocity along the acoustic axis.
      // Residual scale and random errors are illustrative standard-unit settings.
      r:=1.006*(d[i,:]*(v+cross(rates,dvlMount)))+.009*noise(t,160+i);
      if abs(r)>3.75 then raw[i]:=0;end if;
      raw[153+i]:=if raw[i]>0 then .0001*floor(r/.0001+.5) else 0;
    end for;
    raw[158]:=pressure+55*noise(t,158);
    for i in 1:3 loop
      raw[158+i]:=rates[i]+.002*noise(t,158+i)+(if i==3 then .0018 else 0);
      raw[161+i]:=accel[i]+.025*noise(t,161+i);
    end for;
    image:=zeros(864);
    for i in 1:(if camera then 864 else 0) loop
      r:=depth[153+i];signalLevel:=14;
      if r>0 then
        rayWorld:=transpose(T)*d[153+i,:];p:=eye+r*rayWorld;
        texture:=.5+.24*sin(p[1]*6+p[2]*3)*cos(p[3]*5)+.23*sin(p[3]*11+p[2]*4);
        signalLevel:=signalLevel+320*texture*exp(-.19*r)/(1+.025*r*r);
      end if;
      image[i]:=floor(max(0,min(255,signalLevel+1.5*noise(t,1500+i)))+.5);
    end for;
  end instruments;

  function features "FAST corners and measured optical patch correspondences in a spatial grid"
    input Real image[864],old[864];
    output Real ft[24,5] "x, y, previous x, previous y, 12-bit patch signature. -1 marks a missing match";
    output Real ring[16],opticalRate;
    output Integer n,matches;
  protected
    constant Integer offsets[16]={-108,-107,-70,-33,3,39,74,109,108,107,70,33,-3,-39,-74,-109};
    Real diff[16],score,best,first,second,sad,desc,delta;
    Integer p,chosen,bx,by,sx,sy;
  algorithm
    ft:=fill(-1,24,5);ring:=zeros(16);opticalRate:=0;n:=0;matches:=0;
    for gy in 0:2 loop for gx in 0:5 loop
      best:=7;chosen:=0;
      for y in 5+gy*5:min(18,9+gy*5) loop for x in 5+gx*4:min(30,8+gx*4) loop
        p:=1+x+36*y;
        for k in 1:16 loop diff[k]:=image[p+offsets[k]]-image[p];end for;
        score:=Vision.Features.FastCircleScore(diff);
        if score>best then best:=score;chosen:=p;end if;
      end for;end for;
      if chosen>0 then
        n:=n+1;p:=chosen;bx:=mod(p-1,36);by:=div(p-1,36);
        ft[n,1]:=bx;ft[n,2]:=by;desc:=0;
        for k in 1:16 loop diff[k]:=image[p+offsets[k]]-image[p];end for;
        if n==1 then ring:=diff;end if;
        for k in 1:12 loop desc:=2*desc+(if image[p+offsets[k]]>image[p+offsets[mod(k+5,16)+1]] then 1 else 0);end for;
        ft[n,5]:=desc;first:=1e9;second:=1e9;sx:=0;sy:=0;
        for dy in -1:1 loop for dx in -2:2 loop
          sad:=0;for yy in -1:1 loop for xx in -2:2 loop
            sad:=sad+abs(image[p+36*yy+xx]-old[p+36*(yy+dy)+xx+dx]);
          end for;end for;
          if sad<first then second:=first;first:=sad;sx:=dx;sy:=dy;else second:=min(second,sad);end if;
        end for;end for;
        if first<190 and first<.82*second then
          matches:=matches+1;ft[n,3]:=bx+sx;ft[n,4]:=by+sy;
          delta:=atan((17.5-bx-sx)/cameraFx)-atan((17.5-bx)/cameraFx);opticalRate:=opticalRate+delta/.2;
        end if;
      end if;
    end for;end for;
    opticalRate:=opticalRate/max(1,matches);
  end features;

  function registration "Robust sonar point-to-point registration, using gravity and pressure to constrain roll, pitch and depth"
    input Real cloud[48,3],ref[stride],pose[4];
    output Real correction[3];output Integer inliers;output Real rms;
  protected
    Real cx,cy,rx,ry,z,x,y,best,ex,ey,ez,bx,by,sxx,sxy,sumx,sumy,cosine,sine,den,dx,dy;
    Integer off;
  algorithm
    correction:=zeros(3);inliers:=0;rms:=99;
    for iteration in 1:2 loop
      sumx:=0;sumy:=0;sxx:=0;sxy:=0;den:=0;inliers:=0;
      cosine:=cos(pose[4]+correction[3]);sine:=sin(pose[4]+correction[3]);
      for i in 1:48 loop
        if abs(cloud[i,1])+abs(cloud[i,2])>.04 then
          cx:=cosine*cloud[i,1]-sine*cloud[i,2];cy:=sine*cloud[i,1]+cosine*cloud[i,2];
          x:=pose[1]+correction[1]+cx;y:=pose[2]+correction[2]+cy;z:=pose[3]+cloud[i,3];
          best:=.58^2;bx:=0;by:=0;
          for j in 1:48 loop
            off:=8+3*(j-1);rx:=ref[off+1];ry:=ref[off+2];
            if abs(rx)+abs(ry)>.04 then
              ex:=ref[1]+cos(ref[4])*rx-sin(ref[4])*ry-x;ey:=ref[2]+sin(ref[4])*rx+cos(ref[4])*ry-y;ez:=ref[3]+ref[off+3]-z;
              if ex*ex+ey*ey+ez*ez<best then best:=ex*ex+ey*ey+ez*ez;bx:=ex;by:=ey;end if;
            end if;
          end for;
          if best<.58^2 then
            inliers:=inliers+1;den:=den+best;sumx:=sumx+bx;sumy:=sumy+by;
            sxy:=sxy+cx*by-cy*bx;sxx:=sxx+cx*cx+cy*cy;
          end if;
        end if;
      end for;
      if inliers>=6 then
        dx:=sumx/inliers;dy:=sumy/inliers;
        correction:=correction+{max(-.25,min(.25,dx)),max(-.25,min(.25,dy)),max(-.045,min(.045,sxy/max(1,sxx)))};
        rms:=sqrt(den/inliers);
      end if;
    end for;
  end registration;
  function estimate "Pressure/gravity aided sonar keyframe graph; temporal optical features constrain yaw and verify revisits"
    input Real t,raw[195],d[rays,3],image[864],previousImage[864],previous[21],history[keys,stride];
    input Boolean camera=true,mapping=true;
    input Real initialPosition[3]={-5,0,-10},pressureDatum=.29;
    output Real s[21],graph[keys,stride],sensor[91],ft[24,5];
  protected
    Real ring[16],opticalRate,cloud[48,3],r,roll,pitch,cp,sp,cr,sr,x,y,z,kg,fix[3],rms,best,chosenFix[3],innovation[4],oldPose[4];
    Integer nf,nm,nk,beam,nearest,count,loopId,votes,idx,off,edgeIndex;
    Real dist,bestDist,dt,measVar,bodyHeave,levelSurge,levelSway,bearing,oldBearing,axialRange,rateSum,correctedRate;
    Integer rateCount;
    Boolean lock,newKey;
  algorithm
    s:=previous;graph:=history;sensor:=zeros(91);dt:=if t==0 then 0 else .2;
    if t==0 then s[1:3]:=initialPosition;s[9:12]:={.04,.04,.04,.000225};
      // Zero-rate measurement in the stationary launch cradle.
      s[21]:=raw[161];
    end if;
    if camera then
      (ft,ring,opticalRate,nf,nm):=features(image,previousImage);
    else
      ft:=fill(-1,24,5);ring:=zeros(16);opticalRate:=0;nf:=0;nm:=0;
    end if;
    lock:=min(raw[1:4])>0;
    if raw[1]>0 and raw[2]>0 then s[5]:=(raw[154]-raw[155])/(2*sin(pi/8));end if;
    if raw[3]>0 and raw[4]>0 then s[6]:=(raw[156]-raw[157])/(2*sin(pi/8));end if;
    s[7]:=.85*(s[7]+dt*raw[159])+.15*atan2(raw[163],raw[164]);
    s[8]:=.85*(s[8]+dt*raw[160])+.15*atan2(-raw[162],sqrt(raw[163]^2+raw[164]^2));
    roll:=s[7];pitch:=s[8];cr:=cos(roll);sr:=sin(roll);cp:=cos(pitch);sp:=sin(pitch);
    correctedRate:=raw[161]-s[21];
    s[4]:=s[4]+dt*correctedRate;s[12]:=s[12]+dt^2*.003^2;
    // A forward translation produces optical flow even with zero yaw. Use
    // the measured forward sonar profile and DVL to predict the old bearing
    // of each matched corner in the absence of rotation.
    rateSum:=0;rateCount:=0;
    for k in 1:18 loop
      if dt>0 and lock and ft[k,3]>=0 then
        bearing:=atan((17.5-ft[k,1])/cameraFx);oldBearing:=atan((17.5-ft[k,3])/cameraFx);
        beam:=29+integer(floor(bearing*24/ImagingSonar.halfFan+.5));
        axialRange:=raw[beam]*cos(ImagingSonar.depression)*cos(bearing)+ImagingSonar.mount[1]-.62;
        if axialRange>.4 then
          r:=(oldBearing-atan2(axialRange*tan(bearing)+dt*s[6],axialRange+dt*s[5]))/dt;
          if abs(r-correctedRate)<.16 then rateSum:=rateSum+r;rateCount:=rateCount+1;end if;
        end if;
      end if;
    end for;
    opticalRate:=rateSum/max(1,rateCount);
    if rateCount>=3 then
      kg:=s[12]/(s[12]+.012);s[4]:=s[4]+kg*dt*(opticalRate-correctedRate);s[12]:=(1-kg)*s[12];
    end if;
    bodyHeave:=if lock then -sum(raw[154:157])/(4*cos(pi/8)) else 0;
    levelSurge:=cp*s[5]+sp*(sr*s[6]+cr*bodyHeave);
    levelSway:=cr*s[6]-sr*bodyHeave;
    s[1]:=s[1]+dt*(levelSurge*cos(s[4])-levelSway*sin(s[4]));
    s[2]:=s[2]+dt*(levelSurge*sin(s[4])+levelSway*cos(s[4]));
    // DVL radial noise is bounded by .009 m/s. Allow .02 m/s horizontal
    // uncertainty per ping after beam inversion, and .11 m/s during coast.
    s[9]:=s[9]+dt^2*(if lock then .02^2 else .11^2);
    s[10]:=s[10]+dt^2*(if lock then .02^2 else .11^2);
    s[11]:=s[11]+.0025;kg:=s[11]/(s[11]+.0036);s[3]:=s[3]+kg*(pressureDatum-raw[158]/(rho*g)-s[3]);s[11]:=(1-kg)*s[11];
    nk:=integer(s[13]);nearest:=0;count:=0;rms:=99;
    cloud:=zeros(48,3);
    for i in 1:(if mapping then 48 else 0) loop
      beam:=4+i;r:=raw[beam];
      if r>0 then
        // A fan echo has unknown elevation. Use its beam centre for the
        // horizontal registration; depth remains pressure-aided.
        x:=ImagingSonar.mount[1]+r*cos(ImagingSonar.depression)*d[beam,1];
        y:=r*cos(ImagingSonar.depression)*d[beam,2];
        z:=ImagingSonar.mount[3]-r*sin(ImagingSonar.depression);
      else x:=0;y:=0;z:=0;end if;
      cloud[i,:]:={cp*x+sp*(sr*y+cr*z),cr*y-sr*z,-sp*x+cp*(sr*y+cr*z)};
    end for;
    bestDist:=1e9;loopId:=0;
    for i in 1:(if mapping then nk else 0) loop
      dist:=(s[1]-graph[i,1])^2+(s[2]-graph[i,2])^2+(s[3]-graph[i,3])^2;
      if dist<bestDist and cos(s[4]-graph[i,4])>.85 then bestDist:=dist;nearest:=i;end if;
    end for;
    if nearest>0 and bestDist<4 then
      (fix,count,rms):=registration(cloud,graph[nearest,:],s[1:4]);
      if count>=10 and rms<.33 then
        // Slant beams intersect a different patch of wall after translation.
        // The sparse profile has angular sampling uncertainty even for a small residual.
        measVar:=max(.12^2,rms^2);
        for k in 1:2 loop kg:=s[8+k]/(s[8+k]+measVar);s[k]:=s[k]+kg*fix[k];s[8+k]:=(1-kg)*s[8+k];end for;
        kg:=s[12]/(s[12]+.008);s[4]:=s[4]+kg*fix[3];s[12]:=(1-kg)*s[12];
        if nearest<nk-3 and t-s[15]>3 then
          votes:=0;
          for k in 1:nf loop
            for j in 1:18 loop
              if graph[nearest,152+j]>=0 and ft[k,5]==graph[nearest,152+j] then votes:=votes+1;break;end if;
            end for;
          end for;
          if votes>=3 and count>=17 and rms<.23 then loopId:=nearest;end if;
        end if;
      end if;
    end if;
    newKey:=mapping and nk==0;
    if mapping and nk>0 then newKey:=(s[1]-graph[nk,1])^2+(s[2]-graph[nk,2])^2+(s[3]-graph[nk,3])^2>1.35^2 or abs(sin(s[4]-graph[nk,4]))>.45;end if;
    if (newKey or loopId>0) and nk<keys then
      nk:=nk+1;graph[nk,1:4]:=s[1:4];
      if nk>1 then graph[nk,5:8]:=s[1:4]-graph[nk-1,1:4];end if;
      for i in 1:48 loop off:=8+3*(i-1);graph[nk,off+1:off+3]:=cloud[i,:];end for;
      graph[nk,153:170]:=ft[1:18,5];s[13]:=nk;
      if loopId>0 then
        graph[nk,171]:=loopId;graph[nk,172:175]:=s[1:4]-graph[loopId,1:4]+{fix[1],fix[2],0,fix[3]};
        s[14]:=s[14]+1;s[15]:=t;
        for iteration in 1:12 loop
          for i in 2:nk loop
            for kind in 1:2 loop
              edgeIndex:=if kind==1 then i-1 else integer(graph[i,171]);
              if edgeIndex>0 then
                innovation:=graph[i,1:4]-graph[edgeIndex,1:4]-(if kind==1 then graph[i,5:8] else graph[i,172:175]);
                innovation[4]:=atan2(sin(innovation[4]),cos(innovation[4]));
                graph[i,1:4]:=graph[i,1:4]-.38*innovation;
                if edgeIndex>1 then graph[edgeIndex,1:4]:=graph[edgeIndex,1:4]+.38*innovation;end if;
              end if;
            end for;
          end for;
        end for;
        s[1:4]:=graph[nk,1:4];
      end if;
    end if;
    s[16:20]:={nf,nm,count,rms,nearest};
    sensor[1:4]:=raw[154:157];sensor[5:8]:=raw[1:4];sensor[9:57]:=raw[5:53];sensor[58]:=correctedRate;sensor[59]:=raw[158];
    sensor[60:67]:={s[1],s[2],s[4],if lock then 1 else 0,s[5],s[6],nk,count};
    sensor[68:83]:=ring;sensor[84]:=opticalRate;sensor[85]:=if rateCount>=3 then 1 else 0;
    sensor[86:88]:=raw[162:164];sensor[89:91]:=raw[159:161];
  end estimate;

  function seaHeight
    input Real tile[32,32],x,y;output Real h;
  protected
    Integer i,j,ii,jj;Real a,b;
  algorithm
    i:=mod(integer(floor(x/2)),32)+1;j:=mod(integer(floor(y/2)),32)+1;
    ii:=mod(i,32)+1;jj:=mod(j,32)+1;a:=mod(x/2,1);b:=mod(y/2,1);
    h:=(1-a)*(1-b)*tile[j,i]+a*(1-b)*tile[j,ii]+(1-a)*b*tile[jj,i]+a*b*tile[jj,ii];
  end seaHeight;

  function boatEnvironment
    input Real t,tile[32,32],tide,wind,bearing,x,y,heading;
    output Real env[10];
  protected Real dx,dy,sea[32,32],probe[5];
  algorithm
    dx:=seaHeight(tile,x+.5,y)-seaHeight(tile,x-.5,y);dy:=seaHeight(tile,x,y+.5)-seaHeight(tile,x,y-.5);
    (sea,probe):=SpectralSea.field(t,wind,bearing,45000,13.7,tide,{x,y,tide});
    env:={wind*cos(bearing),wind*sin(bearing),.26+.12*sin(2*pi*t/44714+1.2)+.014*wind*cos(bearing),.07+.014*wind*sin(bearing),
      tide+seaHeight(tile,x,y),atan(-dx*sin(heading)+dy*cos(heading)),-atan(dx*cos(heading)+dy*sin(heading)),probe[2],probe[3],probe[4]};
  end boatEnvironment;

  function sampleFrame
    input Real t,truth[18],d[rays,3],overrideDepth[rays],useRaster,oldImage[864],previous[21],history[keys,stride];
    input Boolean camera=true,mapping=true,sonar=true;
    output Real sensor[91],elev[100],image[864],state[21],graph[keys,stride],ft[24,5],ping[65,32];
  protected Real T[3,3],depth[rays],raw[195];
  algorithm
    T:=MultiBodyLie.Math.quaternionToT(truth[4:7]);
    if useRaster>.5 then depth:=overrideDepth;else depth:=renderDepth(truth[1:3],T,d,camera,sonar);end if;
    (raw,image):=instruments(t,truth[1:3],T,truth[8:10],truth[16:18],truth[13:15],truth[12],depth,d,camera);
    if sonar and useRaster<.5 then
      (ping,raw[5:53]):=ImagingSonar.ping(t,truth[1:3],T);
    else ping:=zeros(65,32);end if;
    elev:=raw[54:153];(state,graph,sensor,ft):=estimate(t,raw,d,image,oldImage,previous,history,camera,mapping);
  end sampleFrame;
end Perception;
