within Marine.Sailing;
model PanelSailMembrane "Three-dimensional tensioned sailcloth in a boom-fixed frame"
  constant Integer cols=4;
  parameter Real cutAngle=0 "Foot angle in the unstrained sail pattern";
  parameter Integer battenCount=0;
  final parameter Integer rows=if battenCount>0 then battenCount+2 else 6, n=cols*rows;
  parameter Real battenEI[2](each unit="N.m2")={1.6,3.0} "Effective flexural rigidity at the forward and aft batten stations";
  parameter Real battenRelaxation(unit="s")=.075 "Effective bending relaxation time of fiberglass and pocket";
  parameter Real leechRelaxation(unit="s")=.025 "Effective transverse damping time of a loaded leech";
  parameter Real battenLinearMass(unit="kg/m")=.045;
  // Pocket ends measured from the Hobie sail plan. The main leech fractions
  // account for the lower clew in the simulated foot cut.
  final parameter Real battenLuff[8]=if battenCount==4 then
    {.12,.33,.51,.70,0,0,0,0} else {.09,.20,.30,.40,.50,.61,.72,.86};
  final parameter Real battenLeech[8]=if battenCount==4 then
    {.19,.38,.55,.73,0,0,0,0} else {.160,.263,.357,.455,.557,.666,.774,.886};
  function rowFractions "Luff or leech coordinates of the simulated cloth rows"
    input Integer count,rowCount;
    input Real battens[8];
    output Real fractions[rowCount];
  algorithm
    fractions[1]:=0;
    fractions[rowCount]:=1;
    for row in 2:rowCount-1 loop
      fractions[row]:=if count>0 then battens[row-1] else (row-1)/(rowCount-1);
    end for;
  end rowFractions;
  final parameter Real rowLuff[rows]=rowFractions(battenCount,rows,battenLuff);
  final parameter Real rowLeech[rows]=rowFractions(battenCount,rows,battenLeech);
  final parameter Boolean battenRow[rows]={battenCount>0 and r>1 and r<rows for r in 1:rows};
  parameter Real span=2.6,foot=1.12,rake=0,stepSize=.01,arealMass=.17;
  parameter Boolean attachedAirflow=false "Resolve attached-flow lift in addition to post-stall pressure";
  // Trapezoidal integral of the six chord fractions below.
  final parameter Real aspectRatio=span/(foot*(if footInTrack then (.5+.945+.855+.746+.53+.0075)/5 else .5075));
  final parameter Real inducedDragFactor=1/(Modelica.Constants.pi*.85*aspectRatio);
  final parameter Real liftSlope=if attachedAirflow then 2*Modelica.Constants.pi/(1+2/(.85*aspectRatio)) else 1.4;
  parameter Integer telltaleCount=2;
  parameter Integer telltaleRows[3]={2,4,5} "One-based cloth row below each telltale";
  output Real telltalePanelRows[3] "One-based cloth rows for the corresponding pair of telltales";
  parameter Real mastSupport=.68 "Fractional luff height at the stay tang";
  // Static overhanging beam under a unit tip load, normalized at the head.
  final parameter Real modeShape[rows]={bendShape(rowLuff[r],mastSupport) for r in 1:rows};
  function bendShape
    input Real z,s;
    output Real phi;
  algorithm
    phi:=if z<=s then (z^3/s-s*z)/(2*(1-s)) else
      (s*(z-s)+(1.5*(z-s)^2)-.5*(z-s)^3/(1-s))/(1-s);
  end bendShape;
  input Real mastPlane[4] "Jib contact plane in sail coordinates, zero disables contact";
  input Real mastOffset[3] "Masthead displacement resolved in sail coordinates";
  input Real leechTension "Axial load in the reinforced leech [N]";
  input Real luffShift[rows,3],shroudMid[2,3],wireK[2];
  discrete output Real wireReaction[2](each start=0,each fixed=true);
  input Real boomElevation,clewOffset "Aft travel of the clew along the boom [m]";
  parameter Boolean hasShrouds=false, footInTrack=false;
  input Real shroudFoot[2,3],shroudHead[3] "Two finite sidestays in the rotating sail frame";
  input Real air[3], water[3], omega[3], gravity[3], waterPlane[4] "Signed immersion = plane[4]+plane[1:3]*p";
  // Rows run from the tack toward the head. The luff is attached to the stay.
  // The clew at the other end of the foot is attached to the boom.
  function sectionGirth "Piecewise-linear leech outline from the sail plan"
    input Real fraction;
    input Boolean tracked;
    output Real girth;
  protected
    constant Real stations[6]={1,.945,.855,.746,.53,.015};
    Integer lower;
    Real t;
  algorithm
    if tracked then
      lower:=min(5,1+integer(5*fraction));
      t:=5*fraction+1-lower;
      girth:=(1-t)*stations[lower]+t*stations[lower+1];
    else
      girth:=1-.985*fraction;
    end if;
  end sectionGirth;
  final parameter Real nativeSpan[rows,cols]={(1-(c-1)/(cols-1))*rowLuff[r]+
    (c-1)/(cols-1)*rowLeech[r] for c in 1:cols,r in 1:rows};
  function makeMesh "Loft the cloth between corresponding luff and leech stations"
    input Real luff[:],leech[size(luff,1)],span,foot,rake,cutAngle;
    input Boolean tracked;
    output Real points[size(luff,1)*cols,3];
  protected
    Integer node;
    Real t,f,leechWidth;
  algorithm
    points:=zeros(size(luff,1)*cols,3);
    for row in 1:size(luff,1) loop
      leechWidth:=foot*sectionGirth(leech[row],tracked);
      for col in 1:cols loop
        node:=(row-1)*cols+col;
        t:=(col-1)/(cols-1);
        f:=(1-t)*luff[row]+t*leech[row];
        // Keep the unloaded broadseam shallow so sheet load can transfer
        // through the cloth to the luff. The loaded draft comes from the
        // batten flex and the pressure on the panels.
        points[node,:]:={-rake*f-leechWidth*t,-.025*sin(Modelica.Constants.pi*t)*(1-f),
          span*f+foot*sin(cutAngle)*(1-leech[row])*t};
      end for;
    end for;
  end makeMesh;
  final parameter Real mesh[n,3]=makeMesh(rowLuff,rowLeech,span,foot,rake,cutAngle,footInTrack);
  function clothLinks "Rest length and compliance of warp, weft and bias links"
    input Real rest[:,3],span;
    input Boolean battens[:],tracked;
    input Integer count;
    output Real properties[size(rest,1),4,2];
  protected
    Integer k;
    Boolean batten;
    Real d[3],length,allowance,scale;
  algorithm
    properties:=zeros(size(rest,1),4,2);
    for j in 1:size(rest,1) loop
      for link in 1:4 loop
        k:=if link==1 then j+1 elseif link==2 then j+cols elseif link==3 then j+cols+1 else j+cols-1;
        if k<=size(rest,1) and (link==2 or (link==4 and mod(j-1,cols)>0) or (link<>4 and mod(j,cols)<>0)) then
          batten:=link==1 and battens[div(j-1,cols)+1];
          d:=rest[k,:]-rest[j,:];length:=sqrt(d*d);
          allowance:=if batten then 0 elseif link==1 then .012 elseif link==2 then (if tracked then .003 else .012)
            else (if tracked then .009 else .025);
          // A shorter strip in series with its neighbours has less extension
          // at the same load. Retain the reference fabric compliance per unit span.
          scale:=if count>0 and link>1 then max(.15,abs(d[3])/(span/5)) else 1;
          properties[j,link,:]:={length*(1+allowance),.00005*scale};
        end if;
      end for;
    end for;
  end clothLinks;
  final parameter Real linkProperties[n,4,2]=clothLinks(mesh,span,battenRow,footInTrack,battenCount);
  function coarseStencil "Compatibility mapping to six evenly spaced rows"
    input Real stations[:,cols];
    output Real weights[24,2] "Lower node index and interpolation weight";
  protected
    Integer row,i;
    Real h;
  algorithm
    weights:=zeros(24,2);
    for k in 1:6 loop
      h:=(k-1)/5;
      for col in 1:cols loop
        i:=(k-1)*cols+col;
        row:=1;
        for r in 2:size(stations,1)-1 loop
          if stations[r,col]<=h then
            row:=r;
          end if;
        end for;
        weights[i,1]:=(row-1)*cols+col;
        weights[i,2]:=(h-stations[row,col])/(stations[row+1,col]-stations[row,col]);
      end for;
    end for;
  end coarseStencil;
  function coarseVertices
    input Real points[:,3];
    input Real weights[24,2];
    output Real vertices[24,3];
  protected
    Integer lower;
    Real t;
  algorithm
    vertices:=zeros(24,3);
    for j in 1:24 loop
      lower:=integer(weights[j,1]);
      t:=weights[j,2];
      vertices[j,:]:=(1-t)*points[lower,:]+t*points[lower+cols,:];
    end for;
  end coarseVertices;
  output Real legacyVertex[24,3];
  discrete output Real vertex[n,3](start=mesh,each fixed=true);
  discrete Real velocity[n,3](each start=0,each fixed=true);
  discrete output Real fluidForce[3],fluidMoment[3],wetArea,leechSpeed,clewForce[3],mastLoad[3];
  discrete output Real faceTail[6,3] "Local aerodynamic tape load in sail coordinates, port/starboard at each window";
  discrete output Real separated[6] "Incidence-based separation estimate at each ribbon";
  discrete output Real luffLevel "Airflow-weighted luffing intensity averaged across the sail";
  function sailTelltales
    input Real points[:,3],vel[size(points,1),3],wind[3],spin[3],t;
    input Integer count, windowRows[3];
    output Real tail[6,3],separation[6],luffLevel;
  protected
    Integer j;
    Real chord[3],spanwise[3],normal[3],anchor[3],flowAt[3],tangentFlow[3];
    Real signFace,speed,incidence,exposure,stall,luff,unsteady;
  algorithm
    tail:=zeros(6,3);separation:=zeros(6);luffLevel:=0;
    for k in 1:6 loop
      if k<=2*count then
        j:=(windowRows[div(k-1,2)+1]-1)*cols+1;
        chord:=points[j+1,:]-points[j,:];spanwise:=points[j+cols+1,:]-points[j+1,:];
        normal:=cross(chord,spanwise);normal:=normal/max(1e-8,sqrt(normal*normal));
        chord:=chord/max(1e-8,sqrt(chord*chord));
        anchor:=.40*points[j,:]+.12*points[j+1,:]+.48*points[j+cols+1,:];
        flowAt:=wind-cross(spin,anchor)-(.40*vel[j,:]+.12*vel[j+1,:]+.48*vel[j+cols+1,:]);
        tangentFlow:=flowAt-(flowAt*normal)*normal;
        speed:=sqrt(flowAt*flowAt);
        signFace:=if mod(k,2)==1 then 1 else -1;
        incidence:=atan2(flowAt*normal,max(.001,flowAt*chord));
        // Local tuft response proxy. Lee-side separation increases at high
        // incidence; the windward leading edge lifts as the sail feathers.
        exposure:=signFace*incidence;
        stall:=max(0,min(1,(exposure-.24)/.30));
        luff:=max(0,min(1,(.12-abs(incidence))/.12));
        luffLevel:=luffLevel+luff*min(1,max(0,(speed-1.5)/7))/(2*count);
        separation[k]:=max(stall,max(.65*luff,min(1,max(0,-flowAt*chord)/max(.01,speed))));
        unsteady:=sin(5.2*t+.53*k)+.45*sin(11.3*t+.91*k);
        tail[k,:]:=.6125*speed*((1-.9*separation[k])*tangentFlow+separation[k]*speed*
          (.68*unsteady*spanwise/max(.001,sqrt(spanwise*spanwise))+
           signFace*(.3+.2*unsteady)*normal));
      end if;
    end for;
  end sailTelltales;
  function advanceCloth
    input Real rest[:,3];
    input Integer rows,n;
    input Real old[n,3], oldV[n,3],air[3],water[3],omega[3],gravity[3],plane[4],dt,mu,t,elevation,outhaul;
    input Boolean battenRows[rows];
    input Real EI[2],bendingTime,leechTime,lineMass,links[n,4,2];
    input Boolean contact,trackedFoot;
    input Real sparPlane[4];
    input Real airLiftSlope,inducedDrag;
    input Real footStay[2,3],headStay[3],tipOffset[3],shape[rows],edgeTension,shift[rows,3],wireMid[2,3],contactK[2];
    output Real p[n,3],v[n,3],force[3],moment[3],wetArea,leechSpeed,clewForce[3],mastLoad[3],wireForce[2];
  protected
    Real invMass[n],boomArm[n],mass[n],nodal[n,3],normal[3],a[3],b[3],c[3],center[3],dv[3],q[3],load[3],relative[3];
    Real implicitFactor,gust,pressureSlope,stall,attached,normalWind,liftCoefficient,d[3],low,mid,high,wet,area,u,vn,dist,reference,extension,wa,wb,correction,restArea;
    Real spacing,relaxation,bendDiagonal[2],bendCross,determinant,bendRhs[2,3],bendLoad[2,3];
    Real battenMassWeights[4];
    Real upper[rows],rhs[rows],leftK,rightK,pivot,distLeft,distRight;
    Real panelWind[3],wire[3],wireStart[3],wireEnd[3],offset[3],denom,along,edge,penetration,loadY,totalInv,direction;
    Real wireMinX,wireMaxX,wireMinZ,wireMaxZ,wireMinSide;
    Integer left,j,k,c1,c2,c3,backRow;
  algorithm
    p:=old;v:=oldV;mass:=fill(.0001,n);nodal:=zeros(n,3);force:=zeros(3);moment:=zeros(3);wetArea:=0;
    for row in 0:rows-2 loop
      for col in 1:cols-1 loop
        left:=row*cols+col;
        for tri in 1:2 loop
          c1:=left;c2:=if tri==1 then left+1 else left+cols+1;
          c3:=if tri==1 then left+cols+1 else left+cols;
          a:=old[c1,:];b:=old[c2,:];c:=old[c3,:];
          normal:=cross(b-a,c-a);area:=.5*sqrt(normal*normal);normal:=normal/max(1e-9,2*area);
          center:=(a+b+c)/3;dv:=(oldV[c1,:]+oldV[c2,:]+oldV[c3,:])/3+cross(omega,center);
          d:={plane[4]+plane[1:3]*a,plane[4]+plane[1:3]*b,plane[4]+plane[1:3]*c};
          low:=min(d);high:=max(d);mid:=sum(d)-low-high;
          wet:=if high<=0 then 0 elseif low>=0 then 1 elseif mid<=0 then high/(high-low)*high/(high-mid)
            else 1-low/(low-high)*low/(low-mid);
          wetArea:=wetArea+wet*area;load:=zeros(3);
          for fluid in 1:2 loop
            if (fluid==1 and wet<1) or (fluid==2 and wet>0) then
              relative:=(if fluid==1 then air else water)-dv;
              if fluid==1 then
                // Synthetic crosswind gusts vary across the rig and scale with apparent wind.
                gust:=.09*sin(2.1*t+.8*center[3])+.055*sin(4.7*t-1.6*center[3])+.035*sin(7.2*t+2.9*center[3]);
                relative:=relative+gust*{-air[2],air[1],0};
              end if;
              u:=sqrt(relative*relative);vn:=relative*normal;
              if fluid==1 and airLiftSlope>1.4 then
                // Attached finite-span circulation produces lift across the
                // apparent flow. Blend to separated plate pressure at stall.
                stall:=max(0,min(1,(abs(vn)/max(.001,u)-.22)/.34));
                attached:=1-stall*stall*(3-2*stall);
                pressureSlope:=1.4+(airLiftSlope-1.4)*attached;
                implicitFactor:=1/(1+.5*dt*1.225*u*(1-wet)*pressureSlope/(mu+wet*1025*.04));
                // Dampen the panel's motion relative to the rig. Applying
                // this factor to the entire load attenuates even a fixed sail.
                panelWind:=relative+dv-cross(omega,center);
                normalWind:=panelWind*normal;
                normalWind:=normalWind+(vn-normalWind)*implicitFactor;
                liftCoefficient:=airLiftSlope*normalWind/max(.001,u);
                load:=load+.5*1.225*(1-wet)*area*(u*(1.4*(1-attached)*normalWind*normal+
                  attached*liftCoefficient*(u*normal-vn/max(.001,u)*relative))+
                  (.008+attached*inducedDrag*liftCoefficient^2)*u*relative);
              else
                // Retain the added-mass limit for submerged fabric.
                implicitFactor:=if fluid==1 then 1/(1+dt*.7*1.225*u*(1-wet)/(mu+wet*1025*.04)) else 1/(1+dt*.7*1025*u*wet/(mu+wet*1025*.04));
                load:=load+implicitFactor*.5*(if fluid==1 then 1.225*(1-wet) else 1025*wet)*area*
                  (1.4*u*vn*normal+.008*u*(relative-vn*normal));
              end if;
            end if;
          end for;
          force:=force+load;moment:=moment+cross(center,load);
          q:=cross(rest[c2,:]-rest[c1,:],rest[c3,:]-rest[c1,:]);restArea:=.5*sqrt(q*q);
          for corner in 1:3 loop
            j:=if corner==1 then c1 elseif corner==2 then c2 else c3;
            mass[j]:=mass[j]+restArea*(mu+wet*1025*.04)/3;
            nodal[j,:]:=nodal[j,:]+load/3;
          end for;
        end for;
      end for;
    end for;
    for r in 1:rows loop
      if battenRows[r] then
        for col in 1:cols-1 loop
          j:=(r-1)*cols+col;
          spacing:=links[j,1,1];
          mass[j]:=mass[j]+.5*spacing*lineMass;
          mass[j+1]:=mass[j+1]+.5*spacing*lineMass;
        end for;
      end if;
    end for;
    for i in 1:n loop
      invMass[i]:=if mod(i-1,cols)==0 or i==cols or (trackedFoot and i<=cols) then 0 else 1/mass[i];
      boomArm[i]:=if i==cols or (trackedFoot and i<=cols) then (i-1)/(cols-1) else 0;
      if invMass[i]>0 then
        p[i,:]:=old[i,:]+dt*((1-.7*dt)*oldV[i,:]+dt*(gravity+nodal[i,:]/mass[i]));
      end if;
    end for;
    mastLoad:=zeros(3);wireForce:=zeros(2);
    for r in 1:rows loop
      j:=(r-1)*cols+1;
      p[j,:]:=rest[j,:]+shape[r]*tipOffset+shift[r,:];
      mastLoad:=mastLoad+shape[r]*(nodal[j,:]+mass[j]*gravity);
    end for;
    // Transfer each track reaction at its own lever arm as an equivalent clew load.
    clewForce:=zeros(3);
    for i in 2:cols loop
      if boomArm[i]>0 then
        along:=rest[i,1]-outhaul*boomArm[i];
        p[i,:]:={along*cos(elevation),0,-along*sin(elevation)}+(if trackedFoot then shift[1,:] else zeros(3));
        clewForce:=clewForce+boomArm[i]*(nodal[i,:]+mass[i]*gravity);
      end if;
    end for;
    // Tension-only warp, weft and bias links can still collapse when unloaded.
    for pass in 1:6 loop
      for index in 1:n loop
        j:=if mod(pass,2)==0 then n+1-index else index;
        for link in 1:4 loop
          k:=if link==1 then j+1 elseif link==2 then j+cols elseif link==3 then j+cols+1 else j+cols-1;
          if k<=n and (link==2 or (link==4 and mod(j-1,cols)>0) or (link<>4 and mod(j,cols)<>0)) then
            q:=p[k,:]-p[j,:];dist:=sqrt(q*q);
            reference:=links[j,link,1];
            // Fiberglass carries axial compression as well as tension. Preserve the
            // segment length while allowing the pocket to curve through its joints.
            if link==1 and battenRows[div(j-1,cols)+1] then
              extension:=dist-reference;
            else
              extension:=max(0,dist-reference);
            end if;
            wa:=invMass[j];
            wb:=invMass[k];
            correction:=extension/(max(1e-8,dist)*(wa+wb+links[j,link,2]/dt^2));
            if mod(j-1,cols)==0 then
              mastLoad:=mastLoad+shape[div(j-1,cols)+1]*correction*q/dt^2;
            end if;
            if mod(k-1,cols)==0 then
              mastLoad:=mastLoad-shape[div(k-1,cols)+1]*correction*q/dt^2;
            end if;
            clewForce:=clewForce+(boomArm[j]-boomArm[k])*correction*q/dt^2;
            p[j,:]:=p[j,:]+wa*correction*q;p[k,:]:=p[k,:]-wb*correction*q;
          end if;
        end for;
      end for;
    end for;
    // Implicit transverse balance along the loaded leech. Pocket bending
    // alone cannot damp rotations of a whole batten about the luff. The loaded
    // reinforced edge dissipates motion between consecutive batten tips.
    if edgeTension>0 then
      upper:=zeros(rows);rhs:=zeros(rows);
      relaxation:=leechTime/dt;
      rhs[1]:=p[cols,2];rhs[rows]:=p[n-cols+1,2];
      for r in 2:rows-1 loop
        j:=r*cols;
        d:=rest[j,:]-rest[j-cols,:];distLeft:=max(.001,sqrt(d*d));
        d:=rest[j+cols,:]-rest[j,:];distRight:=max(.001,sqrt(d*d));
        leftK:=(1+relaxation)*edgeTension/distLeft;
        rightK:=(1+relaxation)*edgeTension/distRight;
        pivot:=mass[j]/dt^2+leftK+rightK-leftK*upper[r-1];
        upper[r]:=rightK/pivot;
        rhs[r]:=(mass[j]/dt^2*p[j,2]+leftK*rhs[r-1]+relaxation*edgeTension*
          ((old[j,2]-old[j-cols,2])/distLeft+(old[j,2]-old[j+cols,2])/distRight))/pivot;
      end for;
      for i in 2:rows-1 loop
        backRow:=rows+1-i;rhs[backRow]:=rhs[backRow]+upper[backRow]*rhs[backRow+1];p[backRow*cols,2]:=rhs[backRow];
      end for;
      d:=rest[2*cols,:]-rest[cols,:];
      clewForce[2]:=clewForce[2]+edgeTension*((1+relaxation)*(rhs[2]-rhs[1])
        -relaxation*(old[2*cols,2]-old[cols,2]))/max(.001,sqrt(d*d));
      d:=rest[n,:]-rest[n-cols,:];
      mastLoad[2]:=mastLoad[2]+edgeTension*((1+relaxation)*(rhs[rows-1]-rhs[rows])
        -relaxation*(old[n-cols,2]-old[n,2]))/max(.001,sqrt(d*d));
    end if;
    // The leech tension solve moves the ends of the battens. Equilibrate
    // the axial bar constraints with those updated endpoints before contact.
    for pass in 1:4 loop
      for r in 2:rows loop
        for col in 1:cols-1 loop
          k:=if mod(pass,2)==1 then col else cols-col;
          j:=(r-1)*cols+k;
          if battenRows[r] then
            reference:=links[j,1,1];
            q:=p[j+1,:]-p[j,:];dist:=sqrt(q*q);
            correction:=(dist-reference)/(max(1e-8,dist)*(invMass[j]+invMass[j+1]+.000005/dt^2));
            p[j,:]:=p[j,:]+invMass[j]*correction*q;
            p[j+1,:]:=p[j+1,:]-invMass[j+1]*correction*q;
            if k==1 then
              mastLoad:=mastLoad+shape[r]*correction*q/dt^2;
            end if;
          end if;
        end for;
      end for;
    end for;
    // The two curvature stations share the inner nodes. Solve their coupled
    // implicit bending and pocket damping together so one correction cannot
    // undo the other. The luff is pinned and takes the root reaction.
    relaxation:=bendingTime/dt;
    for r in 2:rows-1 loop
      if battenRows[r] then
        j:=(r-1)*cols+1;
        battenMassWeights:=invMass[j:j+3];
        for station in 1:2 loop
          k:=j+station;
          d:=rest[k+1,:]-rest[k-1,:];
          spacing:=max(.001,.5*sqrt(d*d));
          bendDiagonal[station]:=spacing^3/(EI[station]*dt^2)+(1+relaxation)*
            (battenMassWeights[station]+4*battenMassWeights[station+1]+battenMassWeights[station+2]);
          q:=p[k-1,:]-2*p[k,:]+p[k+1,:];
          bendRhs[station,:]:=q+relaxation*(q-(old[k-1,:]-2*old[k,:]+old[k+1,:]));
        end for;
        bendCross:=-2*(1+relaxation)*(battenMassWeights[2]+battenMassWeights[3]);
        determinant:=bendDiagonal[1]*bendDiagonal[2]-bendCross^2;
        bendLoad[1,:]:=(bendDiagonal[2]*bendRhs[1,:]-bendCross*bendRhs[2,:])/determinant;
        bendLoad[2,:]:=(bendDiagonal[1]*bendRhs[2,:]-bendCross*bendRhs[1,:])/determinant;
        mastLoad:=mastLoad-shape[r]*bendLoad[1,:]/dt^2;
        p[j+1,:]:=p[j+1,:]+battenMassWeights[2]*(2*bendLoad[1,:]-bendLoad[2,:]);
        p[j+2,:]:=p[j+2,:]+battenMassWeights[3]*(2*bendLoad[2,:]-bendLoad[1,:]);
        p[j+3,:]:=p[j+3,:]-battenMassWeights[4]*bendLoad[2,:];
      end if;
    end for;
    // Intersect the projected shroud with all three edges of every cloth
    // triangle in the sail XZ plane. Signed separation varies linearly over
    // the intersection interval, so its extrema occur on those edges.
    if contact then
      for pass in 1:3 loop
        for side in 1:2 loop
          for half in 1:2 loop
            wireStart:=if half==1 then footStay[side,:] else wireMid[side,:];
            wireEnd:=if half==1 then wireMid[side,:] else headStay;
            wire:=wireEnd-wireStart;
            direction:=if side==1 then 1 else -1;
            // Reject separated edge bounds before the projected intersection.
            wireMinX:=min(wireStart[1],wireEnd[1])-1e-9;
            wireMaxX:=max(wireStart[1],wireEnd[1])+1e-9;
            wireMinZ:=min(wireStart[3],wireEnd[3])-1e-9;
            wireMaxZ:=max(wireStart[3],wireEnd[3])+1e-9;
            wireMinSide:=min(direction*wireStart[2],direction*wireEnd[2])-.009-1e-9;
            for j in 1:n loop
              for link in 1:3 loop
                k:=if link==1 then j+1 elseif link==2 then j+cols else j+cols+1;
                if k<=n and (link==2 or mod(j,cols)<>0) then
                  if (invMass[j]>0 or invMass[k]>0)
                    and max(p[j,1],p[k,1])>=wireMinX and min(p[j,1],p[k,1])<=wireMaxX
                    and max(p[j,3],p[k,3])>=wireMinZ and min(p[j,3],p[k,3])<=wireMaxZ
                    and max(direction*p[j,2],direction*p[k,2])>=wireMinSide then
                    d:=p[k,:]-p[j,:];offset:=wireStart-p[j,:];
                    denom:=d[1]*wire[3]-d[3]*wire[1];
                    if abs(denom)>1e-9 then
                      edge:=(offset[1]*wire[3]-offset[3]*wire[1])/denom;
                      along:=(offset[1]*d[3]-offset[3]*d[1])/denom;
                      if edge>=0 and edge<=1 and along>=0 and along<=1 then
                        penetration:=direction*(p[j,2]+edge*d[2]-wireStart[2]-along*wire[2])+.009;
                        if penetration>0 then
                          wa:=invMass[j];
                          wb:=invMass[k];
                          totalInv:=wa*(1-edge)^2+wb*edge^2;
                          if totalInv>1e-8 then
                            loadY:=-direction*penetration/(totalInv+1/(max(1,contactK[side])*dt^2));
                            wireForce[side]:=wireForce[side]-loadY/dt^2;
                            p[j,2]:=p[j,2]+wa*(1-edge)*loadY;
                            p[k,2]:=p[k,2]+wb*edge*loadY;
                            clewForce[2]:=clewForce[2]+(boomArm[j]*(1-edge)+boomArm[k]*edge)*loadY/dt^2;
                          end if;
                        end if;
                      end if;
                    end if;
                  end if;
                end if;
              end for;
            end for;
          end for;
        end for;
      end for;
    end if;
    if sparPlane[4]<>0 or sparPlane[1]<>0 then
      // A shared supporting plane keeps the entire linear triangle outside
      // the spar even when its corners pass on opposite sides of the tube.
      // Leave the forestay attachments fixed. The sheet follows the clew.
      for i in 1:n loop
        if mod(i-1,cols)<>0 then
          penetration:=sparPlane[4]-sparPlane[1:3]*p[i,:];
          if penetration>0 then
            p[i,:]:=p[i,:]+penetration*sparPlane[1:3];
          end if;
        end if;
      end for;
    end if;
    v:=(p-old)/dt;
    leechSpeed:=sqrt(v[div(rows,2)*cols,:]*v[div(rows,2)*cols,:]);
  end advanceCloth;
equation
  telltalePanelRows={telltaleRows[i] for i in 1:3};
  legacyVertex=if battenCount>0 then coarseVertices(vertex,coarseStencil(nativeSpan)) else vertex[1:24,:];
  when sample(0,stepSize) then
    (vertex,velocity,fluidForce,fluidMoment,wetArea,leechSpeed,clewForce,mastLoad,wireReaction)=advanceCloth(mesh,rows,n,pre(vertex),pre(velocity),air,water,omega,gravity,waterPlane,stepSize,arealMass,time,boomElevation,clewOffset,battenRow,battenEI,battenRelaxation,(if battenCount>0 then leechRelaxation else 0),battenLinearMass,linkProperties,hasShrouds,footInTrack,mastPlane,liftSlope,inducedDragFactor,shroudFoot,shroudHead,mastOffset,modeShape,leechTension,luffShift,shroudMid,wireK);
    (faceTail,separated,luffLevel)=sailTelltales(vertex,velocity,air,omega,time,telltaleCount,telltaleRows);
  end when;
end PanelSailMembrane;
