within Marine.Wildlife;
model HerringGull "Flap-gliding herring gull with paired wing strips and a trimming tail"
  parameter Real mass=.705,span=1.35,area=.200,rho=1.225;
  parameter Real startState[8]={4,-12,8,0,13.4,0,0,0};
  parameter Real startMemory[7]={0,0,.015,0,0,0,1};
  parameter Real startWaterMode=0 "0 flying, 1 swimming awaiting release, 2 swimming ready";
  parameter Real beatFrequency=3.13,rollInertia=.027,pitchInertia=.0057;
  parameter Real maxMusclePower=35 "Combined positive aerodynamic work limit [W]";
  input Real wind[2],seaLevel,offset,glideOnly;
  input Real gust[2]=zeros(2) "Local wind minus the mean, ENU [m/s]";
  input Real waterCurrent[2]=zeros(2),surfaceVerticalSpeed=0;
  input Real waterSlope[2]=zeros(2) "Local elevation gradients east and north";
  input Real pilot[4]=zeros(4) "Manual enable, climb, left bank, flap or glide";
  input Real straightFlight=0 "Hold initial heading for straight-flight comparisons",trimSpeed=12.7;
  discrete Real flapping(start=startMemory[7],fixed=true);
  output Real x[8](start=startState,each fixed=true) "ENU, heading, horizontal airspeed, vertical speed, phase and stroke amplitude";
  output Real wings[4] "Shoulder elevation, folded wrist angle, sweep and left bank";
  output Real memory[7] "Bank, roll rate, nose-up pitch, pitch rate, differential incidence, tail incidence, flap latch";
  output Real power "Positive aerodynamic work rate of the two wings [W]";
  output Real moments[2] "Roll and pitch moments about the centre of mass [N m]";
  output Real tailLift "Positive upward tail force in the longitudinal plane [N]";
  output Real surfaceMotion[6] "Immersion, local airspeed, water-relative speed, water mode, port and starboard paddle commands";
protected
  Real bank(start=startMemory[1],fixed=true),rollRate(start=startMemory[2],fixed=true);
  Real pitch(start=startMemory[3],fixed=true),pitchRate(start=startMemory[4],fixed=true);
  Real differential(start=startMemory[5],fixed=true),tailTrim(start=startMemory[6],fixed=true);
  Real zTarget,orbit,airCourse,headingError,bankTarget,drive,rate,stroke,sweep,strokeRate;
  Real gamma,pressure,refCl,desiredVz,pitchTarget,tailCommand,rollCommand;
  Real arm[4],side[4],strokeVz[4],inflow[4],stripCl[4],stripCd[4],stripQ[4],fx[4],fz[4];
  Real chordwiseFlow,stripVertical[4],stripFlow[4],sinInflow[4],cosInflow[4];
  Real lift,forward,scale,work,tailAngle,tailCl,tailPressure,tailDrag,bodyDrag;
  Real courseE,courseN,airSpeed,gustCross,groundSpeed,waterSpeed,waterSide,windSide;
  Real depth,wet,swimCommand,launchCommand,runAssist,wingLift,waterThrust,waterDrag,buoyancy;
  Real wetRoll,wetPitch,wetYaw,yawRate;
  discrete Real waterMode(start=startWaterMode,fixed=true);
  Real takeoffRequested;
  Real waterCoupling,paddleTurn,portPaddle,starboardPaddle;
equation
  courseE=cos(x[4]);courseN=sin(x[4]);
  airSpeed=max(.25,x[5]-gust[1]*courseE-gust[2]*courseN);
  gustCross=-gust[1]*courseN+gust[2]*courseE;
  groundSpeed=x[5]+wind[1]*courseE+wind[2]*courseN;
  waterSpeed=groundSpeed-waterCurrent[1]*courseE-waterCurrent[2]*courseN;
  windSide=-wind[1]*courseN+wind[2]*courseE;
  waterSide=-waterCurrent[1]*courseN+waterCurrent[2]*courseE+.035*(windSide+gustCross);

  // The belly first immerses 9.3 cm below the body origin. A further 5 cm
  // of draft displaces the gull's weight. Damping opposes motion relative
  // to the water and the immersion fades both forces in at contact.
  depth=max(0,seaLevel+.093-x[3]);
  wet=min(1,depth/.05);
  buoyancy=max(0,mass*9.81*depth/.05-14*wet*(x[6]-surfaceVerticalSpeed));
  swimCommand=if noEvent(pilot[1]>.5) then min(1,max(0,pilot[4])) else 0;
  takeoffRequested=if noEvent(pilot[1]>.5 and pilot[2]>.15 and pilot[4]>.15 and glideOnly<.5) then 1 else 0;
  // An immersed landing latches the swim mode across brief loss of contact
  // on wave crests. A launch requires a new climb-and-flap gesture.
  when sample(0,.05) then
    waterMode=if pre(waterMode)<.5 then (if wet>.45 then 1 else 0)
      elseif pre(waterMode)<1.5 then (if takeoffRequested>.5 then 1 else 2)
      elseif takeoffRequested>.5 and x[3]-seaLevel>.35 and x[6]>surfaceVerticalSpeed then 0
      else 2;
  end when;
  launchCommand=if noEvent(takeoffRequested>.5 and (waterMode<.5 or waterMode>1.5))
    then min(1,pilot[2])*swimCommand else 0;
  waterCoupling=if waterMode>.5 then max(wet,1-launchCommand) else wet;
  // A running launch uses leg strokes first. Low-speed wing thrust and lift
  // then fade into the strip model as flying speed and height increase.
  runAssist=launchCommand*min(1,max(0,(12.5-airSpeed)/4))*min(1,max(0,(7-(x[3]-seaLevel))/4));
  wingLift=mass*runAssist*max(0,12-4*max(0,x[6]))*
    min(1,max(0,(max(waterSpeed,.55*airSpeed)-2.1)/1.8));
  // The feet can stroke in opposite directions, turning the bird in place
  // with no net forward paddle thrust.
  paddleTurn=if noEvent(pilot[1]>.5) then pilot[3] else 0;
  portPaddle=swimCommand-.48*paddleTurn;
  starboardPaddle=swimCommand+.48*paddleTurn;
  waterThrust=mass*(portPaddle+starboardPaddle+7.5*runAssist);
  waterDrag=(.65+.95*(1-runAssist))*waterSpeed+(.65-.47*runAssist)*waterSpeed*abs(waterSpeed);
  wetRoll=atan(-waterSlope[1]*courseN+waterSlope[2]*courseE);
  wetPitch=.23+atan(waterSlope[1]*courseE+waterSlope[2]*courseN);
  wetYaw=.85*(starboardPaddle-portPaddle)/.96;

  zTarget=seaLevel+9+2*offset;
  when sample(0,.1) then
    flapping=if airSpeed<trimSpeed-1.1 or x[3]<zTarget-.8 then 1
      elseif airSpeed>trimSpeed+.2 and x[3]>zTarget+.6 then 0 else pre(flapping);
  end when;
  // These input modes are held between communication points. Their changes
  // already arrive at a step boundary, so they need no continuous root search.
  drive=(1-waterCoupling*(1-launchCommand))*(if noEvent(glideOnly>.5 or (pilot[1]>.5 and pilot[4]<-.15)) then 0
    elseif noEvent(pilot[1]>.5 and pilot[4]>.15) then 1 else flapping);
  rate=2*3.14159265*(if waterMode>.5 and launchCommand<.15 then
    .85+1.55*max(abs(portPaddle),abs(starboardPaddle)) else beatFrequency);
  der(x[8])=(drive-x[8])/.32;
  der(x[7])=rate;
  stroke=.095+.64*x[8]*sin(x[7]);
  strokeRate=.64*(der(x[8])*sin(x[7])+x[8]*rate*cos(x[7]));
  sweep=x[8]*(1+cos(x[7]))/2;
  orbit=if noEvent(straightFlight>.5) then startState[4] else
    atan2(x[2]+15,x[1]-3)+1.571+atan((sqrt((x[2]+15)^2+(x[1]-3)^2)-23-6*offset)/13);
  // Crab into the crosswind to hold the requested course over the harbour.
  airCourse=if noEvent(straightFlight>.5) then orbit else orbit+asin(max(-.94,min(.94,
    ((wind[1]+gust[1])*sin(orbit)-(wind[2]+gust[2])*cos(orbit))/max(2,airSpeed))));
  headingError=atan2(sin(airCourse-x[4]),cos(airCourse-x[4]));
  bankTarget=if noEvent(pilot[1]>.5) then .70*pilot[3] else max(-.70,min(.70,1.25*headingError));
  rollCommand=max(-.095,min(.095,.16*(bankTarget-bank)-.040*rollRate));
  der(differential)=max(-.95,min(.95,(rollCommand-differential)/.10));

  gamma=atan2(x[6],max(1,airSpeed));
  pressure=.5*rho*(airSpeed^2+x[6]^2);
  refCl=mass*9.81/max(1,pressure*area*cos(bank));
  desiredVz=(if noEvent(pilot[1]>.5) then 3*pilot[2]+max(0,min(2,.9*(seaLevel+2-x[3])))
            elseif noEvent(glideOnly>.5) then -.65 else max(-1,min(1.35,.55*(zTarget+2.3-x[3]))))+
            max(-2,min(1.5,.75*(airSpeed-trimSpeed)));
  pitchTarget=max(-.45,min(.52,gamma+refCl/4.7-.065+.075*(desiredVz-x[6])));
  tailCommand=max(-.30,min(.30,-.73*(pitchTarget-pitch)+.076*pitchRate));
  der(tailTrim)=max(-2.5,min(2.5,(tailCommand-tailTrim)/.065));

  chordwiseFlow=max(1,airSpeed);
  for j in 1:4 loop
    side[j]=if j<=2 then -1 else 1;
    arm[j]=span*(if mod(j,2)==1 then .12 else .36)*(1-(if mod(j,2)==1 then .03 else .27)*sweep);
    strokeVz[j]=arm[j]*strokeRate*cos(stroke);
    stripVertical[j]=x[6]+strokeVz[j]+side[j]*arm[j]*rollRate;
    inflow[j]=atan2(stripVertical[j],chordwiseFlow);
    // Wrist flexion reduces the outer upstroke area and distal feathers twist
    // toward the local inflow. Opposite incidence changes roll the bird.
    stripCl[j]=max(-.70,min(1.48,4.7*(.065+pitch+side[j]*differential-inflow[j]
      +(if mod(j,2)==1 then .26 else .47)*(inflow[j]-gamma))));
    stripCd[j]=.012+.045*stripCl[j]^2;
    stripQ[j]=.125*rho*area*(airSpeed^2+stripVertical[j]^2)*
      (1-(if mod(j,2)==1 then .04 else .30)*sweep);
    // Rotate the foil force with the normalized inflow components.
    stripFlow[j]=sqrt(chordwiseFlow^2+stripVertical[j]^2);
    sinInflow[j]=stripVertical[j]/stripFlow[j];
    cosInflow[j]=chordwiseFlow/stripFlow[j];
    fx[j]=stripQ[j]*(-stripCl[j]*sinInflow[j]-stripCd[j]*cosInflow[j]);
    fz[j]=stripQ[j]*( stripCl[j]*cosInflow[j]-stripCd[j]*sinInflow[j])*cos(stroke);
  end for;
  work=sum({max(0,-fz[j]*strokeVz[j]) for j in 1:4});
  scale=min(1,maxMusclePower/max(.001,work));power=scale*work;
  tailAngle=pitch-atan2(x[6]-.255*pitchRate,max(1,airSpeed))+tailTrim-.16*refCl;
  tailCl=max(-.95,min(.95,3.15*tailAngle));
  tailPressure=.82*pressure*.024;
  tailLift=tailPressure*tailCl;
  tailDrag=tailPressure*(.014+.12*tailCl^2);
  lift=scale*sum(fz)+tailLift*cos(gamma)-tailDrag*sin(gamma);
  bodyDrag=.00077*airSpeed*airSpeed;
  forward=scale*sum(fx)-tailLift*sin(gamma)-tailDrag*cos(gamma)-bodyDrag;
  // Positive left bank means the starboard wing rises. The aft tail sees
  // increased effective incidence in a nose-up pitch rate and damps the rate.
  moments[1]=scale*sum({side[j]*arm[j]*fz[j] for j in 1:4});
  moments[2]=(-.006-.015*sweep)*scale*sum(fz)-.255*tailLift-.050*pitchRate;
  der(bank)=rollRate;
  der(rollRate)=(1-waterCoupling)*moments[1]/rollInertia+waterCoupling*(43*(wetRoll-bank)-10*rollRate);
  der(pitch)=pitchRate;
  der(pitchRate)=(1-waterCoupling)*moments[2]/pitchInertia+waterCoupling*(52*(wetPitch-pitch)-12*pitchRate);
  der(x[1])=groundSpeed*courseE-(windSide+waterCoupling*(waterSide-windSide))*courseN;
  der(x[2])=groundSpeed*courseN+(windSide+waterCoupling*(waterSide-windSide))*courseE;
  der(x[3])=x[6];
  // Crosswind on the aft tail turns the bill toward the oncoming air.
  yawRate=(1-waterCoupling)*(lift*sin(bank)/(mass*max(2,airSpeed))-
    .38*gustCross/max(2,airSpeed))+waterCoupling*wetYaw;
  der(x[4])=yawRate;
  der(x[5])=((1-waterCoupling)*(forward+6*mass*runAssist)+waterCoupling*(waterThrust-waterDrag))/mass
    -waterCoupling*windSide*yawRate;
  der(x[6])=((1-waterCoupling)*lift*cos(bank)+buoyancy+wingLift)/mass-9.81;
  wings={stroke,-.40*sweep,.50*sweep,bank};
  memory={bank,rollRate,pitch,pitchRate,differential,tailTrim,flapping};
  surfaceMotion={wet,airSpeed,waterSpeed,waterMode,waterCoupling*portPaddle,waterCoupling*starboardPaddle};
end HerringGull;
