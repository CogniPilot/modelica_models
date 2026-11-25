within Marine.Propulsion;
model CruiserPropulsion "Open-water propeller and finite-aspect foil in the inboard cruiser's body frame"
  parameter Real waterDensity(unit="kg/m3")=1025;
  parameter Real propellerDiameter(unit="m")=.46;
  final parameter Real referencePitchRatio=.95 "P/D of the reference series curve";
  parameter Real propellerCenter[3](each unit="m")={-3.11,0,-.95};
  parameter Real shaftInclination(unit="rad")=atan(.36/1.8);
  final parameter Real shaftAxis[3]={cos(shaftInclination),0,sin(shaftInclination)};
  parameter Real rudderStock[3](each unit="m")={-3.50,0,-.89};
  parameter Real rudderChord(unit="m")=.34;
  parameter Real rudderSpan(unit="m")=.43;
  parameter Real wakeFraction=.10 "Effective hull wake fraction ahead";
  parameter Real thrustDeduction=.06 "Reduction from open-water thrust to effective hull thrust";
  parameter Real reverseThrustFactor=.55,reverseTorqueFactor=.72 "Unmeasured astern corrections";
  parameter Real maximumRudder(unit="rad")=.52;
  parameter Real maximumRudderRate(unit="rad/s")=.85;
  // Oosterveld and van Oossanen (1975), Table 5. Each cubic collects terms
  // at Z=3, AE/A0=.50, P/D=.95, Re=2e6. Exponents here run from zero to three.
  parameter Real thrustCurve[4]={.3853974837,-.2449743044,-.1894542159,.0641471649};
  parameter Real torqueCurve[4]={.05401982266,-.03122365829,-.01941178288,.002751653757};
  parameter Real curveLimit=1.05 "Upper bound of the positive loading fit";

  input Real shaftRevolutions(unit="1/s") "Signed shaft revolutions per second";
  input Real helm "Port-positive command";
  input Real propellerInflow[3](each unit="m/s") "Uninduced body velocity relative to water at the shaft";
  input Real rudderInflow[3](each unit="m/s") "Uninduced body velocity relative to water at the foil";
  input Real upInBody[3] "World up expressed in body axes";
  input Real propellerDepth(unit="m"),rudderDepth(unit="m") "Positive below the local waterline";
  output Real axialAdvance(unit="m/s"),advanceCoefficient;
  output Real advanceCoefficientValid "One when the shaft is turning fast enough to report a ratio";
  output Real thrustCoefficient,torqueCoefficient;
  output Real propellerThrust(unit="N") "Signed open-water thrust after wet-area and astern corrections";
  output Real shaftTorque(unit="N.m") "Signed hydrodynamic torque absorbed by the propeller";
  output Real shaftPower(unit="W") "Mechanical power absorbed at the propeller";
  output Real diskInduction(unit="m/s"),slipstreamAtRudder(unit="m/s"),rudderWashFraction;
  output Real propellerWetFraction,rudderWetFraction;
  output Real rudderAngle(unit="rad",start=0,fixed=true) "Signed rotation about the actual stock";
  output Real rudderCenter[3](each unit="m");
  output Real propellerForce[3](each unit="N"),rudderForce[3](each unit="N"),shaftReaction[3](each unit="N.m");
protected
  constant Real pi=Modelica.Constants.pi;
  final parameter Real diskArea(unit="m2")=pi*propellerDiameter^2/4;
  final parameter Real rudderArea(unit="m2")=rudderChord*rudderSpan;
  final parameter Real rudderAspect=rudderSpan/rudderChord;
  Real n(unit="1/s"),direction,j;
  Real diskHeight(unit="m"),diskIntersection,coastFraction,coastingDrag(unit="N");
  Real washPoint[3](each unit="m"),axialDistance(unit="m"),jetIncrement(unit="m/s"),jetRadius(unit="m");
  Real incoming(unit="m/s"),jetHalfHeight(unit="m"),jetCenterZ(unit="m"),wetSpan(unit="m");
  Real wetBottom(unit="m"),wetTop(unit="m"),washedSpan(unit="m"),upVertical;
  Real ambientForce[2](each unit="N"),washForce[2](each unit="N");
  function foilForce "Normal force from the finite-aspect lift slope and profile drag"
    input Real u(unit="m/s"),v(unit="m/s"),angle(unit="rad"),area(unit="m2"),aspect,rho(unit="kg/m3");
    output Real force[2](each unit="N");
  protected
    Real s,c,speed(unit="m/s"),normalFlow,pressure(unit="Pa"),normalForce(unit="N"),drag(unit="N"),liftSlope;
  algorithm
    s:=sin(angle);c:=cos(angle);
    speed:=sqrt(u*u+v*v);
    normalFlow:=(v*c-u*s)/max(.001,speed);
    pressure:=.5*rho*speed^2;
    // Fujii slope as used in the MMG standard. The bounded normal coefficient
    // approximates the progressive loss of lift at high incidence.
    liftSlope:=6.13*aspect/(aspect+2.25);
    normalForce:=-pressure*area*liftSlope*normalFlow/sqrt(1+(normalFlow/.57)^2);
    drag:=.012*pressure*area/max(.001,speed);
    force:={-s*normalForce-drag*u,c*normalForce-drag*v};
  end foilForce;
equation
  n=abs(shaftRevolutions);
  direction=noEvent(if shaftRevolutions<0 then -1 else 1);
  axialAdvance=(1-wakeFraction)*(shaftAxis*propellerInflow);
  advanceCoefficientValid=noEvent(if n*propellerDiameter>.1 then 1 else 0);
  advanceCoefficient=noEvent(if n*propellerDiameter>.1 then direction*axialAdvance/(n*propellerDiameter) else 0);
  j=noEvent(min(curveLimit,max(0,direction*axialAdvance/max(.1,n*propellerDiameter))));
  thrustCoefficient=noEvent(max(0,thrustCurve[1]+j*(thrustCurve[2]+j*(thrustCurve[3]+j*thrustCurve[4]))));
  torqueCoefficient=noEvent(max(0,torqueCurve[1]+j*(torqueCurve[2]+j*(torqueCurve[3]+j*torqueCurve[4]))));
  diskHeight=.5*propellerDiameter*sqrt(max(.01,1-(shaftAxis*upInBody)^2));
  diskIntersection=noEvent(min(1,max(-1,propellerDepth/diskHeight)));
  propellerWetFraction=.5+(asin(diskIntersection)+diskIntersection*sqrt(max(0,1-diskIntersection^2)))/pi;
  propellerThrust=direction*(if noEvent(shaftRevolutions<0) then reverseThrustFactor else 1)
    *waterDensity*n^2*propellerDiameter^4*thrustCoefficient*propellerWetFraction;
  shaftTorque=direction*(if noEvent(shaftRevolutions<0) then reverseTorqueFactor else 1)
    *waterDensity*n^2*propellerDiameter^5*torqueCoefficient*propellerWetFraction;
  shaftPower=2*pi*shaftRevolutions*shaftTorque;
  coastFraction=noEvent(min(1,max(0,1-n*propellerDiameter*curveLimit/max(.1,abs(axialAdvance)))));
  coastingDrag=.5*waterDensity*.18*diskArea*propellerWetFraction*coastFraction*axialAdvance*abs(axialAdvance);
  propellerForce=shaftAxis*((1-thrustDeduction)*propellerThrust-coastingDrag);
  shaftReaction=-shaftAxis*shaftTorque;

  der(rudderAngle)=noEvent(min(maximumRudderRate,max(-maximumRudderRate,
    (-maximumRudder*min(1,max(-1,helm))-rudderAngle)/.18)));
  // The stock is a quarter chord behind the leading edge of this foil.
  rudderCenter=rudderStock;
  upVertical=max(.1,upInBody[3]);
  wetBottom=rudderCenter[3]-.5*rudderSpan;
  wetTop=noEvent(max(wetBottom,min(rudderCenter[3]+.5*rudderSpan,rudderCenter[3]+rudderDepth/upVertical)));
  wetSpan=wetTop-wetBottom;
  rudderWetFraction=wetSpan/rudderSpan;

  incoming=noEvent(max(0,axialAdvance));
  // Actuator-disk axial momentum: T=2 rho A (V_A+v_i) v_i. Aft rudder wash
  // is confined to a contracted streamtube and exists only under ahead thrust.
  diskInduction=sqrt(.25*incoming^2+noEvent(max(0,propellerThrust))
    /(2*waterDensity*diskArea*max(.01,propellerWetFraction)))-.5*incoming;
  // Sample the jet at mid-chord. Vortex-cylinder induction grows from vi at
  // the disk to 2 vi downstream. Continuity sets the local streamtube radius.
  washPoint=rudderStock-.25*rudderChord*{cos(rudderAngle),sin(rudderAngle),0};
  axialDistance=(propellerCenter[1]-washPoint[1])/shaftAxis[1];
  jetIncrement=diskInduction*(1+axialDistance/sqrt(axialDistance^2+(.5*propellerDiameter)^2));
  slipstreamAtRudder=jetIncrement*shaftAxis[1];
  jetRadius=.5*propellerDiameter*sqrt(if noEvent(incoming+jetIncrement>1e-8) then
    (incoming+diskInduction)/(incoming+jetIncrement) else 1);
  jetHalfHeight=sqrt(max(0,jetRadius^2-(washPoint[2]-propellerCenter[2])^2))/shaftAxis[1];
  jetCenterZ=propellerCenter[3]-axialDistance*shaftAxis[3];
  washedSpan=noEvent(max(0,min(wetTop,jetCenterZ+jetHalfHeight)-max(wetBottom,jetCenterZ-jetHalfHeight)));
  rudderWashFraction=(washedSpan/rudderSpan)*diskInduction/(diskInduction+.01);
  ambientForce=foilForce(rudderInflow[1],rudderInflow[2],rudderAngle,
    rudderArea*(rudderWetFraction-rudderWashFraction),rudderAspect,waterDensity);
  washForce=foilForce(rudderInflow[1]+slipstreamAtRudder,rudderInflow[2],rudderAngle,
    rudderArea*rudderWashFraction,rudderAspect,waterDensity);
  rudderForce={ambientForce[1]+washForce[1],ambientForce[2]+washForce[2],0};
end CruiserPropulsion;
