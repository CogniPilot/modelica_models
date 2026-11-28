within Marine.Powerboats;
model EngineFuelSystem "Starter, idle shutdown and gasoline tank for a motorboat"
  parameter Real capacity(min=0,unit="l") "Usable tank capacity";
  parameter Real tankCenter[3](each unit="m") "Tank centroid in the forward, port, up hull frame";
  parameter Real initialFuelLitres(min=0,max=capacity,unit="l")=capacity;
  parameter Real idleFuelRate(min=0,unit="l/h")=3;
  parameter Real fullFuelRate(min=idleFuelRate,unit="l/h")=55;
  parameter Real fuelDensity(min=0,unit="kg/m3")=740;
  parameter Real engineOffAfter(min=.01,unit="s")=10 "Time in neutral before shutting down";
  parameter Real engineCrankTime(min=0,unit="s")=1;
  parameter Real throttleDeadband(min=0,max=1)=.008 "Smallest command that engages the engine";

  input Real throttleLever "Signed throttle command, with neutral at zero";
  output Real fuelLitres(unit="l") "Fuel remaining";
  output Real fuelMass(unit="kg") "Remaining fuel mass";
  output Real engineRunning "One when the engine can deliver power";
  output Real fuelState[6] "Litres remaining, tank capacity, litres/hour, and tank centroid";
  discrete output Real idleTime(unit="s",start=engineOffAfter,fixed=true);
  discrete output Real starterRemaining(unit="s",start=0,fixed=true);
  discrete output Real throttle(start=0,fixed=true) "Signed normalized throttle accepted by the starter";

protected
  constant Real samplePeriod(unit="s")=.01;
  constant Real timerTolerance(unit="s")=1e-8;
  Real remaining(unit="l",start=initialFuelLitres,fixed=true);
  Real consumption(unit="l/h");
equation
  when sample(0,samplePeriod) then
    starterRemaining=if remaining<=0 then 0
      elseif abs(throttleLever)>throttleDeadband and pre(idleTime)>=engineOffAfter-timerTolerance
      then engineCrankTime else max(0,pre(starterRemaining)-samplePeriod);
    idleTime=if abs(throttleLever)>throttleDeadband and remaining>0 then 0
      else min(engineOffAfter,pre(idleTime)+samplePeriod);
    throttle=if starterRemaining>timerTolerance or remaining<=0 or abs(throttleLever)<=throttleDeadband
      then 0 else max(-1,min(1,throttleLever));
  end when;
  engineRunning=noEvent(if idleTime<engineOffAfter-timerTolerance and starterRemaining<=timerTolerance and remaining>0
    then 1 else 0);
  // A reduced-order load curve, calibrated by idle and full-throttle fuel flow.
  consumption=engineRunning*(idleFuelRate+(fullFuelRate-idleFuelRate)*abs(throttle)^1.7);
  der(remaining)=-consumption/3600;
  fuelLitres=noEvent(max(0,remaining));
  fuelMass=fuelLitres*fuelDensity/1000;
  fuelState=cat(1,{fuelLitres,capacity,consumption},tankCenter);
end EngineFuelSystem;
