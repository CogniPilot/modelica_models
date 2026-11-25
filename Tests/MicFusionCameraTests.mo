within Tests;
model MicFusionCameraTests "Visible and thermal optics, mount stops, rainfall and wiping"
  import MIC = Vision.Sensors.MicFusion9000;
  parameter Real wide[4]=MIC.visibleIntrinsics(1);
  parameter Real tele[4]=MIC.visibleIntrinsics(30);
  parameter Real infrared[4]=MIC.thermalIntrinsics();
  MIC.UprightMount mount;
  Real wet[8,3];
  Real blade[4];
  Real thermalWet[8,3];
  Real thermalBlade[4];
equation
  mount.requestedPanRate = 5;
  mount.requestedTiltRate = if time < 3 then 5 elseif time < 7 then -5 else 0;
  (wet,blade) = MIC.windowView(time,7.99,1,false,4,1,20);
  (thermalWet,thermalBlade) = MIC.windowView(time,7.99,1,true,4,1,20);
  assert(abs(2*atan(960/wide[1])-63.7*MIC.rad)<1e-9 and abs(2*atan(960/tele[1])-2.3*MIC.rad)<1e-9,
    "Visible zoom endpoints");
  assert(abs(2*atan(240/infrared[2])-9.3*MIC.rad)<1e-9 and abs(infrared[3]-320)<1e-9, "Thermal geometry");
  assert(MIC.bandRadiance(300)>54 and MIC.bandRadiance(300)<56 and MIC.bandRadiance(310)>MIC.bandRadiance(300),
    "Room-temperature reference radiance and monotonicity");
  assert(abs(panError)<1e-5 and mount.tilt<=MIC.tiltLimits[2]+1e-6 and mount.tilt>=MIC.tiltLimits[1]-1e-6,"Mount limits");
  assert(abs(blade[3]-thermalBlade[3]-2)<1e-9,"Both windows share one blade");
  when time >= 2 then
    assert(abs(mount.tilt-MIC.tiltLimits[2])<1e-5 and wet[1,3]>0,"Upper stop and first droplets");
  end when;
  when time >= 5 then
    assert(wet[1,3]<1e-9 and thermalWet[1,3]<1e-9,"A full sweep cleared early droplets on both windows");
  end when;
  when time >= 7 then
    assert(abs(mount.tilt-MIC.tiltLimits[1])<1e-5,"Tilt reverses away from upper stop and reaches lower stop");
  end when;
  when time >= 29 then
    assert(max(wet[:,3])<1e-9 and max(thermalWet[:,3])<1e-9,"Dry after rain ends");
  end when;
protected
  Real panError=mount.pan-time*MIC.panRateLimit;
  annotation(experiment(StopTime=30,Interval=0.05));
end MicFusionCameraTests;
