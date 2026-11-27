within Tests;
model GroundVehicleTests "Independent contact height, steering and rigid offset checks"
  parameter Real parked[11]={0,0,0,0,0,0,0,0,1,0,850};
  Vehicles.Ground.SprungBicycleStep bump(
    previous=parked, ground={0.24,0,0,0}, drive=0, brake=0, steering=0, dt=0.05, boost=0);
  Vehicles.Ground.SprungBicycleStep driven(
    previous=parked, ground=zeros(4), drive=1, brake=0, steering=1, dt=0.05, boost=0);
  Vehicles.Ground.SprungBicycleStep stop(
    previous={0,0,0,0.1,0,0,0,0,1,0,900}, ground=zeros(4), drive=0, brake=1, steering=0, dt=0.05, boost=0);
  Real offsetENU[3];
equation
  offsetENU=Vehicles.Ground.bodyOffsetPointENU({4,5,6},{asin(1.0),0,0},{1,2,3});
  assert(max(abs(offsetENU-{6,4,9}))<1e-10,"Right and forward offsets at an east-facing heading");
  assert(abs(bump.wheelsENU[1,3]-bump.wheelsENU[2,3]-0.24)<1e-10 and
    bump.chassisENU[3]>0 and bump.chassisENU[3]<0.06,"Wheel contact and sprung heave differ over a bump");
  assert(driven.state[4]>0 and driven.state[3]>0 and driven.wheelSteer[2]>driven.wheelSteer[1],
    "Driving and right steering advance the vehicle with Ackermann wheel headings");
  assert(stop.state[4]==0 and stop.braking==1,"Braking brings the vehicle to rest");
end GroundVehicleTests;
