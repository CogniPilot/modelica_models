within Marine.Wreck;
model PiccardStationHoldExample "Exercise current compensation, yaw feedback and motor disengagement"
  extends Modelica.Icons.Example;
  PiccardStationController controller;
  parameter Real currentSpeed(unit="m/s")=.30 "Prescribed current along east";
  final parameter Real requiredSurge(unit="N")=
    -controller.linearDrag*currentSpeed-controller.surgeDrag*currentSpeed*abs(currentSpeed);
equation
  controller.active=if time>=.10 and time<.70 then 1 else 0;
  controller.pose={0,0,5};
  controller.attitude=zeros(3);
  controller.gyro={0,0,if time>=.50 and time<.70 then .60 else 0};
  controller.groundSpeed=zeros(3);
  controller.bottomLock=1;
  controller.currentENU={currentSpeed,0};
  controller.pilot={0,0,if time>=.30 and time<.70 then .30 else 0,0};
  controller.pitchReference=0;
  controller.netWeight=0;
  controller.restoringMoment=zeros(3);

  when time>=.20 then
    assert(abs(controller.motors[1]-requiredSurge/2)<1e-8
      and abs(controller.motors[2]-requiredSurge/2)<1e-8,
      "The axial pair should counter the prescribed current symmetrically");
    assert(min(controller.tankTargets)>0 and max(controller.tankTargets)<=controller.tankCapacity,
      "Tank targets should stay within piston travel");
  end when;
  when time>=.40 then
    assert(controller.motors[2]>controller.motors[1],"Positive yaw should increase starboard thrust");
  end when;
  when time>=.60 then
    assert(controller.motors[2]<controller.motors[1],"Measured yaw above the target should reverse the differential");
  end when;
  when time>=.80 then
    assert(sum(abs(controller.motors))<1e-8,"A disengaged controller should command zero thrust");
  end when;

  annotation(experiment(StopTime=1,Interval=.01,Tolerance=1e-8),
    Documentation(info="<html><p>A deterministic measurement sequence exercises the standalone controller at a stationary pose in an easterly current. After engaging, it commands a turn, then reports a gyro rate above the target. The final interval checks disengagement.</p></html>"));
end PiccardStationHoldExample;
