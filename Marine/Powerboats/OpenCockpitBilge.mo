within Marine.Powerboats;
model OpenCockpitBilge "Water retained by an open cockpit with scuppers and a switched bilge pump"
  import Modelica.Units.SI;
  constant SI.Acceleration gravity=Modelica.Constants.g_n;
  parameter SI.Area cockpitArea(min=Modelica.Constants.small)=2.9 "Effective unobstructed sole area";
  parameter SI.Length rimHeight(min=0)=.34 "Height of the cockpit rim above the sole";
  parameter SI.Length crestWidth[4](each min=0)=fill(.35,4) "Effective overtopping length at each corner";
  parameter SI.Area scupperArea[2](each min=0)=fill(Modelica.Constants.pi*(.019)^2,2);
  parameter SI.VolumeFlowRate pumpCapacity(min=0)=.0011 "Nominal pump flow with a covered pickup";
  parameter SI.Volume pumpPickupVolume(min=0)=.003 "Water left below the pump pickup";
  parameter SI.Volume initialVolume(min=0,max=cockpitArea*rimHeight)=0;
  parameter Real weirCoefficient(min=0)=.42;
  parameter Real scupperCoefficient(min=0)=.62;
  input SI.Length rimDepth[4] "Local sea elevation minus rim elevation at each corner";
  input SI.Length scupperDepth[2] "Local sea elevation minus the outlet elevation";
  input Real upright "Cosine of the angle between the vessel's up axis and world up";
  input Real pumpCommand=0 "Pump runs above 0.5 when the pickup is covered";
  output SI.Volume volume "Water currently retained in the cockpit";
  output SI.Length standingDepth "Mean depth above the sole, equivalent with the boat level";
  output SI.VolumeFlowRate inflow "Overtopping water captured by the cockpit";
  output SI.VolumeFlowRate pumpFlow;
  output SI.VolumeFlowRate gravityDrain "One-way scuppers and spillback over the rim";
protected
  final parameter SI.Volume capacity=cockpitArea*rimHeight;
  final parameter SI.Volume meteringVolume=.002 "Flow taper near empty or full";
  SI.Volume storedWater(start=initialVolume,fixed=true,stateSelect=StateSelect.always);
  SI.Length reducedRimHeight,spillHead;
  SI.VolumeFlowRate scupperFlow[2],spillFlow;
  Real captureFraction,availableWater,pumpUpright;
equation
  volume=noEvent(min(capacity,max(0,storedWater)));
  standingDepth=volume/cockpitArea;
  captureFraction=noEvent(max(0,min(1,upright/.20)));
  pumpUpright=noEvent(max(0,min(1,upright/.50)));
  availableWater=noEvent(max(0,min(1,volume/meteringVolume)));
  inflow=noEvent(weirCoefficient*sqrt(2*gravity)*captureFraction*
    sum({crestWidth[i]*max(0,rimDepth[i])^1.5 for i in 1:4})*
    min(1,max(0,(capacity-storedWater)/meteringVolume)));
  reducedRimHeight=rimHeight*captureFraction*noEvent(max(0,upright));
  spillHead=noEvent(max(0,standingDepth-reducedRimHeight));
  spillFlow=weirCoefficient*sum(crestWidth)*sqrt(2*gravity)*spillHead^1.5;
  for i in 1:2 loop
    scupperFlow[i]=noEvent(scupperCoefficient*scupperArea[i]*
      sqrt(2*gravity*max(0,standingDepth-max(0,scupperDepth[i]))));
  end for;
  gravityDrain=availableWater*(sum(scupperFlow)+spillFlow);
  pumpFlow=noEvent(if pumpCommand>.5 then pumpCapacity*pumpUpright*
    max(0,min(1,(volume-pumpPickupVolume)/meteringVolume)) else 0);
  der(storedWater)=inflow-gravityDrain-pumpFlow;
  annotation(Documentation(info="<html><p>A reduced-order cockpit water balance. Overtopping follows a rectangular-weir approximation. One-way scuppers discharge according to the difference between interior water head and exterior water head. Water spills back over the rim as the boat heels or overturns.</p><p>The pump runs independently of propulsion. A submerged scupper can block gravity drainage while a covered pump pickup can still discharge. The square-root scupper law and retention estimate assume a connected cockpit without sealed compartments.</p></html>"));
end OpenCockpitBilge;
