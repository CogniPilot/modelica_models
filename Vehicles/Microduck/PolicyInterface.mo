within Vehicles.Microduck;
model PolicyInterface "Modelica signal interface for the external Pollen model_api=1 network"
  input Real forwardVelocity(unit = "m/s")=0 "m/s, robot forward";
  input Real lateralVelocity(unit = "m/s")=0 "m/s, robot left";
  input Real steeringRate(unit = "rad/s")=0 "rad/s, positive turns left";
  input Real trunkGyro[3](each unit = "rad/s")=zeros(3);
  input Real gravityRow[3]={0,0,1} "Third row of world-from-body rotation";
  input Real jointPosition[15](each unit = "rad")=zeros(15);
  input Real jointVelocity[15](each unit = "rad/s")=zeros(15);
  input Real policyHome[14](each unit = "rad")=zeros(14) "Issued by the robot's Modelica servo components";
  input Real policyHalfRange[14](each unit = "rad")=ones(14) "Issued by the robot's Modelica servo components";
  input Real policyAction[14](each unit = "rad")=zeros(14) "Raw network offsets, radians";
  input Real previousPolicyAction[14](each unit = "rad")=zeros(14);
  input Real previousCommand[3]=zeros(3) "Held Modelica output from the previous policy tick";
  input Real previousPositionCommand[15](each unit = "rad")=zeros(15);
  input Boolean hasPreviousPositionCommand=false;
  parameter Real commandAlpha=0.2 "Native robotd Walk command EMA";
  parameter Real actionScale=0.9 "Native robotd Walk radians per raw action";
  parameter Real headLowpass=0.5;
  parameter Real legsLowpass=0.7;
  output Real policyObservation[61];
  output Real filteredCommand[3];
  output Real positionCommand[15](each unit = "rad") "Native proposed joint positions, before physical joint limits";
  output Real action[15];
  output Real clippedPolicyActions;
protected
  final parameter Integer policyJoint[14]={1,2,3,4,5,6,7,8,9,11,12,13,14,15};
  Real proposedPosition[15];
  Real requestedCommand[3];
equation
  requestedCommand={forwardVelocity,lateralVelocity,steeringRate};
  for i in 1:3 loop
    filteredCommand[i]=previousCommand[i]+commandAlpha*(requestedCommand[i]-previousCommand[i]);
    policyObservation[i]=trunkGyro[i];
    policyObservation[i+3]=-gravityRow[i];
  end for;
  for i in 1:14 loop
    policyObservation[i+6]=jointPosition[policyJoint[i]]-policyHome[i];
    policyObservation[i+20]=jointVelocity[policyJoint[i]];
    policyObservation[i+34]=previousPolicyAction[i];
    action[policyJoint[i]]=max(-1,min(1,policyAction[i]/policyHalfRange[i]));
    proposedPosition[policyJoint[i]]=policyHome[i]+actionScale*policyAction[i];
  end for;
  proposedPosition[10]=0 "Native scatter skips the mouth and its home is zero";
  for j in 1:15 loop
    positionCommand[j]=if not hasPreviousPositionCommand or j==10 then proposedPosition[j]
      else if j>=6 and j<=9 then headLowpass*proposedPosition[j]+(1-headLowpass)*previousPositionCommand[j]
      else legsLowpass*proposedPosition[j]+(1-legsLowpass)*previousPositionCommand[j];
  end for;
  action[10]=-1 "Close the separate jaw; it is absent from the pretrained network";
  policyObservation[49]=filteredCommand[1];
  policyObservation[50]=filteredCommand[2];
  policyObservation[51]=filteredCommand[3];
  for i in 52:61 loop
    policyObservation[i]=0;
  end for;
  clippedPolicyActions=sum({if abs(policyAction[i])>policyHalfRange[i] then 1 else 0 for i in 1:14});
end PolicyInterface;
