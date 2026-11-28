within Marine.Mooring;
partial model PartialCruiserAnchor "Rigid anchor and tension-only windlass rode"
  inner MultiBodyLie.World world(n={0,0,-1});
  parameter Real stowed[3]={0,0,1};
  parameter Real mass=11,flukeDepth=.12;
  parameter Real holdingForce(unit="N")=2800 "Assumed fully buried horizontal capacity in sand";
  parameter Real breakoutForce(unit="N")=1200 "Assumed fully buried vertical breakout capacity";
  parameter Real rodeStiffness(unit="N/m")=1800;
  parameter Real rodeBrakeLoad(unit="N")=4200 "Load at which the windlass brake slips";
  parameter Real setDistance(unit="m")=.45 "Drag distance for one e-fold of fluke embedment";
  parameter Real bedStiffness(unit="N/m")=9500 "Recoverable horizontal soil stiffness";
  output Real embedment(start=0,fixed=true) "Mobilised fraction of fluke holding capacity";
  MultiBodyLie.Parts.Body body(m=mass,I_11=.11,I_22=.16,I_33=.11,
    angles_fixed=true,w_0_fixed=true,r_0(start=stowed,each fixed=true,each stateSelect=StateSelect.always),
    v_0(each fixed=true,each stateSelect=StateSelect.always),Q(each stateSelect=StateSelect.always),
    w_a(each stateSelect=StateSelect.always));
  MultiBodyLie.Forces.WorldForceAndTorque loads;
  input Real lower,sea,mount[3],mountVelocity[3],current[2];
  input Real guidedLength(unit="m")=0 "Rode length supported by guides fixed to the vessel";
  output Real horizontalSpeed "Speed over seabed of the anchor [m/s]";
  output Real position[3],orientation[4],reaction[3],tension,paid(start=.08,fixed=true),grounded;
protected
  Real bedAtMount(unit="m"),bedAtAnchor(unit="m") "Local ENU seabed elevations under the mount and the anchor";
  Real lead[3],distance,direction[3],rate,rodeTarget,depth,bed,normal,drag[3],horizontal[2],
    friction[2],submerged,wetVelocity[3],stowForce[3],linkForce[3],unbrakedTension;
  Real bedDeflection[2](each start=0,each fixed=true),elasticLoad,shearLimit,plasticRate,
    pullAngleFactor,uplift,breakout,suction;
equation
  connect(body.frame_a,loads.frame_b);
  position=body.r_0;
  orientation=body.Q;
  depth=max(.1,mount[3]-bedAtMount);
  rodeTarget=if lower>.5 then min(62,max(4,5*depth)) else .08;
  der(paid)=noEvent(if lower>.5 then
    max(0,min(3.35,2*(rodeTarget-paid)))+
      max(0,min(20,(unbrakedTension-rodeBrakeLoad)/175))*min(1,max(0,70-paid))
    else -min(1.65,2*max(0,paid-rodeTarget)));
  lead=mount-position;
  distance=sqrt(lead*lead);
  direction=lead/max(.001,distance);
  rate=(mountVelocity-body.v_0)*direction;
  // A finite brake load limits shock load on the much lighter anchor.
  unbrakedTension=noEvent(max(0,rodeStiffness*(distance+guidedLength-paid)+
    92*rate*min(1,max(0,(distance+guidedLength-paid)/.08))));
  tension=noEvent(min(1.20*rodeBrakeLoad,unbrakedTension));
  // Guide the shank into its roller once the rode reaches the retrieval stop.
  stowForce=if noEvent(paid<.22 and lower<.5) then 3800*lead+230*(mountVelocity-body.v_0)
    else zeros(3);
  linkForce=tension*direction+stowForce;
  reaction=-linkForce;
  bed=bedAtAnchor;
  grounded=if noEvent(position[3]<bed+flukeDepth+.012) then 1 else 0;
  normal=if grounded>.5 then max(0,9000*(bed+flukeDepth-position[3])-270*body.v_0[3]) else 0;
  horizontal=body.v_0[1:2];
  // Drag embeds the flukes under a shallow lead. An upward pull can extract
  // them even while the shank is still touching bottom.
  horizontalSpeed=sqrt(horizontal*horizontal);
  pullAngleFactor=max(0,min(1,(.65-direction[3])/.30));
  uplift=max(0,linkForce[3]);
  breakout=max(0,min(1,(uplift-60-breakoutForce*embedment)/200));
  der(embedment)=if noEvent(grounded>.5) then
    (1-embedment)*horizontalSpeed/setDistance*pullAngleFactor*min(1,tension/90)
    -4*breakout*embedment else -4*embedment;
  suction=if noEvent(grounded>.5) then min(uplift,breakoutForce*embedment) else 0;
  shearLimit=.55*normal+holdingForce*embedment;
  elasticLoad=bedStiffness*sqrt(bedDeflection*bedDeflection);
  // An elastic bed holds a static load at zero slip speed. Plastic yielding
  // lets the anchor plough beyond its finite holding capacity.
  plasticRate=25*max(0,elasticLoad-shearLimit)/max(1,elasticLoad);
  der(bedDeflection)=if noEvent(grounded>.5) then horizontal-plasticRate*bedDeflection
    else -20*bedDeflection;
  friction=if noEvent(grounded>.5) then
    -bedStiffness*bedDeflection*min(1,shearLimit/max(1,elasticLoad))-65*horizontal
    else zeros(2);
  submerged=max(0,min(1,(sea-position[3])/.16));
  wetVelocity=body.v_0-{current[1],current[2],0};
  drag= -(1.0+13*submerged*sqrt(wetVelocity*wetVelocity))*wetVelocity;
  loads.force=linkForce+drag+{friction[1],friction[2],normal-suction+12*submerged};
  loads.torque=transpose(body.frame_a.R.T)*(-2.8*body.w_a);
end PartialCruiserAnchor;
