within Marine.Sailing;
function jibMastPlane "Supporting plane of the lower spar, expressed in jib axes"
  input Real stations[8,3],rotation[3,3],tack[3];
  input Real mastRadius(unit="m")=.065;
  output Real plane[4] "Allowed half-space is normal dot position >= offset";
protected
  Real normal[3],axis[3],offset;
algorithm
  // The clew sweeps around the forward face when it changes sides.
  // Follow the actual sheet angle so a backed jib stays on its working side.
  axis:=stations[6,:]-stations[1,:];
  normal:={.22,-rotation[2,1],0};
  normal[3]:=-(normal[1]*axis[1]+normal[2]*axis[2])/max(.1,axis[3]);
  normal:=normal/sqrt(normal*normal);
  offset:=normal*(stations[1,:]-tack);
  for i in 2:6 loop
    offset:=max(offset,normal*(stations[i,:]-tack));
  end for;
  plane[1:3]:=transpose(rotation)*normal;
  plane[4]:=offset+mastRadius;
end jibMastPlane;
