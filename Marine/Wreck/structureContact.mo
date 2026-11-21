within Marine.Wreck;
function structureContact "Six hull and thruster guard spheres against the wreck and seabed"
  input Real pos[3], T[3,3], velocity[3], omega[3];
  output Real force[3], moment[3] "World frame";
protected
  constant Real centre[6,3]=[0.49,0,0.08;-0.45,0,0.08;0.36,0.31,-0.03;0.36,-0.31,-0.03;-0.38,0.31,-0.03;-0.38,-0.31,-0.03];
  constant Real radius[6]={0.19,0.19,0.14,0.14,0.14,0.14};
  MultiBodyLie.Collision.ContactPoint point;
  MultiBodyLie.Collision.Material material(k=1300,d=0.8,mu=0.22,eps=0.0003,v_eps=0.12);
  Integer candidates[WreckSolids.maxCandidates];
  Real box[18], arm[3], axes[3,3], f[3], w[3];
algorithm
  force:=zeros(3); moment:=zeros(3); w:=transpose(T)*omega;
  candidates:=WreckSolids.contactCandidates(pos);
  for j in 1:WreckSolids.maxCandidates loop
    if candidates[j]>0 then
      box:=WreckSolids.getBox(candidates[j]);
      if abs(pos[1]-box[1])<box[16]+WreckSolids.guardExtent and
         abs(pos[2]-box[2])<box[17]+WreckSolids.guardExtent and
         abs(pos[3]-box[3])<box[18]+WreckSolids.guardExtent then
        for k in 1:3 loop
          axes[k,:]:=box[4+3*k:6+3*k];
        end for;
        for i in 1:6 loop
          arm:=transpose(T)*centre[i,:];
          point:=MultiBodyLie.Collision.sphereBox(pos,arm,radius[i],box[1:3],zeros(3),axes,box[4:6]);
          f:=MultiBodyLie.Collision.contactForce(point,velocity+cross(w,point.armA),material);
          force:=force+f;
          moment:=moment+cross(point.armA,f);
        end for;
      end if;
    end if;
  end for;
end structureContact;
