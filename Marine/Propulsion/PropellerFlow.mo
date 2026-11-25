within Marine.Propulsion;
model PropellerFlow "Isothermal incompressible-limit D3Q19 fluid, local lattice kernel for domain decomposition"
  function equilibrium
    input Real density, ux, uy, uz;
    output Real populations[19];
  protected
    constant Integer cx[19]={0,1,-1,0,0,0,0,1,-1,1,-1,1,-1,1,-1,0,0,0,0};
    constant Integer cy[19]={0,0,0,1,-1,0,0,1,-1,-1,1,0,0,0,0,1,-1,1,-1};
    constant Integer cz[19]={0,0,0,0,0,1,-1,0,0,0,0,1,-1,-1,1,1,-1,-1,1};
    Real eu,v2;
  algorithm
    v2:=ux^2+uy^2+uz^2;
    for i in 1:19 loop
      eu:=cx[i]*ux+cy[i]*uy+cz[i]*uz;
      populations[i]:=density*(if i==1 then 1/3 elseif i<=7 then 1/18 else 1/36)*(1+3*eu+4.5*eu^2-1.5*v2);
    end for;
  end equilibrium;

  function collision "Regularized BGK with Smagorinsky eddy viscosity and moving immersed solid"
    input Real incoming[19], solidFraction, y, z, omega, viscosity, smagorinsky;
    input Real wallTensor[6] "Symmetric velocity constraint xx, yy, zz, xy, xz, yz";
    output Real outgoing[19], bodyForce[3], velocity[3], pressure;
  protected
    constant Integer cx[19]={0,1,-1,0,0,0,0,1,-1,1,-1,1,-1,1,-1,0,0,0,0};
    constant Integer cy[19]={0,0,0,1,-1,0,0,1,-1,-1,1,0,0,0,0,1,-1,1,-1};
    constant Integer cz[19]={0,0,0,0,0,1,-1,0,0,0,0,1,-1,-1,1,1,-1,-1,1};
    Real rho,u[3],eq[19],forced[19],du[3],gap[3],dev,sxx,syy,szz,sxy,sxz,syz;
    Real tau0,tau,stress,quadratic,w;
  algorithm
    rho:=sum(incoming);
    u:={sum(cx[i]*incoming[i] for i in 1:19)/rho,
        sum(cy[i]*incoming[i] for i in 1:19)/rho,
        sum(cz[i]*incoming[i] for i in 1:19)/rho};
    gap:={0,-omega*z,omega*y}-u;
    du:=solidFraction*{wallTensor[1]*gap[1]+wallTensor[4]*gap[2]+wallTensor[5]*gap[3],
      wallTensor[4]*gap[1]+wallTensor[2]*gap[2]+wallTensor[6]*gap[3],
      wallTensor[5]*gap[1]+wallTensor[6]*gap[2]+wallTensor[3]*gap[3]};
    bodyForce:=-rho*du;
    velocity:=u+0.5*du;
    pressure:=(rho-1)/3;
    eq:=equilibrium(rho,u[1],u[2],u[3]);
    forced:=equilibrium(rho,u[1]+du[1],u[2]+du[2],u[3]+du[3]);
    sxx:=0; syy:=0; szz:=0; sxy:=0; sxz:=0; syz:=0;
    for i in 1:19 loop
      dev:=incoming[i]-eq[i];
      sxx:=sxx+dev*cx[i]^2; syy:=syy+dev*cy[i]^2; szz:=szz+dev*cz[i]^2;
      sxy:=sxy+dev*cx[i]*cy[i]; sxz:=sxz+dev*cx[i]*cz[i]; syz:=syz+dev*cy[i]*cz[i];
    end for;
    stress:=sqrt(2*(sxx^2+syy^2+szz^2+2*(sxy^2+sxz^2+syz^2)));
    tau0:=0.5+3*viscosity;
    tau:=.5*(tau0+sqrt(tau0^2+18*smagorinsky^2*stress/rho));
    for i in 1:19 loop
      w:=if i==1 then 1/3 elseif i<=7 then 1/18 else 1/36;
      quadratic:=(cx[i]^2-1/3)*sxx+(cy[i]^2-1/3)*syy+(cz[i]^2-1/3)*szz
                 +2*(cx[i]*cy[i]*sxy+cx[i]*cz[i]*sxz+cy[i]*cz[i]*syz);
      outgoing[i]:=forced[i]+(1-1/tau)*4.5*w*quadratic;
    end for;
  end collision;
  input Real populations[19],fraction,y,z,omega,nu,cs;
  input Real wallTensor[6]={1,1,1,0,0,0} "Identity gives volume no-slip. A normal projector tests free-slip on mixed blade cells";
  output Real post[19],force[3],velocity[3],pressure;
equation
  (post,force,velocity,pressure)=collision(populations,fraction,y,z,omega,nu,cs,wallTensor);
end PropellerFlow;
