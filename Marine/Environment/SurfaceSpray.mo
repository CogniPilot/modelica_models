within Marine.Environment;
model SurfaceSpray "Water diverted at a moving waterline and film draining off an emergent edge"
  parameter Real gravity=9.81, density=1025, surfaceTension=.074;
  // Slots may represent either wetted bow sections or elements of a fluke edge.
  input Real sections[20,4] "Wetted cross-section (m2), forward speed (m/s), immersion rate (m/s), time since emersion (s, negative while submerged)";
  output Real release[20,4] "Atomized volume flux (m3/s), lateral and vertical launch speeds (m/s), residual film flux per unit edge (m2/s)";
  function source
    input Real area,speed,entry,age,g,rho,sigma;
    output Real outflow[4];
  protected
    Real impingement,capillarySpeed,breaking,flux;
  algorithm
    impingement:=sqrt((.27*speed)^2+max(0,entry)^2);
    capillarySpeed:=(4*g*sigma/rho)^.25;
    breaking:=max(0,1-(capillarySpeed/max(.001,impingement))^2);
    flux:=max(0,area)*impingement*breaking;
    // A finite film inventory drains from each metre of newly exposed edge.
    outflow:={.045*flux,.72*impingement,.85*impingement,if age>=0 then .0009/.48*exp(-age/.48) else 0};
  end source;
equation
  for i in 1:20 loop
    release[i,:]=source(sections[i,1],sections[i,2],sections[i,3],sections[i,4],gravity,density,surfaceTension);
  end for;
end SurfaceSpray;
