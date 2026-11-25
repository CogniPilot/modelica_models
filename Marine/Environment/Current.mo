within Marine.Environment;
package Current "Residual, tidal and wind-driven water velocity"
  constant Real profile[3,3]=[0,.26,.07;8,.19,.04;16,.08,.02];
  function shelterAt
    input Real pos[3](each unit="m") "Position in the reference wreck scene, ENU";
    output Real shelter;
  algorithm
    shelter:=1-.78*(.5+.5*tanh(pos[1]-1))*(.5+.5*tanh(28-pos[1]))*(.5+.5*tanh(4-abs(pos[2])))*(.5+.5*tanh(-6-pos[3]));
  end shelterAt;
  function meanVelocity
    input Real z(unit="m"),tide(unit="m"),wind(unit="m/s"),bearing(unit="rad");
    input Real tidalEast(unit="m/s"),residual[2](each unit="m/s"),shelter "ENU residual and local speed multiplier";
    output Real velocity[3](each unit="m/s") "Mean water velocity in ENU";
  algorithm
    velocity:=shelter*({residual[1]+tidalEast,residual[2],0}+
      .014*wind*exp(min(0,z-tide)/2.2)*{cos(bearing),sin(bearing),0});
  end meanVelocity;
end Current;
