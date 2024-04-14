within Lie2.SO3.Group;

model EulerB321

  extends Euler(final sequence = {3, 2, 1}, final body=true);
  
  function fromMatrix
    extends BaseType.fromMatrix;
  protected
    Real phi, theta, psi;
  algorithm
    theta := asin(-a[3, 1]);
    if (abs(theta - Constants.pi/2) < Constants.eps) then
      phi := 0;
      psi := atan2(a[2, 3], a[1, 3]);
    elseif (abs(theta + Constants.pi/2) < Constants.eps) then
      phi := 0;
      psi := atan2(-a[2, 3], -a[1, 3]);
    else
      phi := atan2(a[3, 2], a[3, 3]);
      psi := atan2(a[2, 1], a[1, 1]);
    end if;
    res := {psi, theta, phi};
    annotation(Inline = true);
  end fromMatrix;

  function Dr
    extends BaseType.Dr;
  protected
    Real psi, theta, phi;
  algorithm
    psi := a[1];
    theta := a[2];
    phi := a[3];
    res := {
      w[2]*sin(phi)/cos(theta) + w[3]*cos(phi)/cos(theta), // psi_dot
      w[2]*cos(phi) - w[3]*sin(phi), // theta_dot
      w[1] + w[2]*sin(phi)*tan(theta) + w[3]*cos(phi)*tan(theta) // phi_dot
      };
    annotation(Inline = true);
  end Dr;

end EulerB321;
