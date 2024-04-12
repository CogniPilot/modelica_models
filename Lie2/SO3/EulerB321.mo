within Lie2.SO3;

model EulerB321

  extends Euler(final sequence = {3, 2, 1}, final body=true);
  
  function fromMatrix
    extends BaseType.fromMatrix;
  protected
    Real phi, theta, psi;
  algorithm
    theta := asin(-a[3, 1]);
    if (abs(theta - pi/2) < eps) then
      phi := 0;
      psi := atan2(a[2, 3], a[1, 3]);
    elseif (abs(theta + pi/2) < eps) then
      phi := 0;
      psi := atan2(-a[2, 3], -a[1, 3]);
    else
      phi := atan2(a[3, 2], a[3, 3]);
      psi := atan2(a[2, 1], a[1, 1]);
    end if;
    res := {psi, theta, phi};
    annotation(Inline = true);
  end fromMatrix;

end EulerB321;