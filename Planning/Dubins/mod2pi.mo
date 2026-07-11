within Planning.Dubins;
function mod2pi "Wrap an angle to [0, 2*pi)"
  input Real angle;
  output Real wrapped;
protected
  constant Real twoPi = 4.0 * asin(1.0);
algorithm
  wrapped := atan2(sin(angle), cos(angle));
  if wrapped < 0.0 then
    wrapped := wrapped + twoPi;
  end if;
end mod2pi;
