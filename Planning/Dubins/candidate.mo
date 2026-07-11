within Planning.Dubins;
function candidate "Evaluate one classical Dubins path family"
  input Real startPosition[2];
  input Real startHeading;
  input Real goalPosition[2];
  input Real goalHeading;
  input Real turnRadius;
  input Planning.Dubins.PathType pathType;
  output Planning.Dubins.Candidate result;
protected
  Real dx;
  Real dy;
  Real d;
  Real direction;
  Real alpha;
  Real beta;
  Real pSquared;
  Real temporary;
  Real t;
  Real p;
  Real q;
  constant Real twoPi = 4.0 * asin(1.0);
  constant Real infeasibleLength = 1.0e100;
algorithm
  assert(turnRadius > 0.0, "Dubins turn radius must be positive");
  dx := (goalPosition[1] - startPosition[1]) / turnRadius;
  dy := (goalPosition[2] - startPosition[2]) / turnRadius;
  d := sqrt(dx * dx + dy * dy);
  direction := if d > 1.0e-14 then atan2(dy, dx) else 0.0;
  alpha := Planning.Dubins.mod2pi(startHeading - direction);
  beta := Planning.Dubins.mod2pi(goalHeading - direction);
  result.pathType := pathType;
  result.feasible := true;
  t := 0.0;
  p := 0.0;
  q := 0.0;

  if pathType == PathType.LSL then
    temporary := d + sin(alpha) - sin(beta);
    pSquared := 2.0 + d * d - 2.0 * cos(alpha - beta)
      + 2.0 * d * (sin(alpha) - sin(beta));
    if pSquared >= -1.0e-12 then
      p := sqrt(max(0.0, pSquared));
      q := atan2(cos(beta) - cos(alpha), temporary);
      t := Planning.Dubins.mod2pi(-alpha + q);
      q := Planning.Dubins.mod2pi(beta - q);
    else
      result.feasible := false;
    end if;
  elseif pathType == PathType.RSR then
    temporary := d - sin(alpha) + sin(beta);
    pSquared := 2.0 + d * d - 2.0 * cos(alpha - beta)
      + 2.0 * d * (-sin(alpha) + sin(beta));
    if pSquared >= -1.0e-12 then
      p := sqrt(max(0.0, pSquared));
      q := atan2(cos(alpha) - cos(beta), temporary);
      t := Planning.Dubins.mod2pi(alpha - q);
      q := Planning.Dubins.mod2pi(-beta + q);
    else
      result.feasible := false;
    end if;
  elseif pathType == PathType.LSR then
    pSquared := -2.0 + d * d + 2.0 * cos(alpha - beta)
      + 2.0 * d * (sin(alpha) + sin(beta));
    if pSquared >= -1.0e-12 then
      p := sqrt(max(0.0, pSquared));
      temporary := atan2(-cos(alpha) - cos(beta),
        d + sin(alpha) + sin(beta)) - atan2(-2.0, p);
      t := Planning.Dubins.mod2pi(-alpha + temporary);
      q := Planning.Dubins.mod2pi(-beta + temporary);
    else
      result.feasible := false;
    end if;
  elseif pathType == PathType.RSL then
    pSquared := d * d - 2.0 + 2.0 * cos(alpha - beta)
      - 2.0 * d * (sin(alpha) + sin(beta));
    if pSquared >= -1.0e-12 then
      p := sqrt(max(0.0, pSquared));
      temporary := atan2(cos(alpha) + cos(beta),
        d - sin(alpha) - sin(beta)) - atan2(2.0, p);
      t := Planning.Dubins.mod2pi(alpha - temporary);
      q := Planning.Dubins.mod2pi(beta - temporary);
    else
      result.feasible := false;
    end if;
  elseif pathType == PathType.RLR then
    temporary := (6.0 - d * d + 2.0 * cos(alpha - beta)
      + 2.0 * d * (sin(alpha) - sin(beta))) / 8.0;
    if abs(temporary) <= 1.0 + 1.0e-12 then
      temporary := max(-1.0, min(1.0, temporary));
      p := Planning.Dubins.mod2pi(twoPi - acos(temporary));
      t := Planning.Dubins.mod2pi(alpha
        - atan2(cos(alpha) - cos(beta), d - sin(alpha) + sin(beta))
        + 0.5 * p);
      q := Planning.Dubins.mod2pi(alpha - beta - t + p);
    else
      result.feasible := false;
    end if;
  else
    temporary := (6.0 - d * d + 2.0 * cos(alpha - beta)
      + 2.0 * d * (-sin(alpha) + sin(beta))) / 8.0;
    if abs(temporary) <= 1.0 + 1.0e-12 then
      temporary := max(-1.0, min(1.0, temporary));
      p := Planning.Dubins.mod2pi(twoPi - acos(temporary));
      t := Planning.Dubins.mod2pi(-alpha
        - atan2(cos(alpha) - cos(beta), d + sin(alpha) - sin(beta))
        + 0.5 * p);
      q := Planning.Dubins.mod2pi(beta - alpha - t + p);
    else
      result.feasible := false;
    end if;
  end if;

  result.normalizedSegmentLength := if result.feasible
    then {t, p, q} else zeros(3);
  result.length := if result.feasible
    then turnRadius * (t + p + q) else infeasibleLength;
end candidate;
