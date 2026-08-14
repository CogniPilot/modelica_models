within Planning.Dubins;
function plan "Select the shortest feasible path from all six Dubins families"
  input Real startPosition[2];
  input Real startHeading;
  input Real goalPosition[2];
  input Real goalHeading;
  input Real turnRadius;
  input Boolean allowThreeTurnPaths = true
    "Include RLR and LRL candidates in the classical shortest-path search";
  output Planning.Dubins.Path path;
protected
  // The six families are held in separate record locals rather than a
  // `Candidate[6]` array: rumoca rejects assigning a function result into an
  // element of a record array (EX002 "record value does not match its checked
  // field layout"), which made this function — the entry point of the whole
  // Dubins chain — unevaluable on that toolchain. A single record local is
  // supported, so the selection is written straight-line over six of them.
  Planning.Dubins.Candidate lsl;
  Planning.Dubins.Candidate rsr;
  Planning.Dubins.Candidate lsr;
  Planning.Dubins.Candidate rsl;
  Planning.Dubins.Candidate rlr;
  Planning.Dubins.Candidate lrl;
  Planning.Dubins.Candidate best;
algorithm
  assert(turnRadius > 0.0, "Dubins turn radius must be positive");
  lsl := Planning.Dubins.candidate(
    startPosition, startHeading, goalPosition, goalHeading,
    turnRadius, PathType.LSL);
  rsr := Planning.Dubins.candidate(
    startPosition, startHeading, goalPosition, goalHeading,
    turnRadius, PathType.RSR);
  lsr := Planning.Dubins.candidate(
    startPosition, startHeading, goalPosition, goalHeading,
    turnRadius, PathType.LSR);
  rsl := Planning.Dubins.candidate(
    startPosition, startHeading, goalPosition, goalHeading,
    turnRadius, PathType.RSL);
  rlr := Planning.Dubins.candidate(
    startPosition, startHeading, goalPosition, goalHeading,
    turnRadius, PathType.RLR);
  lrl := Planning.Dubins.candidate(
    startPosition, startHeading, goalPosition, goalHeading,
    turnRadius, PathType.LRL);

  // Equivalent to the former `bestIndex` scan starting at LSL: a candidate
  // wins only when it is feasible and either the incumbent is infeasible or
  // the candidate is strictly shorter. Ties keep the earlier family, so the
  // family preference order LSL, RSR, LSR, RSL, RLR, LRL is preserved.
  best := lsl;
  if rsr.feasible and (not best.feasible or rsr.length < best.length) then
    best := rsr;
  end if;
  if lsr.feasible and (not best.feasible or lsr.length < best.length) then
    best := lsr;
  end if;
  if rsl.feasible and (not best.feasible or rsl.length < best.length) then
    best := rsl;
  end if;
  if allowThreeTurnPaths and rlr.feasible and
      (not best.feasible or rlr.length < best.length) then
    best := rlr;
  end if;
  if allowThreeTurnPaths and lrl.feasible and
      (not best.feasible or lrl.length < best.length) then
    best := lrl;
  end if;

  path.startPosition := startPosition;
  path.startHeading := startHeading;
  path.goalPosition := goalPosition;
  path.goalHeading := goalHeading;
  path.turnRadius := turnRadius;
  path.pathType := best.pathType;
  path.normalizedSegmentLength := best.normalizedSegmentLength;
  path.length := best.length;
  path.feasible := best.feasible;
end plan;
