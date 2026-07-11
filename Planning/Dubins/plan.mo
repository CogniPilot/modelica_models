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
  Planning.Dubins.Candidate candidates[6];
  Integer bestIndex;
algorithm
  assert(turnRadius > 0.0, "Dubins turn radius must be positive");
  candidates[1] := Planning.Dubins.candidate(
    startPosition, startHeading, goalPosition, goalHeading,
    turnRadius, PathType.LSL);
  candidates[2] := Planning.Dubins.candidate(
    startPosition, startHeading, goalPosition, goalHeading,
    turnRadius, PathType.RSR);
  candidates[3] := Planning.Dubins.candidate(
    startPosition, startHeading, goalPosition, goalHeading,
    turnRadius, PathType.LSR);
  candidates[4] := Planning.Dubins.candidate(
    startPosition, startHeading, goalPosition, goalHeading,
    turnRadius, PathType.RSL);
  candidates[5] := Planning.Dubins.candidate(
    startPosition, startHeading, goalPosition, goalHeading,
    turnRadius, PathType.RLR);
  candidates[6] := Planning.Dubins.candidate(
    startPosition, startHeading, goalPosition, goalHeading,
    turnRadius, PathType.LRL);

  bestIndex := 1;
  for candidateIndex in 2:6 loop
    if (candidateIndex <= 4 or allowThreeTurnPaths) and
        candidates[candidateIndex].feasible and
        (not candidates[bestIndex].feasible or
         candidates[candidateIndex].length < candidates[bestIndex].length) then
      bestIndex := candidateIndex;
    end if;
  end for;

  path.startPosition := startPosition;
  path.startHeading := startHeading;
  path.goalPosition := goalPosition;
  path.goalHeading := goalHeading;
  path.turnRadius := turnRadius;
  path.pathType := candidates[bestIndex].pathType;
  path.normalizedSegmentLength :=
    candidates[bestIndex].normalizedSegmentLength;
  path.length := candidates[bestIndex].length;
  path.feasible := candidates[bestIndex].feasible;
end plan;
