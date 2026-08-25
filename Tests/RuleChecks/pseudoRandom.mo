within Tests.RuleChecks;
function pseudoRandom "Deterministic pseudo-random stream on [-1, 1]"
  input Integer seed "Seed in 1 .. 2147483646";
  input Integer count "Number of values to draw";
  output Real values[count];
protected
  Real state;
algorithm
  state := seed;
  for i in 1:count loop
    // Park and Miller minimal standard generator. Both factors stay below
    // 2^53, so the recurrence is exact in binary64 and every tool that runs
    // this suite draws the identical points.
    state := mod(16807.0 * state, 2147483647.0);
    values[i] := 2.0 * (state / 2147483647.0) - 1.0;
  end for;
end pseudoRandom;
