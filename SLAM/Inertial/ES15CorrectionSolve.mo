within SLAM.Inertial;
// The reference uses strict positive definiteness, without a teaching pivot floor.
// All symmetry checks, rejected solves, and sixteen RHS stay in SPD6Solve.
model ES15CorrectionSolve
  import SPD6Solve = LinearAlgebra.SPD6Solve;

  extends SPD6Solve(pivot_floor=0.0);
end ES15CorrectionSolve;
