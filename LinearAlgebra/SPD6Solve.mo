within LinearAlgebra;
// Shared Cholesky factorization of a 6×6 geometric innovation covariance.
// Sixteen RHS carry cross-covariance transpose plus innovation; inputs are finite.
// valid=0 rejects nonsymmetric/nonpositive covariance and returns zero solutions.
// Invalid diagonal factors use a unit value to keep following arithmetic bounded.
// Fixed six-row formulas avoid a dependent reduction-domain compiler limitation.
// Regenerate with scripts/generate-spd6-model.mjs; students can edit this source.
model SPD6Solve
  parameter Real pivot_floor = 1e-12;
  parameter Real symmetry_absolute = 1e-12;
  parameter Real symmetry_relative = 1e-8;
  input Real A[6,6] = identity(6);
  input Real B[6,16] = fill(0.0,6,16);
  output Real X[6,16];
  output Real valid;
protected
  Real L[6,6]; Real pivot[6]; Real Z[6,16]; Real solution[6,16];
  Real symmetryChecks[6,6]; Real symmetry_errors;
equation
  for i in 1:6 loop
    for j in 1:6 loop
      symmetryChecks[i,j] = if noEvent(abs(A[i,j]-A[j,i]) <=
        symmetry_absolute+symmetry_relative*abs(A[j,i])) then 0.0 else 1.0;
    end for;
  end for;
  symmetry_errors = sum(symmetryChecks[i,j] for i in 1:6, j in 1:6);
  pivot[1] = A[1,1]-(0.0);
  L[1,1] = sqrt(if noEvent(pivot[1] > pivot_floor) then pivot[1] else 1.0);
  L[1,2] = 0.0;
  L[1,3] = 0.0;
  L[1,4] = 0.0;
  L[1,5] = 0.0;
  L[1,6] = 0.0;
  L[2,1] = (A[2,1]-(0.0))/L[1,1];
  pivot[2] = A[2,2]-(L[2,1]*L[2,1]);
  L[2,2] = sqrt(if noEvent(pivot[2] > pivot_floor) then pivot[2] else 1.0);
  L[2,3] = 0.0;
  L[2,4] = 0.0;
  L[2,5] = 0.0;
  L[2,6] = 0.0;
  L[3,1] = (A[3,1]-(0.0))/L[1,1];
  L[3,2] = (A[3,2]-(L[3,1]*L[2,1]))/L[2,2];
  pivot[3] = A[3,3]-(L[3,1]*L[3,1]+L[3,2]*L[3,2]);
  L[3,3] = sqrt(if noEvent(pivot[3] > pivot_floor) then pivot[3] else 1.0);
  L[3,4] = 0.0;
  L[3,5] = 0.0;
  L[3,6] = 0.0;
  L[4,1] = (A[4,1]-(0.0))/L[1,1];
  L[4,2] = (A[4,2]-(L[4,1]*L[2,1]))/L[2,2];
  L[4,3] = (A[4,3]-(L[4,1]*L[3,1]+L[4,2]*L[3,2]))/L[3,3];
  pivot[4] = A[4,4]-(L[4,1]*L[4,1]+L[4,2]*L[4,2]+L[4,3]*L[4,3]);
  L[4,4] = sqrt(if noEvent(pivot[4] > pivot_floor) then pivot[4] else 1.0);
  L[4,5] = 0.0;
  L[4,6] = 0.0;
  L[5,1] = (A[5,1]-(0.0))/L[1,1];
  L[5,2] = (A[5,2]-(L[5,1]*L[2,1]))/L[2,2];
  L[5,3] = (A[5,3]-(L[5,1]*L[3,1]+L[5,2]*L[3,2]))/L[3,3];
  L[5,4] = (A[5,4]-(L[5,1]*L[4,1]+L[5,2]*L[4,2]+L[5,3]*L[4,3]))/L[4,4];
  pivot[5] = A[5,5]-(L[5,1]*L[5,1]+L[5,2]*L[5,2]+L[5,3]*L[5,3]+L[5,4]*L[5,4]);
  L[5,5] = sqrt(if noEvent(pivot[5] > pivot_floor) then pivot[5] else 1.0);
  L[5,6] = 0.0;
  L[6,1] = (A[6,1]-(0.0))/L[1,1];
  L[6,2] = (A[6,2]-(L[6,1]*L[2,1]))/L[2,2];
  L[6,3] = (A[6,3]-(L[6,1]*L[3,1]+L[6,2]*L[3,2]))/L[3,3];
  L[6,4] = (A[6,4]-(L[6,1]*L[4,1]+L[6,2]*L[4,2]+L[6,3]*L[4,3]))/L[4,4];
  L[6,5] = (A[6,5]-(L[6,1]*L[5,1]+L[6,2]*L[5,2]+L[6,3]*L[5,3]+L[6,4]*L[5,4]))/L[5,5];
  pivot[6] = A[6,6]-(L[6,1]*L[6,1]+L[6,2]*L[6,2]+L[6,3]*L[6,3]+L[6,4]*L[6,4]+L[6,5]*L[6,5]);
  L[6,6] = sqrt(if noEvent(pivot[6] > pivot_floor) then pivot[6] else 1.0);
  valid = if noEvent(symmetry_errors < 0.5 and
    pivot[1] > pivot_floor and pivot[2] > pivot_floor and pivot[3] > pivot_floor and pivot[4] > pivot_floor and pivot[5] > pivot_floor and pivot[6] > pivot_floor) then 1.0 else 0.0;
  for column in 1:16 loop
    Z[1,column] = (B[1,column]-(0.0))/L[1,1];
    Z[2,column] = (B[2,column]-(L[2,1]*Z[1,column]))/L[2,2];
    Z[3,column] = (B[3,column]-(L[3,1]*Z[1,column]+L[3,2]*Z[2,column]))/L[3,3];
    Z[4,column] = (B[4,column]-(L[4,1]*Z[1,column]+L[4,2]*Z[2,column]+L[4,3]*Z[3,column]))/L[4,4];
    Z[5,column] = (B[5,column]-(L[5,1]*Z[1,column]+L[5,2]*Z[2,column]+L[5,3]*Z[3,column]+L[5,4]*Z[4,column]))/L[5,5];
    Z[6,column] = (B[6,column]-(L[6,1]*Z[1,column]+L[6,2]*Z[2,column]+L[6,3]*Z[3,column]+L[6,4]*Z[4,column]+L[6,5]*Z[5,column]))/L[6,6];
    solution[6,column] = (Z[6,column]-(0.0))/L[6,6];
    solution[5,column] = (Z[5,column]-(L[6,5]*solution[6,column]))/L[5,5];
    solution[4,column] = (Z[4,column]-(L[5,4]*solution[5,column]+L[6,4]*solution[6,column]))/L[4,4];
    solution[3,column] = (Z[3,column]-(L[4,3]*solution[4,column]+L[5,3]*solution[5,column]+L[6,3]*solution[6,column]))/L[3,3];
    solution[2,column] = (Z[2,column]-(L[3,2]*solution[3,column]+L[4,2]*solution[4,column]+L[5,2]*solution[5,column]+L[6,2]*solution[6,column]))/L[2,2];
    solution[1,column] = (Z[1,column]-(L[2,1]*solution[2,column]+L[3,1]*solution[3,column]+L[4,1]*solution[4,column]+L[5,1]*solution[5,column]+L[6,1]*solution[6,column]))/L[1,1];
    for i in 1:6 loop
      X[i,column] = if noEvent(valid > 0.5) then solution[i,column] else 0.0;
    end for;
  end for;
end SPD6Solve;
