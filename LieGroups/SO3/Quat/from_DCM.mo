within LieGroups.SO3.Quat;
function from_DCM "Convert 3x3 rotation matrix to unit quaternion (Shepperd's method)"
  input Real R[3,3] "Rotation matrix";
  output Real q[4] "Unit quaternion {w,x,y,z}";
protected
  Real tr;
  Real b1, b2, b3, b4;
  constant Real eps = 1e-10;
algorithm
  tr := R[1,1] + R[2,2] + R[3,3];

  // Compute every branch scale factor at top level. The GALEC branch lowering
  // keeps only the last assignment written inside a conditional branch, so the
  // quaternion is assembled with a single whole-vector assignment per branch
  // and no scalar assignment is left inside a branch to be dropped.
  b1 := 0.5 * sqrt(max(1.0 + tr, eps));
  b2 := 0.5 * sqrt(max(1.0 + R[1,1] - R[2,2] - R[3,3], eps));
  b3 := 0.5 * sqrt(max(1.0 - R[1,1] + R[2,2] - R[3,3], eps));
  b4 := 0.5 * sqrt(max(1.0 - R[1,1] - R[2,2] + R[3,3], eps));

  if tr > 0 then
    // Case 1: trace > 0 (w is largest)
    q := {b1,
          (R[3,2] - R[2,3]) / max(4.0*b1, eps),
          (R[1,3] - R[3,1]) / max(4.0*b1, eps),
          (R[2,1] - R[1,2]) / max(4.0*b1, eps)};
  elseif R[1,1] > R[2,2] and R[1,1] > R[3,3] then
    // Case 2: R[1,1] is largest diagonal
    q := {(R[3,2] - R[2,3]) / max(4.0*b2, eps),
          b2,
          (R[1,2] + R[2,1]) / max(4.0*b2, eps),
          (R[1,3] + R[3,1]) / max(4.0*b2, eps)};
  elseif R[2,2] > R[3,3] then
    // Case 3: R[2,2] is largest diagonal
    q := {(R[1,3] - R[3,1]) / max(4.0*b3, eps),
          (R[1,2] + R[2,1]) / max(4.0*b3, eps),
          b3,
          (R[2,3] + R[3,2]) / max(4.0*b3, eps)};
  else
    // Case 4: R[3,3] is largest diagonal
    q := {(R[2,1] - R[1,2]) / max(4.0*b4, eps),
          (R[1,3] + R[3,1]) / max(4.0*b4, eps),
          (R[2,3] + R[3,2]) / max(4.0*b4, eps),
          b4};
  end if;

  // Roundoff or a mildly noisy input matrix can perturb the norm.
  q := LieGroups.SO3.Quat.normalize(q);

  // Ensure positive w convention. If-EXPRESSION (not a no-else if-statement) so
  // AD/both-branch compilers evaluate the condition correctly.
  q := if q[1] < 0 then -q else q;
end from_DCM;
