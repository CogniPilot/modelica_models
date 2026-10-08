The SE(3) Jacobian used to reset ESKF covariance evaluated its closed coefficients above 0.1 rad. In generated binary32 arithmetic, subtraction cancellation in those coefficients remains significant at the ESKF attitude correction limit of 0.15 rad.

The change extends the existing Taylor branch to 0.2 rad by changing its squared-angle threshold from 0.01 to 0.04. The coefficients, tangent convention, correction limit and estimator tuning remain unchanged.

The focused checker compiled the actual Modelica sources with Rumoca 0.10.2 before and after the change. It evaluated 624 identical binary32 tangents against an independent 24-term Lie-adjoint series in float64. The cases cover zero, small rotations, both sides of the old threshold and the 0.15-rad correction limit, with random translation and velocity components. Before: 62 failed cases, maximum entry error 7.867240928e-05. After: no failed cases, maximum entry error 1.033423331e-06. The fixed acceptance tolerances are absolute 3e-6 and relative 4e-6, with generated runtime status required to be zero.

before.json and after.json retain the commands, generated-source hashes, input hash, checker hash and binary hash. The before source is commit 67d1bf2; both results use the committed checker. This is a numerical reset regression, not a claim of better flight RMS, covariance calibration or superiority over native EKF2/EKF3. The terrain experiment remains outside this commit.

Reproduce with the project Python environment and Rumoca 0.10.2:
python tools/estimator_comparison/check_se23_reset.py --source-root . --work "$HOME/scratch/modelica_models/se23-reset-check" --cache-dir "$HOME/scratch/modelica_models/se23-reset-cache" --rumoca /path/to/rumoca --cc /path/to/cc
Choose a fresh work directory. Owned generated code, logs and binaries remain there.

Lean validation is separate from the binary32 check. ResetSeries.lean proves error bounds for all five Taylor coefficients from GNC's Taylor theorem and mathlib's sine/cosine derivative bounds, without assumed coefficient error estimates. For 0 < angle <= 0.2 rad, each exact-real coefficient differs from its closed form by at most 2e-10. The branch-coverage theorem proves that corrections in [0, 0.15] lie strictly below the new squared-angle threshold. The numerical check separately includes zero rotation and actual floating-point evaluation. The formal certificate does not establish full nonlinear filter boundedness or superiority over native estimators.

check_theory.py rechecks the source with GNC's pinned Lean 4.29.1 and audits every public theorem for axioms, allowing only propext, Classical.choice and Quot.sound. Use --cache /path/to/gnc_lean/.lake --lean /path/to/lean --build "$HOME/scratch/modelica_models/reset-proof" --output "$HOME/scratch/modelica_models/reset-proof.json". Dependencies reuse the checked GNC/mathlib cache; project checkouts are not modified.
