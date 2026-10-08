Modelica synthetic landmark camera
8 October 2026

SLAM.Simulation.LandmarkCamera executes the camera observation mathematics in
Modelica. A supplied fixed landmark set or generated planar grid is projected
using body pose, camera extrinsics and calibrated intrinsics or centered FOV.
Explicit error, detection and availability inputs support matched experiments
without image processing. Visibility checks both geometry and noisy observation
against image bounds and optical depth limits. Absent observations remain zero
and carry a false validity flag; consumers must honor that flag.

The actual public block was exported with Rumoca 0.10.2 and compiled locally.
256 cases check independent randomized geometry, visibility and negative cases
for a supplied four-point cloud. The default twelve-point grid, FOV-derived
calibration and stable identities are also checked on every case. Visibility
matches the independent oracle in all cases. Maximum projection error is
8.264e-5 in the observation entries; the default scene agrees exactly.
All finite-input cases have zero runtime error status. One deliberately NaN
noise sample is rejected with a zero payload and the generated GALEC comparison
correctly signals NAN (status bit 4). No unexpected runtime errors occur.

qualification.json and synthetic-result.json retain the actual source/artifact
hashes and outputs. The three existing tangent, Jacobian and visual-correction
qualifications also passed in that fresh run. Python in that qualification
provides an independent test oracle, not the camera used by the Modelica model.

Retained development failures: a flattened row/column write exceeded Rumoca's
index-proof support, and using div/mod inside the loop triggered ED020. The
readable single-loop row/column counter implementation exports successfully.
The first checker also incorrectly required status zero for the NaN negative
case; the generated comparison contract explicitly signals NAN and evaluates
false. The corrected checker requires status 4 for that case and zero for every
other case; its geometry and visibility criteria were not relaxed.

This qualifies the abstract camera, not live CV, retained-map uncertainty,
occlusion inference, data association or the full SLAM graph. Real CV can supply
a frozen initial cloud in a declared frame. Its use as simulated scene geometry
does not certify that cloud as independent real-world ground truth. See
docs/synthetic-landmark-camera.txt for the complete interface and map contract.
