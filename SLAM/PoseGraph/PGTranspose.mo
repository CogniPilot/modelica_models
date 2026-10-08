within SLAM.PoseGraph;
// Bounded pose-graph numerical component; no loop detection or truth inputs.
// Body-to-world R, world-additive p and RIGHT-local attitude increments.
// Residual [Ri'*(pj-pi)-translation, Log(measuredRotation'*Ri'*Rj)].
// This product residual is not the coupled SE(3) logarithm (no V^-1 translation).
// Small numerical helpers are editable Modelica, not host math.
function PGTranspose
  input Real A[3,3]; output Real B[3,3];
algorithm
  B[1,1] := A[1,1];
  B[2,1] := A[1,2];
  B[3,1] := A[1,3];
  B[1,2] := A[2,1];
  B[2,2] := A[2,2];
  B[3,2] := A[2,3];
  B[1,3] := A[3,1];
  B[2,3] := A[3,2];
  B[3,3] := A[3,3];
end PGTranspose;
