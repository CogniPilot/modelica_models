within Estimation.StrapdownINS.ESKF;

function constrainGain
  "Project considered states and limit the finite attitude correction"
  input Real gain[TangentLength, :];
  input Real residual[size(gain, 2)];
  input Real attitudeAxis[3];
  input Boolean headingOnly;
  output Real constrainedGain[TangentLength, size(gain, 2)];
  output TangentVector correction;
protected
  Real projector[3, 3];
  Real projectedGain[TangentLength, size(gain, 2)];
  Real attitudeAngle;
  Real trustScale;
algorithm
  projector := considerGainProjector(attitudeAxis);
  if headingOnly then
    projectedGain := cat(1, zeros(6, size(gain, 2)),
      projector * gain[7:9, :], projector * gain[10:12, :],
      zeros(3, size(gain, 2)));
  else
    projectedGain := cat(1, gain[1:6, :], projector * gain[7:9, :],
      gain[10:TangentLength, :]);
  end if;
  correction := projectedGain * residual;
  attitudeAngle := sqrt(correction[7:9] * correction[7:9]);
  trustScale := if attitudeAngle > MaxAttitudeCorrection_rad
    then MaxAttitudeCorrection_rad / attitudeAngle else 1.0;
  constrainedGain := cat(1, projectedGain[1:6, :],
    trustScale * projectedGain[7:9, :], projectedGain[10:TangentLength, :]);
  correction := constrainedGain * residual;
  annotation(Inline=false);
end constrainGain;
