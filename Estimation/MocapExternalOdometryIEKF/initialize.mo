within Estimation.MocapExternalOdometryIEKF;
function initialize "Initialize flat IEKF state from a pose measurement"
  input Real position_enu_m[3];
  input Real attitude_wxyz[4];
  input Real attitudeVariance;
  input Real velocityVariance;
  input Real positionVariance;
  input Real angularVelocityVariance;
  output Real x[157]
    "Flat state: attitude, velocity, position, angular velocity, covariance";
protected
  Real attitude[4];
algorithm
  x := zeros(157);
  attitude := LieGroups.SO3.Quat.normalize(attitude_wxyz);
  for i in 1:4 loop
    x[i] := attitude[i];
  end for;
  for i in 1:3 loop
    x[i + 7] := position_enu_m[i];
    x[13 + (i - 1) * 12 + i] := attitudeVariance;
    x[13 + (i + 2) * 12 + i + 3] := velocityVariance;
    x[13 + (i + 5) * 12 + i + 6] := positionVariance;
    x[13 + (i + 8) * 12 + i + 9] := angularVelocityVariance;
  end for;
end initialize;
