within Estimation.MultiSensorInvariant;

function continuousTransition
  "Continuous left-invariant error dynamics for held IMU input"
  input Real angularVelocityBodyFlu_rad_s[3];
  input Real specificForceBodyFlu_m_s2[3];
  output Real A[TangentLength, TangentLength];
protected
  Real omegaCross[3, 3];
  Real forceCross[3, 3];
algorithm
  omegaCross := LieGroups.SO3.Quat.wedge(angularVelocityBodyFlu_rad_s);
  forceCross := LieGroups.SO3.Quat.wedge(specificForceBodyFlu_m_s2);
  A := cat(1,
    cat(2, -omegaCross, identity(3), zeros(3, 9)),
    cat(2, zeros(3, 3), -omegaCross, -forceCross,
      zeros(3, 3), -identity(3)),
    cat(2, zeros(3, 6), -omegaCross, -identity(3), zeros(3, 3)),
    zeros(6, TangentLength));
end continuousTransition;
