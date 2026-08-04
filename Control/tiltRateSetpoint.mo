within Control;

// SPDX-License-Identifier: Apache-2.0

function tiltRateSetpoint
  "Geometric roll/pitch rate demand from the world-up direction"
  input Real desiredTilt[2] "{roll, pitch} demand [rad]";
  input Real worldUpBody[3] "World up expressed in the body frame";
  input Real gain[2] "Tilt-error gains {roll, pitch} [1/s]";
  input Real rateLimit[2] "Symmetric body-rate limits [rad/s]";
  output Real rateSetpoint[2] "{roll rate, nose-up rate} [rad/s]";

protected
  Real desiredRotation[3, 3];
  Real desiredWorldUpBody[3];
  Real tiltError[3];

algorithm
  assert(rateLimit[1] >= 0.0 and rateLimit[2] >= 0.0,
    "Tilt-rate limits must be nonnegative");
  desiredRotation := LieGroups.SO3.EulerB321.to_DCM(
    {0.0, desiredTilt[2], desiredTilt[1]});
  desiredWorldUpBody := desiredRotation[3, :];
  tiltError := cross(worldUpBody, desiredWorldUpBody);
  rateSetpoint := {
    MathUtilities.clip(-gain[1] * tiltError[1],
      -rateLimit[1], rateLimit[1]),
    MathUtilities.clip(gain[2] * tiltError[2],
      -rateLimit[2], rateLimit[2])};
end tiltRateSetpoint;
