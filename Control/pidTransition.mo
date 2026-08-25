within Control;

// SPDX-License-Identifier: Apache-2.0

function pidTransition
  "Pure fixed-dimensional PID state transition for analysis and deployment"
  input PidParameters params;
  input PidCoefficients coefficients;
  input PidState previous;
  input Real setpoint;
  input Real measurement;
  input Real previousAppliedCommand;
  output PidState next;
  output PidResult result;

protected
  Real rawDerivative;
  Real saturationCorrection;

algorithm
  next.error := setpoint - measurement;
  rawDerivative := (next.error - previous.error) / params.samplePeriod;
  next.derivative := previous.derivative
    + coefficients.derivativeWeight * (rawDerivative - previous.derivative);

  result.preview := MathUtilities.clip(
    params.kp * next.error + params.kd * next.derivative,
    params.commandMin,
    params.commandMax);
  result.unconstrainedCommand := params.kp * next.error
    + previous.integralContribution
    + params.kd * next.derivative;
  result.command := MathUtilities.clip(
    result.unconstrainedCommand,
    params.commandMin,
    params.commandMax);

  saturationCorrection := coefficients.trackingCoefficient
    * (previousAppliedCommand - result.unconstrainedCommand);
  next.integralContribution := MathUtilities.clip(
    previous.integralContribution
      + params.ki * next.error * params.samplePeriod
      + saturationCorrection,
    -params.integralLimit,
    params.integralLimit);
end pidTransition;
