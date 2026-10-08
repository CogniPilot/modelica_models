within Control;

// SPDX-License-Identifier: Apache-2.0

block PidController "Always-running sampled PID with saturation anti-windup"
  parameter PidParameters params = PidParameters();
  final parameter PidCoefficients coefficients = PidCoefficients(
    derivativeWeight = if params.derivativeCutoffHz > 0.0 then
      1.0 - exp(-6.283185307179586
        * params.derivativeCutoffHz * params.samplePeriod)
      else 1.0,
    trackingCoefficient =
      params.samplePeriod / max(params.trackingTime, 1.0e-9));

  input Real setpoint;
  input Real measurement;

  discrete output Real error(start=0.0);
  discrete output Real derivative(start=0.0);
  discrete output Real integral(start=0.0);
  discrete output Real command(start=0.0);
  discrete output Real preview(start=0.0) "Integral-free command";
  output Boolean saturated
    "Current command is limited; this is an equation-derived predicate, not a mode";

protected
  discrete Real unconstrainedCommand(start=0.0);
  PidState nextState;
  PidResult result;

equation
  saturated = unconstrainedCommand < params.commandMin
    or unconstrainedCommand > params.commandMax;

algorithm
  when sample(0.0, params.samplePeriod) then
    (nextState, result) := pidTransition(
      params,
      coefficients,
      PidState(
        pre(integral),
        pre(error),
        pre(derivative)),
      setpoint,
      measurement,
      pre(command));
    integral := nextState.integralContribution;
    error := nextState.error;
    derivative := nextState.derivative;
    unconstrainedCommand := result.unconstrainedCommand;
    command := result.command;
    preview := result.preview;
  end when;
  annotation(Documentation(info = "<html>
    <h4>Use</h4>
<p>Supply setpoint and measurement in the same units. Configure gains, sampling
period, command bounds, integral limit and derivative cutoff through
<a href=\"modelica://Control.PidParameters\">PidParameters</a>.</p>
<pre>Control.PidController controller(params=Control.PidParameters(
  samplePeriod=0.01, kp=2.0, ki=0.5, kd=0.0,
  commandMin=-1.0, commandMax=1.0));</pre>
<p>The block updates on its own sample clock. <code>command</code> is limited;
<code>preview</code> is the integral-free command, not a second actuator output.
<code>saturated</code> reports clipping. For explicit scheduling or replay, use
<a href=\"modelica://Control.pidTransition\">pidTransition</a> and carry
<a href=\"modelica://Control.PidState\">PidState</a> between calls.</p>
    </html>"));
end PidController;
