within MathUtilities;
function squareRootRamp
  "Correction toward zero error that a bounded derivative can still unwind"
  input Real error "Quantity to drive to zero";
  input Real proportionalGain(min = 0.0) "Gain of the linear region";
  input Real derivativeLimit(min = 0.0)
    "Bound on the rate of change of the result";
  output Real result "Correction with the sign of the error";
protected
  Real linearWidth "Half-width of the linear region";
algorithm
  if proportionalGain <= 0.0 then
    result := sign(error) * sqrt(2.0 * derivativeLimit * abs(error));
  elseif derivativeLimit <= 0.0 then
    result := proportionalGain * error;
  else
    linearWidth := derivativeLimit / proportionalGain ^ 2;
    result := if abs(error) > linearWidth then
        sign(error)
        * sqrt(2.0 * derivativeLimit * (abs(error) - 0.5 * linearWidth))
      else
        proportionalGain * error;
  end if;
  annotation(Documentation(info = "<html>
    <p>ArduPilot's <code>sqrt_controller</code>
    (<code>libraries/AP_Math/control.cpp</code>). Far from the target the
    result follows <code>sqrt(2 * derivativeLimit * error)</code>, the largest
    correction whose own unwinding at <code>derivativeLimit</code> exactly
    consumes the remaining error, so the approach is asymptotic rather than
    overshooting. Near the target it becomes the linear gain, and the two
    pieces join with matching value and slope at
    <code>derivativeLimit / proportionalGain^2</code>.</p>
    <p>A plain proportional law with the same limits overshoots badly: it holds
    full deceleration until the error is nearly gone and then has to unwind a
    saturated correction through the same rate bound, reversing well past the
    target. That failure is why every production position-hold implementation
    uses either this ramp or a full jerk-limited profile.</p>
  </html>"));
end squareRootRamp;
