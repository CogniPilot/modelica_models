within Vehicles.Interfaces;
function centeredPwmToUnit "Map centered PWM microseconds to [-1, 1]"
  input Real pwm_us;
  output Real command;
algorithm
  command := MathUtilities.clip((pwm_us - 1500.0) / 500.0, -1.0, 1.0);
  annotation(Inline = true);
end centeredPwmToUnit;
