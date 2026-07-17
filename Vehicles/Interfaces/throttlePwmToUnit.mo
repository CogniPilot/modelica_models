within Vehicles.Interfaces;
function throttlePwmToUnit "Map throttle PWM microseconds to [0, 1]"
  input Real pwm_us;
  output Real command;
algorithm
  command := MathUtilities.clip((pwm_us - 1000.0) / 1000.0, 0.0, 1.0);
  annotation(Inline = true);
end throttlePwmToUnit;
