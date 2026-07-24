within Vehicles.Rdd2;
model LogLinearController "RDD2 parameterization of the log-linear controller"
  extends Control.Multirotor.LogLinear.Controller(
    mass = 2.0,
    gravity = 9.8,
    thrustTrim = 19.6,
    attitudeGain = {2.0, 2.0, 1.0});
  annotation(Documentation(info="<html>
    <p>Applies the RDD2 mass, design gravity, trim thrust, and attitude gains
    to <code>Control.Multirotor.LogLinear.Controller</code>. The reusable
    geometric control algorithms remain in the <code>Control</code> library.</p>
  </html>"));
end LogLinearController;
