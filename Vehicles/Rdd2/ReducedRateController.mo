within Vehicles.Rdd2;

// SPDX-License-Identifier: Apache-2.0

block ReducedRateController
  "RDD2 flight controller with a reduced-authority body-rate loop"
  extends Vehicles.Rdd2.AvionicsSystem(
    rateTask(rateGain = {12.0, 12.0, 6.0}));

  annotation(Documentation(info = "<html>
    <p>Second concrete implementation of
    <code>Vehicles.Rdd2.PartialController</code>. It is identical to the
    default <code>AvionicsSystem</code> stack in structure and signal routing
    and differs only in the body-rate loop gain, which is softened from the
    nominal <code>{20, 20, 10}</code> to <code>{12, 12, 6}</code>. The lower
    gain reduces rate-loop bandwidth and control authority, producing a more
    sluggish closed-loop response.</p>
    <p>Its only purpose is to prove that the controller boundary is a real swap
    point symmetric to the estimator boundary: a single
    <code>redeclare block ControllerModel = Vehicles.Rdd2.ReducedRateController</code>
    line in a mission selects it in place of the default stack, changing the
    controller and touching nothing else in the closed loop. It is not claimed
    to fly better; it exists to exercise the interface.</p>
  </html>"));
end ReducedRateController;
