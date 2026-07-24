within;
package FlatReferenceProjection
  model Example
    Planning.Bezier.MultirotorTrajectory trajectory;
    Planning.Bezier.FlatReference reference;
    output Real thrust;
  equation
    trajectory.position = {0.0, 0.0, 2.0};
    trajectory.velocity = {0.5, 0.0, 0.0};
    trajectory.acceleration = {0.25, 0.0, 0.0};
    trajectory.jerk = {0.0, 0.0, 0.0};
    trajectory.snap = {0.0, 0.0, 0.0};
    trajectory.yaw = 0.0;
    trajectory.yawRate = 0.0;
    trajectory.yawAcceleration = 0.0;
    reference = Planning.Bezier.flatReference(
      trajectory,
      2.0,
      9.80665,
      diagonal({0.02166666666666667, 0.02166666666666667, 0.04}));
    thrust = reference.thrust;
  end Example;
end FlatReferenceProjection;
