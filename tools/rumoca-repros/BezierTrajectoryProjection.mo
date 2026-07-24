within;
package BezierTrajectoryProjection
  model Example
    Real startDerivative[3, 4];
    Real endDerivative[3, 4];
    Real positionControlPoint[3, 8];
    Real yawControlPoint[1, 4];
    Planning.Bezier.MultirotorTrajectory trajectory;
    output Real position[3];
  equation
    startDerivative = [
      0.0, 0.0, 0.0, 0.0;
      0.0, 0.0, 0.0, 0.0;
      2.0, 0.0, 0.0, 0.0];
    endDerivative = [
      4.0 + 1.0e-3 * time, 0.0, 0.0, 0.0;
      0.0, 0.0, 0.0, 0.0;
      2.0, 0.0, 0.0, 0.0];
    positionControlPoint = Planning.Bezier.septicControlPoints(
      startDerivative, endDerivative, 7.0);
    yawControlPoint = fill(0.0, 1, 4);
    trajectory = Planning.Bezier.evaluateMultirotor(
      positionControlPoint, yawControlPoint, 7.0, time);
    position = trajectory.position;
  end Example;
end BezierTrajectoryProjection;
