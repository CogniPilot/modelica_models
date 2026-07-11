within RigidBody;
record StateDerivative "Derivative corresponding to RigidBody.State"
  Real worldPosition[3];
  Real bodyVelocity[3];
  Real attitude[4];
  Real bodyAngularVelocity[3];
end StateDerivative;
