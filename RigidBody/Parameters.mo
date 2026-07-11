within RigidBody;
record Parameters "Physical parameters used by the pure rigid-body vector field"
  Real mass(min=0.0) "Mass [kg]; callers must require mass > 0";
  Real gravity(min=0.0) "Gravity magnitude [m/s^2]";
  Real inertia[3, 3] "Body inertia [kg*m^2]";
  Real quaternionNormGain(min=0.0)
    "Off-manifold norm stabilization; zero gives the physical vector field";
end Parameters;
