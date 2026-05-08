# modelica_models

Reusable Modelica building blocks for FastDyn/Rumoca simulations.

## Layout

- `LieGroup/`: lightweight SO(3) and quaternion utilities used by the
  generic rigid-body templates.
- `Geodesy/`: reusable local-frame and geodetic conversion helpers.
- `RigidBody/`: reusable six-degree-of-freedom rigid-body dynamics.
- `RigidBody/Examples/`: reusable base plants for quadrotor, rover, and
  fixed-wing use cases. These plants contain vehicle-generic dynamics only.

FastDyn-specific vehicle wrappers live in the FastDyn repository under
`modelica/FastDyn`. Those wrappers inherit these base plants and add
the sensor, actuator, and driver-interface equations needed by FastDyn.
