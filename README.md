# modelica_models

Reusable Modelica building blocks for FastDyn/Rumoca simulations.

## Layout

- `LieGroup/`: lightweight SO(3) and quaternion utilities used by the
  generic rigid-body templates.
- `LieGroups/`: full Lie group library (SO(2)/SO(3), SE(2)/SE(3)/SE_2(3),
  with quaternion, DCM, MRP, and Euler-B321 SO(3) charts) providing exp/log
  maps, products, adjoints, and left/right Jacobians.
- `Geodesy/`: reusable local-frame and geodetic conversion helpers.
- `RigidBody/`: reusable six-degree-of-freedom rigid-body dynamics.
- `RigidBody/Examples/`: reusable base plants for quadrotor, rover, and
  fixed-wing use cases. These plants contain vehicle-generic dynamics only.

FastDyn-specific vehicle wrappers live in the FastDyn repository under
`modelica/FastDyn`. Those wrappers inherit these base plants and add
the sensor, actuator, and driver-interface equations needed by FastDyn.
