within Vehicles.Microduck;
model JawLocked "Opt-in rigid-jaw mechanism; actuator and controller qualification remain separate"
  extends Vehicles.Microduck.Robot(
    head(lumpedInertia={0.0003208120000563054,0.00025410001070930085,0.00014988198923439406,
      -6.322868601343577e-7,6.5715394732933165e-6,4.1846270963894155e-6}),
    jawServo(
      redeclare Vehicles.Microduck.MatchedMechanics.RigidServoJoint joint,
      home=0,commandCentre=0,armature=0,kp=0,damping=0,friction=0,torqueLimit=0));
  annotation(Documentation(info="<html><p>The jaw is rigidly attached at its existing zero-angle mount, matching the reference mechanism rather than using a zero-target servo. The original separate head and jaw bodies remain connected through the rigid joint. Their recombined mass, centre of mass and spatial inertia match the effective MuJoCo 3.11 compiled head/jaw inertia.</p><p>This configuration changes the jaw mechanism and effective head inertia only. The remaining actuator law, contact, walking policy and integrator are inherited from Microduck and have not become a complete matched-controller configuration. The ordinary Microduck model retains its original defaults.</p></html>"));
end JawLocked;
