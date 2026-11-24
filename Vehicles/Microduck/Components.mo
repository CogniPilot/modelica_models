within Vehicles.Microduck;
package Components "Editable component defaults for the Microduck assembly"
  import MB = MultiBodyLie;
  model Trunk "Rigid body of the trunk, the root of the tree: mass, centre of mass and inertia"
    extends MB.Parts.Body(
      m=0.199224*1^3,
      r_CM={-0.0226332,-2.70828e-5,0.00280064}*1,
      I_11=0.000123975*1^5,
      I_22=0.000145931*1^5,
      I_33=0.000115351*1^5,
      I_21=7.1651e-08*1^5,
      I_31=-2.06915e-05*1^5,
      I_32=1.32429e-07*1^5,
      animation=false,
      angles_fixed=true
    );
  end Trunk;
  model LeftHipYawLink "Rigid body of the left hip yaw link: mass, centre of mass and inertia"
    extends MB.Parts.Body(
      m=0.0230406*1^3,
      r_CM={-8.8e-11,0.00099414,0.0168917}*1,
      I_11=4.1964e-06*1^5,
      I_22=3.41752e-06*1^5,
      I_33=2.3265e-06*1^5,
      I_21=-0.0*1^5,
      I_31=-0.0*1^5,
      I_32=-5.0881e-08*1^5,
      animation=false
    );
  end LeftHipYawLink;
  model LeftHipRollLink "Rigid body of the left hip roll link: mass, centre of mass and inertia"
    extends MB.Parts.Body(
      m=0.00618934*1^3,
      r_CM={0.014815,1.6179e-8,-0.00829992}*1,
      I_11=8.60626e-07*1^5,
      I_22=1.15753e-06*1^5,
      I_33=6.51588e-07*1^5,
      I_21=1e-12*1^5,
      I_31=3.89882e-07*1^5,
      I_32=-1e-12*1^5,
      animation=false
    );
  end LeftHipRollLink;
  model LeftUpperLeg "Rigid body of the left upper leg: mass, centre of mass and inertia"
    parameter Real length(unit = "m")=0.042190056700720374;
    final parameter Real lengthScale=length/0.042190056700720374;
    extends MB.Parts.Body(
      m=0.0482067*lengthScale^3,
      r_CM={0.006766,0.0228838,0.0134531}*lengthScale,
      I_11=1.68753e-05*lengthScale^5,
      I_22=1.03691e-05*lengthScale^5,
      I_33=1.98269e-05*lengthScale^5,
      I_21=-4.41852e-06*lengthScale^5,
      I_31=2.27982e-07*lengthScale^5,
      I_32=3.89298e-07*lengthScale^5,
      animation=false
    );
  end LeftUpperLeg;
  model LeftLowerLeg "Rigid body of the left lower leg: mass, centre of mass and inertia"
    parameter Real length(unit = "m")=0.04939635614091388;
    final parameter Real lengthScale=length/0.04939635614091388;
    extends MB.Parts.Body(
      m=0.0215844*lengthScale^3,
      r_CM={-1.52e-10,0.0318937,-0.00953505}*lengthScale,
      I_11=5.25056e-06*lengthScale^5,
      I_22=2.15507e-06*lengthScale^5,
      I_33=4.33925e-06*lengthScale^5,
      I_21=-1e-12*lengthScale^5,
      I_31=0.0*lengthScale^5,
      I_32=7.65005e-07*lengthScale^5,
      animation=false
    );
  end LeftLowerLeg;
  model LeftFoot "Rigid body of the left foot: mass, centre of mass and inertia"
    extends MB.Parts.Body(
      m=0.0300246*1^3,
      r_CM={-0.00615742,-0.0137483,-0.0156585}*1,
      I_11=5.91525e-06*1^5,
      I_22=1.11534e-05*1^5,
      I_33=7.74016e-06*1^5,
      I_21=-2.05354e-07*1^5,
      I_31=8.218e-08*1^5,
      I_32=5.6722e-08*1^5,
      animation=false
    );
  end LeftFoot;
  model NeckLink "Rigid body of the neck link: mass, centre of mass and inertia"
    extends MB.Parts.Body(
      m=0.0368414*1^3,
      r_CM={-0.0,-0.025,0.0141665}*1,
      I_11=1.72925e-05*1^5,
      I_22=3.12694e-06*1^5,
      I_33=1.63707e-05*1^5,
      I_21=-0.0*1^5,
      I_31=0.0*1^5,
      I_32=-0.0*1^5,
      animation=false
    );
  end NeckLink;
  model HeadPitchLink "Rigid body of the head pitch link: mass, centre of mass and inertia"
    extends MB.Parts.Body(
      m=0.00572*1^3,
      r_CM={-6.533e-9,0.0116641,-0.0145}*1,
      I_11=1.09186e-06*1^5,
      I_22=9.66855e-07*1^5,
      I_33=5.12111e-07*1^5,
      I_21=0.0*1^5,
      I_31=0.0*1^5,
      I_32=0.0*1^5,
      animation=false
    );
  end HeadPitchLink;
  model HeadYawLink "Rigid body of the head yaw link: mass, centre of mass and inertia"
    extends MB.Parts.Body(
      m=0.0486*1^3,
      r_CM={-0.000166657,-0.00500964,0.0178065}*1,
      I_11=6.97903e-06*1^5,
      I_22=8.10293e-06*1^5,
      I_33=9.57884e-06*1^5,
      I_21=3.21657e-07*1^5,
      I_31=-3.4797e-08*1^5,
      I_32=3.4295e-07*1^5,
      animation=false
    );
  end HeadYawLink;
  model Head "Rigid body of the head: mass, centre of mass and inertia"
    parameter Real jawMass(unit = "kg")=0.005;
    parameter Real jawLength(unit = "m")=0.03;
    parameter Real lumpedInertia[6](each unit = "kg.m2")={0.000320812,0.0002541,0.000149882,-6.32289e-7,6.57154e-6,4.18475e-6}
      "Imported head/jaw COM inertia: xx, yy, zz, xy, xz, yz";
    final parameter Real jawToHead[3,3]=LieGroups.SO3.Quat.to_DCM(LieGroups.SO3.Quat.normalize({0.704015,-0.0,0.710185,0.0}));
    final parameter Real jawCentreInHead[3](each unit = "m")={-0.00809334,-0.0,-0.0777383}-jawToHead*{jawLength/2,0,0};
    final parameter Real jawInertia[3,3](each unit = "kg.m2")={{jawMass*(0.025^2+0.012^2)/12,0,0},{0,jawMass*(jawLength^2+0.012^2)/12,0},{0,0,jawMass*(jawLength^2+0.025^2)/12}};
    final parameter Vehicles.Microduck.Mechanics.MassProperties properties=Vehicles.Microduck.Mechanics.splitHead(0.188766,{0.00460523,0.00120047,-0.0324761},lumpedInertia,jawMass,jawCentreInHead,jawToHead,jawInertia);
    extends MB.Parts.Body(
      m=properties.m,
      r_CM=properties.c,
      I_11=properties.I[1,1],
      I_22=properties.I[2,2],
      I_33=properties.I[3,3],
      I_21=properties.I[1,2],
      I_31=properties.I[1,3],
      I_32=properties.I[2,3],
      animation=false
    );
  end Head;
  model RightHipYawLink "Rigid body of the right hip yaw link: mass, centre of mass and inertia"
    extends MB.Parts.Body(
      m=0.0230406*1^3,
      r_CM={-8.8e-11,0.00099414,0.0168917}*1,
      I_11=4.1964e-06*1^5,
      I_22=3.41752e-06*1^5,
      I_33=2.3265e-06*1^5,
      I_21=-0.0*1^5,
      I_31=-0.0*1^5,
      I_32=-5.0881e-08*1^5,
      animation=false
    );
  end RightHipYawLink;
  model RightHipRollLink "Rigid body of the right hip roll link: mass, centre of mass and inertia"
    extends MB.Parts.Body(
      m=0.00618934*1^3,
      r_CM={0.014815,1.6212e-8,-0.00829992}*1,
      I_11=8.60626e-07*1^5,
      I_22=1.15753e-06*1^5,
      I_33=6.51588e-07*1^5,
      I_21=1e-12*1^5,
      I_31=3.89882e-07*1^5,
      I_32=-1e-12*1^5,
      animation=false
    );
  end RightHipRollLink;
  model RightUpperLeg "Rigid body of the right upper leg: mass, centre of mass and inertia"
    parameter Real length(unit = "m")=0.042190056700720374;
    final parameter Real lengthScale=length/0.042190056700720374;
    extends MB.Parts.Body(
      m=0.0482067*lengthScale^3,
      r_CM={-0.006766,0.0228838,0.0134531}*lengthScale,
      I_11=1.68753e-05*lengthScale^5,
      I_22=1.03691e-05*lengthScale^5,
      I_33=1.98269e-05*lengthScale^5,
      I_21=4.41852e-06*lengthScale^5,
      I_31=-2.27982e-07*lengthScale^5,
      I_32=3.89298e-07*lengthScale^5,
      animation=false
    );
  end RightUpperLeg;
  model RightLowerLeg "Rigid body of the right lower leg: mass, centre of mass and inertia"
    parameter Real length(unit = "m")=0.04939635614091388;
    final parameter Real lengthScale=length/0.04939635614091388;
    extends MB.Parts.Body(
      m=0.0215844*lengthScale^3,
      r_CM={-0.0318937,-1.52e-10,-0.00953505}*lengthScale,
      I_11=2.15507e-06*lengthScale^5,
      I_22=5.25056e-06*lengthScale^5,
      I_33=4.33925e-06*lengthScale^5,
      I_21=1e-12*lengthScale^5,
      I_31=-7.65005e-07*lengthScale^5,
      I_32=0.0*lengthScale^5,
      animation=false
    );
  end RightLowerLeg;
  model RightFoot "Rigid body of the right foot: mass, centre of mass and inertia"
    extends MB.Parts.Body(
      m=0.0300251*1^3,
      r_CM={0.00615539,-0.0137482,-0.0156566}*1,
      I_11=5.91534e-06*1^5,
      I_22=1.11534e-05*1^5,
      I_33=7.74015e-06*1^5,
      I_21=2.04877e-07*1^5,
      I_31=-8.2467e-08*1^5,
      I_32=5.7017e-08*1^5,
      animation=false
    );
  end RightFoot;
  model LeftHipYawMount "Fixed pose of the left hip yaw frame in its parent body"
    parameter Real hipWidth(unit = "m")=0.035;
    extends MB.Parts.FixedRotation(
      r={0.006,hipWidth/2,-0.005},
      rotationType=MB.Types.RotationTypes.TwoAxesVectors,
      n_x={0.0,0.9999999999999998,0.0},
      n_y={0.9999999999999998,0.0,0.0},
      animation=false
    );
  end LeftHipYawMount;
  model LeftHipRollMount "Fixed pose of the left hip roll frame in its parent body"
    extends MB.Parts.FixedRotation(
      r={0.0,0.0165,0.0125},
      rotationType=MB.Types.RotationTypes.TwoAxesVectors,
      n_x={0.9999999999999998,-0.0,0.0},
      n_y={0.0,0.0,-0.9999999999999998},
      animation=false
    );
  end LeftHipRollMount;
  model LeftHipPitchMount "Fixed pose of the left hip pitch frame in its parent body"
    extends MB.Parts.FixedRotation(
      r={0.025,0.0,-0.0185},
      rotationType=MB.Types.RotationTypes.TwoAxesVectors,
      n_x={0.0,-1.0,0.0},
      n_y={0.0,0.0,-1.0},
      animation=false
    );
  end LeftHipPitchMount;
  model LeftKneeMount "Fixed pose of the left knee frame in its parent body"
    parameter Real upperLegLength(unit = "m")=0.042190056700720374;
    extends MB.Parts.FixedRotation(
      r={0.022,0.0357771,-0.004}*(upperLegLength/0.042190056700720374),
      rotationType=MB.Types.RotationTypes.TwoAxesVectors,
      n_x={0.0,0.9999999999999998,0.0},
      n_y={0.9999999999999998,0.0,0.0},
      animation=false
    );
  end LeftKneeMount;
  model LeftAnkleMount "Fixed pose of the left ankle frame in its parent body"
    parameter Real lowerLegLength(unit = "m")=0.04939635614091388;
    extends MB.Parts.FixedRotation(
      r={-0.0,0.042,-0.026}*(lowerLegLength/0.04939635614091388),
      rotationType=MB.Types.RotationTypes.TwoAxesVectors,
      n_x={1.0,0.0,0.0},
      n_y={0.0,-1.0,-0.0},
      animation=false
    );
  end LeftAnkleMount;
  model NeckPitchMount "Fixed pose of the neck pitch frame in its parent body"
    extends MB.Parts.FixedRotation(
      r={0.026,0.0145,0.0324215},
      rotationType=MB.Types.RotationTypes.TwoAxesVectors,
      n_x={-0.9999999999999998,0.0,-0.0},
      n_y={0.0,0.0,-0.9999999999999998},
      animation=false
    );
  end NeckPitchMount;
  model HeadPitchMount "Fixed pose of the head pitch frame in its parent body"
    extends MB.Parts.FixedRotation(
      r={-0.0,-0.05,0.0},
      rotationType=MB.Types.RotationTypes.TwoAxesVectors,
      n_x={1.0,-0.0,0.0},
      n_y={0.0,-1.0,0.0},
      animation=false
    );
  end HeadPitchMount;
  model HeadYawMount "Fixed pose of the head yaw frame in its parent body"
    extends MB.Parts.FixedRotation(
      r={-0.0,0.0186931,-0.0145},
      rotationType=MB.Types.RotationTypes.TwoAxesVectors,
      n_x={-0.9999999999999998,0.0,0.0},
      n_y={0.0,0.0,0.9999999999999998},
      animation=false
    );
  end HeadYawMount;
  model HeadRollMount "Fixed pose of the head roll frame in its parent body"
    extends MB.Parts.FixedRotation(
      r={-0.0179,0.0,0.0145},
      rotationType=MB.Types.RotationTypes.TwoAxesVectors,
      n_x={0.0,-0.0,0.9999999999999998},
      n_y={0.0,0.9999999999999998,0.0},
      animation=false
    );
  end HeadRollMount;
  model RightHipYawMount "Fixed pose of the right hip yaw frame in its parent body"
    parameter Real hipWidth(unit = "m")=0.035;
    extends MB.Parts.FixedRotation(
      r={0.006,-hipWidth/2,-0.005},
      rotationType=MB.Types.RotationTypes.TwoAxesVectors,
      n_x={0.0,0.9999999999999998,0.0},
      n_y={0.9999999999999998,0.0,-0.0},
      animation=false
    );
  end RightHipYawMount;
  model RightHipRollMount "Fixed pose of the right hip roll frame in its parent body"
    extends MB.Parts.FixedRotation(
      r={-0.0,0.0165,0.0125},
      rotationType=MB.Types.RotationTypes.TwoAxesVectors,
      n_x={-0.9999999999999998,0.0,0.0},
      n_y={0.0,0.0,0.9999999999999998},
      animation=false
    );
  end RightHipRollMount;
  model RightHipPitchMount "Fixed pose of the right hip pitch frame in its parent body"
    extends MB.Parts.FixedRotation(
      r={0.025,0.0,-0.0185},
      rotationType=MB.Types.RotationTypes.TwoAxesVectors,
      n_x={0.0,-1.0,0.0},
      n_y={0.0,0.0,-1.0},
      animation=false
    );
  end RightHipPitchMount;
  model RightKneeMount "Fixed pose of the right knee frame in its parent body"
    parameter Real upperLegLength(unit = "m")=0.042190056700720374;
    extends MB.Parts.FixedRotation(
      r={-0.022,0.0357771,-0.004}*(upperLegLength/0.042190056700720374),
      rotationType=MB.Types.RotationTypes.TwoAxesVectors,
      n_x={1.0,-0.0,0.0},
      n_y={0.0,-1.0,0.0},
      animation=false
    );
  end RightKneeMount;
  model RightAnkleMount "Fixed pose of the right ankle frame in its parent body"
    parameter Real lowerLegLength(unit = "m")=0.04939635614091388;
    extends MB.Parts.FixedRotation(
      r={-0.042,0.0,-0.026}*(lowerLegLength/0.04939635614091388),
      rotationType=MB.Types.RotationTypes.TwoAxesVectors,
      n_x={0.0,0.9999999999999998,0.0},
      n_y={0.9999999999999998,0.0,-0.0},
      animation=false
    );
  end RightAnkleMount;
  model RevoluteServo "Position servo on a revolute joint: clipped target, proportional torque with a limit, damping, friction and soft end stops"
    parameter Real n[3]={0,0,1};
    parameter Real minimum(unit = "rad")=-1.5707963267948966;
    parameter Real maximum(unit = "rad")=1.5707963267948966;
    parameter Real home(unit = "rad")=0;
    parameter Real commandCentre(unit = "rad")=home;
    parameter Real armature(unit = "kg.m2")=0.0018;
    parameter Real kp(unit = "N.m/rad")=0.55;
    parameter Real damping(unit = "N.m.s/rad")=0.053;
    parameter Real friction(unit = "N.m")=0.0048;
    parameter Real torqueLimit(unit = "N.m")=0.96;
    input Real action;
    input Boolean usePositionTarget=false;
    input Real positionTarget(unit = "rad")=home;
    MB.Interfaces.Frame_a frame_a;
    MB.Interfaces.Frame_b frame_b;
    output Real phi(unit = "rad");
    output Real w(unit = "rad/s");
    output Real target(unit = "rad");
    output Real torque(unit = "N.m");
    replaceable MB.Joints.Revolute joint(n=n,useAxisFlange=true,armature=armature,phi(start=home,fixed=true),w(start=0,fixed=true)) constrainedby MB.Joints.PartialServoJoint;
    Vehicles.Microduck.Mechanics.Drive rotor;
  equation
    connect(frame_a,joint.frame_a);
    connect(joint.frame_b,frame_b);
    connect(rotor.flange,joint.axis);
    phi=joint.phi;
    w=joint.w;
    target=max(minimum,min(maximum,if usePositionTarget then positionTarget else commandCentre+0.5*(maximum-minimum)*max(-1,min(1,action))));
    torque=max(-torqueLimit,min(torqueLimit,kp*(target-phi)));
    rotor.command=torque-damping*w-friction*tanh(w/0.01)
      +20*max(0,minimum-phi)-20*max(0,phi-maximum);
  end RevoluteServo;
  model LeftHipYawServo "Servo of the left hip yaw joint: axis, angle limits and home angle"
    extends RevoluteServo(
      n={0.0,0.0,1.0},
      minimum=-0.4363323129985824,
      maximum=0.5235987755982988,
      home=0
    );
  end LeftHipYawServo;
  model LeftHipRollServo "Servo of the left hip roll joint: axis, angle limits and home angle"
    extends RevoluteServo(
      n={0.0,0.0,1.0},
      minimum=-0.3839724354386992,
      maximum=0.38397243543880577,
      home=-0.0873
    );
  end LeftHipRollServo;
  model LeftHipPitchServo "Servo of the left hip pitch joint: axis, angle limits and home angle"
    extends RevoluteServo(
      n={0.0,0.0,1.0},
      minimum=-1.570796326794949,
      maximum=1.5707963267948442,
      home=-0.4579
    );
  end LeftHipPitchServo;
  model LeftKneeServo "Servo of the left knee joint: axis, angle limits and home angle"
    extends RevoluteServo(
      n={0.0,0.0,1.0},
      minimum=-1.5707963267948983,
      maximum=1.5707963267948948,
      home=-0.0049
    );
  end LeftKneeServo;
  model LeftAnkleServo "Servo of the left ankle joint: axis, angle limits and home angle"
    extends RevoluteServo(
      n={0.0,0.0,1.0},
      minimum=-1.5707963267949063,
      maximum=1.5707963267948868,
      home=0.453
    );
  end LeftAnkleServo;
  model NeckPitchServo "Servo of the neck pitch joint: axis, angle limits and home angle"
    extends RevoluteServo(
      n={0.0,0.0,1.0},
      minimum=-1.5707963267948974,
      maximum=1.0471975511965967,
      home=0.3491
    );
  end NeckPitchServo;
  model HeadPitchServo "Servo of the head pitch joint: axis, angle limits and home angle"
    extends RevoluteServo(
      n={0.0,0.0,1.0},
      minimum=-1.5707963267948966,
      maximum=1.5707963267948966,
      home=0.3491
    );
  end HeadPitchServo;
  model HeadYawServo "Servo of the head yaw joint: axis, angle limits and home angle"
    extends RevoluteServo(
      n={0.0,0.0,1.0},
      minimum=-2.9670597283903613,
      maximum=2.9670597283903595,
      home=0
    );
  end HeadYawServo;
  model HeadRollServo "Servo of the head roll joint: axis, angle limits and home angle"
    extends RevoluteServo(
      n={0.0,0.0,1.0},
      minimum=-0.43633231299858327,
      maximum=0.4363323129985815,
      home=0
    );
  end HeadRollServo;
  model RightHipYawServo "Servo of the right hip yaw joint: axis, angle limits and home angle"
    extends RevoluteServo(
      n={0.0,0.0,1.0},
      minimum=-0.523598775598297,
      maximum=0.43633231299858416,
      home=0
    );
  end RightHipYawServo;
  model RightHipRollServo "Servo of the right hip roll joint: axis, angle limits and home angle"
    extends RevoluteServo(
      n={0.0,0.0,1.0},
      minimum=-0.3839724354387507,
      maximum=0.38397243543875426,
      home=0.0873
    );
  end RightHipRollServo;
  model RightHipPitchServo "Servo of the right hip pitch joint: axis, angle limits and home angle"
    extends RevoluteServo(
      n={0.0,0.0,1.0},
      minimum=-1.57079632679494,
      maximum=1.570796326794853,
      home=0.4579
    );
  end RightHipPitchServo;
  model RightKneeServo "Servo of the right knee joint: axis, angle limits and home angle"
    extends RevoluteServo(
      n={0.0,0.0,1.0},
      minimum=-1.5707963267949339,
      maximum=1.5707963267948593,
      home=0.0049
    );
  end RightKneeServo;
  model RightAnkleServo "Servo of the right ankle joint: axis, angle limits and home angle"
    extends RevoluteServo(
      n={0.0,0.0,1.0},
      minimum=-1.5707963267949028,
      maximum=1.5707963267948903,
      home=-0.453
    );
  end RightAnkleServo;
  model JawServo "Servo of the jaw joint: axis, angle limits and home angle"
    extends RevoluteServo(
      n={0,1,0},
      minimum=0,
      maximum=0.6108652381980153,
      home=0,
      commandCentre=0.30543261909900765,
      friction=0
    );
  end JawServo;
  model Jaw "Rigid body of the jaw as a uniform box of the given length"
    parameter Real length(unit = "m")=0.03;
    extends MB.Parts.Body(m=0.005,r_CM={length/2,0,0},I_11=m*(0.025^2+0.012^2)/12,
      I_22=m*(length^2+0.012^2)/12,I_33=m*(length^2+0.025^2)/12,animation=false);
  end Jaw;
  model TrunkImuMount "Fixed pose of the trunk IMU frame in its parent body"
    extends MB.Parts.FixedRotation(
      r={-0.021,6.64098e-5,-0.0146984},
      rotationType=MB.Types.RotationTypes.TwoAxesVectors,
      n_x={1.0,0.0,0.0},
      n_y={0.0,1.0,0.0},
      animation=false
    );
  end TrunkImuMount;
  model LeftSoleMount "Fixed pose of the left sole frame in its parent body"
    extends MB.Parts.FixedRotation(
      r={-0.0,-0.0238146,-0.0140852},
      rotationType=MB.Types.RotationTypes.TwoAxesVectors,
      n_x={-0.9999999999999998,0.0,0.0},
      n_y={0.0,0.0,0.9999999999999998},
      animation=false
    );
  end LeftSoleMount;
  model CameraMount "Fixed pose of the camera frame in its parent body"
    extends MB.Parts.FixedRotation(
      r={0.0155,-9.13778e-5,-0.0733},
      rotationType=MB.Types.RotationTypes.TwoAxesVectors,
      n_x={0.0,0.0,-0.9999999999999998},
      n_y={-0.0,0.9999999999999998,0.0},
      animation=false
    );
  end CameraMount;
  model TofMount "Fixed pose of the time-of-flight sensor frame in its parent body"
    extends MB.Parts.FixedRotation(
      r={0.0135,0.0224086,-0.0733},
      rotationType=MB.Types.RotationTypes.TwoAxesVectors,
      n_x={0.0,0.0,-0.9999999999999998},
      n_y={-0.0,0.9999999999999998,0.0},
      animation=false
    );
  end TofMount;
  model HeadImuMount "Fixed pose of the head IMU frame in its parent body"
    extends MB.Parts.FixedRotation(
      r={0.0152323,0.000111069,-0.05106},
      rotationType=MB.Types.RotationTypes.TwoAxesVectors,
      n_x={1.0,0.0,0.0},
      n_y={-0.0,1.0,0.0},
      animation=false
    );
  end HeadImuMount;
  model JawMount "Fixed pose of the jaw frame in its parent body"
    parameter Real jawLength(unit = "m")=0.03;
    final parameter Real childToHead[3,3]=LieGroups.SO3.Quat.to_DCM(LieGroups.SO3.Quat.normalize({0.704015,-0.0,0.710185,0.0}));
    extends MB.Parts.FixedRotation(
      r={-0.00809334,-0.0,-0.0777383}-childToHead*{jawLength,0,0},
      rotationType=MB.Types.RotationTypes.TwoAxesVectors,
      n_x={-0.00872561527001331,0.0,-0.9999619310944591},
      n_y={-0.0,1.0,0.0},
      animation=false
    );
  end JawMount;
  model RightSoleMount "Fixed pose of the right sole frame in its parent body"
    extends MB.Parts.FixedRotation(
      r={0.0,-0.0238146,-0.0140852},
      rotationType=MB.Types.RotationTypes.TwoAxesVectors,
      n_x={0.9999999999999998,0.0,0.0},
      n_y={0.0,0.0,-0.9999999999999998},
      animation=false
    );
  end RightSoleMount;
  model SoleContact "Four ground-contact spheres at the corners of a rectangular sole"
    parameter Real footLength(unit = "m")=0.07;
    parameter Real footWidth(unit = "m")=0.04;
    extends MultiBodyLie.Contact.GroundContactBody(n=4,
      centre={{-footLength/2,-footWidth/2,0},{-footLength/2,footWidth/2,0},{footLength/2,-footWidth/2,0},{footLength/2,footWidth/2,0}},
      radius=fill(0.003,4),k=10000,mu=0.8);
  end SoleContact;
  model BodyContact "One ground-contact sphere at a point of a body"
    parameter Real point[3](each unit = "m")=zeros(3);
    parameter Real radiusValue(unit = "m")=0.012;
    extends MultiBodyLie.Contact.GroundContactBody(
      n=1,
      centre={point},
      radius={radiusValue},
      k=10000,
      mu=0.8
    );
  end BodyContact;
end Components;
