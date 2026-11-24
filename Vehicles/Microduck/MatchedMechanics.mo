within Vehicles.Microduck;
package MatchedMechanics "Opt-in mechanical constraints for reference-model qualification"
  model RigidServoJoint "Zero relative pose; carries the complete rigid-frame wrench"
    extends MultiBodyLie.Joints.PartialServoJoint;
    MultiBodyLie.Parts.FixedTranslation fixedPose(r=zeros(3),animation=false);
  initial equation
    assert(armature==0,"Rigid servo replacement has no reflected rotor state");
  equation
    connect(frame_a,fixedPose.frame_a);
    connect(fixedPose.frame_b,frame_b);
    phi=0;
    w=0;
    a=0;
    axis.phi=0;
    support.phi=0;
    tau=axis.tau;
    annotation(Documentation(info="<html><p>Interface-compatible replacement for the servo's revolute joint. The two spatial frames are rigidly connected at zero relative pose; their full force and torque reaction goes through FixedTranslation. There is no independent jaw coordinate, angular velocity or reflected rotor inertia. The rotational flanges expose the fixed coordinate for the unchanged servo wrapper; they do not replace the rigid spatial wrench path.</p><p>The caller must set armature to zero. A zero-target revolute servo is not this constraint. This class does not implement contact, control or integration policy.</p></html>"));
  end RigidServoJoint;
  function weldedSpatialInertia "Fold a fixed child into its parent without a joint coordinate"
    input Real parentInertia[6,6];
    input Real childInertia[6,6];
    input Real childFromParent[3,3] "Fixed parent-to-child rotation";
    input Real childOffset[3](each unit = "m") "Child origin in the parent frame";
    output Real combined[6,6];
  protected
    Real transform[6,6];
  algorithm
    transform := MultiBodyLie.Math.motionTransform(childFromParent,childOffset);
    combined := parentInertia + transpose(transform)*childInertia*transform;
    annotation(Documentation(info="<html><p>This is rigid-body inertia addition at a fixed attachment: X maps the parent's spatial velocity to the child's. It introduces no velocity coordinate, armature, solver or integration choice. The caller must have an actual fixed relative pose; a zero-target hinge does not satisfy this condition.</p></html>"));
  end weldedSpatialInertia;
end MatchedMechanics;
