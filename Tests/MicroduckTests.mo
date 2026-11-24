within Tests;
package MicroduckTests "Microduck assembly: start pose against the rest-tree kinematics"
  model InitialPose "The start height computed from the rest tree puts the lowest sole sphere 1 mm above the ground"
    Vehicles.Microduck.Robot robot;
    Real soleCorner[2,4,3](each unit="m") "Sole contact-sphere centres, world";
    Real lowest(unit="m") "Lowest sole-sphere bottom";
  protected
    Real halfLength=robot.footLength/2;
    Real halfWidth=robot.footWidth/2;
  equation
    for x in 1:2 loop
      for y in 1:2 loop
        soleCorner[1,2*(x-1)+y,:]=robot.leftSole.frame_b.r_0+transpose(robot.leftSole.frame_b.R.T)
          *{(2*x-3)*halfLength,(2*y-3)*halfWidth,0};
        soleCorner[2,2*(x-1)+y,:]=robot.rightSole.frame_b.r_0+transpose(robot.rightSole.frame_b.R.T)
          *{(2*x-3)*halfLength,(2*y-3)*halfWidth,0};
      end for;
    end for;
    lowest=min(soleCorner[:,:,3])-robot.leftSoleContact.radius[1];
    // Mechanics.restTree and the MultiBodyLie joints and mounts compute the same forward kinematics.
    assert(not initial() or abs(lowest-0.001)<1e-9,"Start height does not place the soles 1 mm above the ground");
  end InitialPose;
end MicroduckTests;
