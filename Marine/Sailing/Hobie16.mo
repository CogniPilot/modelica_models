within Marine.Sailing;
model Hobie16 "Automated full-size Hobie 16 with a 60 kg two-axis ballast carriage"
  parameter Real jibHeadRise(unit="m")=4.56 "Estimated cloth rise below the exposed upper forestay";
  extends Sailboat(catamaran=true,mass=205,ballastMass=60,
    halfSpacing=1.02,stationX={-2.555,-1.9,-.9,.15,1.12,1.95,2.555},
    beamHalf={.09,.156,.185,.184,.155,.093,.009},deckZ=.23,centerMassX=-.28,
    spans={7.15,jibHeadRise},feet={2.78,1.98},mainRoach=.82,forestaySlope=1.82/5.1,
    footX={.44,2.26},footZ={.97,.75},yawInertia=315,rollInertia=187,pitchInertia=292,
    foilX={.15,-2.42,.15,-2.42},foilY={1.02,1.02,-1.02,-1.02},
    foilArea={.62,.11,.62,.11},aspect={.65,3.4,.65,3.4});
end Hobie16;
