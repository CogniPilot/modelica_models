within SLAM;
model D435FastSLAMStep
  import D435ImageProfile = Vision.Sensors.D435ImageProfile;
  import RGBDFastSLAMStep = SLAM.RGBDFastSLAMStep;

  extends RGBDFastSLAMStep(imageHeight=D435ImageProfile.height,
    imageWidth=D435ImageProfile.width,channelCount=D435ImageProfile.colorChannels,
    depthUnits=D435ImageProfile.depthUnits);
end D435FastSLAMStep;
