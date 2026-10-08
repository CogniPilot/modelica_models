within SLAM;
model D435FastSLAMIntervals
  import D435ImageProfile = Vision.Sensors.D435ImageProfile;
  import RGBDFastSLAMIntervals = SLAM.RGBDFastSLAMIntervals;

  extends RGBDFastSLAMIntervals(imageHeight=D435ImageProfile.height,
    imageWidth=D435ImageProfile.width,channelCount=D435ImageProfile.colorChannels,
    depthUnits=D435ImageProfile.depthUnits);
end D435FastSLAMIntervals;
