within Vision.Features;
model D435FastFeatures
  import D435ImageProfile = Vision.Sensors.D435ImageProfile;
  import FastNativeFrame = Vision.Features.FastNativeFrame;

  extends FastNativeFrame(height=D435ImageProfile.height,width=D435ImageProfile.width,
    channels=D435ImageProfile.colorChannels);
end D435FastFeatures;
