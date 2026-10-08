within Vision.Features;
model D435HarrisFeatures
  import D435ImageProfile = Vision.Sensors.D435ImageProfile;
  import HarrisNativeFrame = Vision.Features.HarrisNativeFrame;

  extends HarrisNativeFrame(height=D435ImageProfile.height,width=D435ImageProfile.width);
end D435HarrisFeatures;
