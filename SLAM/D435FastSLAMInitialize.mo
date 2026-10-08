within SLAM;
// Native-camera entrypoints for the complete authored SLAM processing graph.
// Native RGB8/Z16 shape and depth-scale bindings: math and State are shared
// with the parameterized entrypoints. These targets still require Rumoca WASM
// issuance and end-to-end browser qualification before runtime admission.
model D435FastSLAMInitialize
  import D435ImageProfile = Vision.Sensors.D435ImageProfile;
  import RGBDFastSLAMInitialize = SLAM.RGBDFastSLAMInitialize;

  extends RGBDFastSLAMInitialize(imageHeight=D435ImageProfile.height,
    imageWidth=D435ImageProfile.width,channelCount=D435ImageProfile.colorChannels,
    depthUnits=D435ImageProfile.depthUnits);
end D435FastSLAMInitialize;
