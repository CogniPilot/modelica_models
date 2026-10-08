within Vision.Features;
// Row-major RGB/RGBA input; alpha is ignored.
model FastNativeFrame
  import FastFrameScores = Vision.Features.FastFrameScores;

  parameter Integer height = 90;
  parameter Integer width = 160;
  constant Integer radius = 3;
  parameter Integer channels(min=3,max=4) = 4;
  parameter Real absolute_threshold = 18;
  parameter Real relative_threshold = 0;
  parameter Real rank_scale = 1e8;
  parameter Real suppression_radius = 3;
  parameter Real feature_cap = 240;
  input Boolean enabled = true "False on held-IMU intervals without a camera acquisition";
  input Real rgb[height,width,channels] = fill(0.0,height,width,channels);
  output Real scores[height*width];
  output Real selection[8];
equation
  scores = FastFrameScores(rgb,enabled);
  selection = {absolute_threshold,relative_threshold,rank_scale,suppression_radius,feature_cap,1,3,3};
end FastNativeFrame;
