within Vision.Features;
model FeatureSelection
  import SelectRasterFeatures = Vision.Features.SelectRasterFeatures;

  parameter Integer width = 160;
  parameter Integer height = 90;
  // Storage covers the full editable feature limit.
  parameter Integer capacity = 14400;
  // Structural loop bound; presets may modify it before compilation.
  parameter Integer minimumBorder = 0 annotation(Evaluate = true);
  parameter Boolean grid = false;
  input Boolean enabled = true;
  input Real scores[width*height] = zeros(width*height);
  input Real settings[8] = {1e-9,0.01,1e12,3.0,240.0,1.0,0.0,0.0};
  output Real features[capacity,3];
  output Real count;
  output Real valid;
equation
  (features,count,valid) = SelectRasterFeatures(scores,width,height,capacity,minimumBorder,settings,grid,enabled);
end FeatureSelection;
