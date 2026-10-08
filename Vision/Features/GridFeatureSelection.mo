within Vision.Features;
model GridFeatureSelection
  import FeatureSelection = Vision.Features.FeatureSelection;

  extends FeatureSelection(capacity=14400,grid=true,settings={0.0,0.0,1.0,0.0,14400.0,6.0,5.0,5.0});
end GridFeatureSelection;
