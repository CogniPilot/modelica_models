within Vision.Features;
model FastFeatureSelection
  import FeatureSelection = Vision.Features.FeatureSelection;

  extends FeatureSelection(minimumBorder=3,settings={18.0,0.0,1e8,3.0,240.0,1.0,3.0,3.0});
end FastFeatureSelection;
