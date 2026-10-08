within Vision.Matching;
// Reserve feature slots for pixels with valid calibrated depth.
function RGBDDepthQualifiedScores
  import RGBDCalibratedPoint = Vision.Matching.RGBDCalibratedPoint;

  input Real depth[:,:];
  input Real scores[size(depth,1)*size(depth,2)];
  input Real rgbCalibration[4];
  input Real depthCalibration[4];
  input Real nearDepth;
  input Real farDepth;
  input Real disparityNoise;
  input Real noiseReferenceFx;
  input Real baseline;
  input Boolean enabled = true;
  output Real qualified[size(scores,1)];
  input Real depthUnits = 1.0 "Meters per depth sample; 1 for metric depth, SDK scale for Z16";
protected
  Real point[3];
  Boolean valid;
  Integer index;
algorithm
  qualified := zeros(size(scores,1));
  if enabled then
    qualified := scores;
    for row in 1:size(depth,1) loop
      for column in 1:size(depth,2) loop
        index := (row-1)*size(depth,2)+column;
        // Leave invalid scores untouched for the selector to refuse. Zero
        // scores need no depth work; disabled acquisitions read no depth.
        if scores[index] > 0.0 and scores[index] <= 1e8 then
          (point,valid) := RGBDCalibratedPoint(depth,{column-1.0,row-1.0},true,
            rgbCalibration,depthCalibration,nearDepth,farDepth,
            disparityNoise,noiseReferenceFx,baseline,depthUnits=depthUnits);
          if not valid then
            qualified[index] := 0.0;
          end if;
        end if;
      end for;
    end for;
  end if;
end RGBDDepthQualifiedScores;
