within Vision;

function normalizeImage "Per-channel affine image normalization"
  input Real image[:,:,:] "Rows, columns, channels";
  input Real offset[size(image,3)] = zeros(size(image,3));
  input Real scale[size(image,3)] = ones(size(image,3));
  output Real normalized[size(image,1),size(image,2),size(image,3)];
algorithm
  for row in 1:size(image,1) loop
    for column in 1:size(image,2) loop
      for channel in 1:size(image,3) loop
        normalized[row,column,channel] :=
          (image[row,column,channel]-offset[channel])*scale[channel];
      end for;
    end for;
  end for;
end normalizeImage;
