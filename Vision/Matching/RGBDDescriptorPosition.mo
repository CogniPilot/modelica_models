within Vision.Matching;
function RGBDDescriptorPosition
  input Real pixel[2];
  input Integer imageSize[2];
  input Boolean requested;
  output Boolean valid;
  output Integer x;
  output Integer y;
algorithm
  valid := requested and pixel[1] >= 3.0 and pixel[1] <= imageSize[2]-4 and
    pixel[2] >= 3.0 and pixel[2] <= imageSize[1]-4 and
    floor(pixel[1]) == pixel[1] and floor(pixel[2]) == pixel[2];
  x := if valid then integer(pixel[1]) else 3;
  y := if valid then integer(pixel[2]) else 3;
end RGBDDescriptorPosition;
