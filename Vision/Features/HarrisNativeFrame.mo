within Vision.Features;
// Harris response from central gradients and 5x5 structure tensors.
// score[y,x] corresponds to image pixel (x+scoreBorder,y+scoreBorder).
model HarrisNativeFrame
  parameter Integer height = 90;
  parameter Integer width = 160;
  parameter Real harris_k = 0.04;
  constant Integer channelCount = 3;
  constant Real channelMaximum = 255.0;
  constant Integer gradientRadius = 1;
  constant Integer tensorWindow = 5;
  constant Integer tensorOffset = 1;
  constant Integer scoreBorder = 4;
  final parameter Integer gradientHeight = height-2*gradientRadius;
  final parameter Integer gradientWidth = width-2*gradientRadius;
  final parameter Integer scoreHeight = height-2*scoreBorder;
  final parameter Integer scoreWidth = width-2*scoreBorder;
  input Real rgb[height,width,channelCount] = fill(0.0,height,width,channelCount);
  output Real score[scoreHeight,scoreWidth];
protected
  Real gray[height,width];
  Real dx[gradientHeight,gradientWidth];
  Real dy[gradientHeight,gradientWidth];
  Real gxx[gradientHeight,gradientWidth];
  Real gyy[gradientHeight,gradientWidth];
  Real gxy[gradientHeight,gradientWidth];
  Real xx[scoreHeight,scoreWidth];
  Real yy[scoreHeight,scoreWidth];
  Real xy[scoreHeight,scoreWidth];
equation
  for y in 1:height loop
    for x in 1:width loop
      gray[y,x] = ((rgb[y,x,1]+rgb[y,x,2])+rgb[y,x,3])/channelCount/channelMaximum;
    end for;
  end for;
  for y in 1:gradientHeight loop
    for x in 1:gradientWidth loop
      dx[y,x] = (gray[y+gradientRadius,x+2*gradientRadius]-gray[y+gradientRadius,x])/2.0;
      dy[y,x] = (gray[y+2*gradientRadius,x+gradientRadius]-gray[y,x+gradientRadius])/2.0;
      gxx[y,x] = dx[y,x]*dx[y,x];
      gyy[y,x] = dy[y,x]*dy[y,x];
      gxy[y,x] = dx[y,x]*dy[y,x];
    end for;
  end for;
  // Keep row outside column and divide only after the complete 25-term sum.
  for y in 1:scoreHeight loop
    for x in 1:scoreWidth loop
      xx[y,x] = sum(gxx[y+row,x+column]
        for row in tensorOffset:tensorOffset+tensorWindow-1,
          column in tensorOffset:tensorOffset+tensorWindow-1)/(tensorWindow*tensorWindow);
      yy[y,x] = sum(gyy[y+row,x+column]
        for row in tensorOffset:tensorOffset+tensorWindow-1,
          column in tensorOffset:tensorOffset+tensorWindow-1)/(tensorWindow*tensorWindow);
      xy[y,x] = sum(gxy[y+row,x+column]
        for row in tensorOffset:tensorOffset+tensorWindow-1,
          column in tensorOffset:tensorOffset+tensorWindow-1)/(tensorWindow*tensorWindow);
      score[y,x] = xx[y,x]*yy[y,x]-xy[y,x]*xy[y,x]-harris_k*((xx[y,x]+yy[y,x])*(xx[y,x]+yy[y,x]));
    end for;
  end for;
end HarrisNativeFrame;
