within Tests;

model NeuralTests "Shape, convolution and probability checks for NeuralNetworks"
  parameter Real image[3,3,2]={{{1,10},{2,20},{3,30}},
                               {{4,40},{5,50},{6,60}},
                               {{7,70},{8,80},{9,90}}};
  parameter Real sumKernel[1,1,2,2]=fill(1.0,1,1,2,2);
  parameter Real boxKernel[3,3,2]=fill(1.0,3,3,2);
  parameter Real pointKernel[1,1,2,1]=fill(0.1,1,1,2,1);
  parameter Real evenImage[4,4,1]={{{1},{2},{3},{4}},{{5},{6},{7},{8}},
                                  {{9},{10},{11},{12}},{{13},{14},{15},{16}}};
  Real channelSum[3,3,2];
  Real pooled[2];
  Real depthwise[2,2,2];
  Real projected[3,3,1];
  Real activated[3,3,1];
  Real swish[1,1,3];
  Real gated[1,1,3];
  Real logits[3];
  Real probability[3];
  Real strided[2,2,1];
  Real stridedDepthwise[2,2,1];
  Real evenKernel[3,3,2];
  Real localPool[1,1,2];
equation
  channelSum=NeuralNetworks.conv2DSame(image,sumKernel);
  pooled=NeuralNetworks.globalAveragePool(channelSum);
  depthwise=NeuralNetworks.depthwiseConv2DSame(image,boxKernel,zeros(2),2);
  projected=NeuralNetworks.conv2DSame(image,pointKernel,{0.5});
  activated=NeuralNetworks.relu6(projected);
  swish=NeuralNetworks.hardSwish({{{-4,0,4}}});
  gated=NeuralNetworks.scaleChannels(swish,NeuralNetworks.hardSigmoid({-3,0,3}));
  logits=NeuralNetworks.dense({1,2},{{1,0},{0,1},{1,1}},{0,0,0});
  probability=NeuralNetworks.softmax(logits);
  strided=NeuralNetworks.conv2DSame(evenImage,ones(3,3,1,1),stride=2);
  stridedDepthwise=NeuralNetworks.depthwiseConv2DSame(evenImage,ones(3,3,1),stride=2);
  evenKernel=NeuralNetworks.depthwiseConv2DSame(image,ones(2,2,2));
  localPool=NeuralNetworks.maxPool2D(Vision.normalizeImage(image,{1,10},{1,0.1}));
  assert(abs(channelSum[2,2,1]-55)<1e-12 and abs(channelSum[2,2,2]-55)<1e-12,
    "1x1 channel projection");
  assert(max(abs(pooled-{55,55}))<1e-12,"global average pooling");
  assert(abs(depthwise[1,1,1]-12)<1e-12 and abs(depthwise[1,1,2]-120)<1e-12,
    "strided depthwise SAME convolution");
  assert(strided[1,1,1]==54 and strided[1,2,1]==45 and strided[2,1,1]==72 and strided[2,2,1]==54 and
    max(abs(stridedDepthwise-strided))<1e-12,"SAME_UPPER puts excess stride padding at the bottom or right");
  assert(evenKernel[1,1,1]==12 and evenKernel[3,3,2]==90,"Even kernels use SAME_UPPER padding");
  assert(max(abs(localPool[1,1,:]-{4,4}))<1e-12,
    "Per-channel normalization and max pooling exclude incomplete windows");
  assert(abs(projected[2,2,1]-6)<1e-12 and abs(activated[2,2,1]-6)<1e-12,
    "pointwise projection and ReLU6");
  assert(abs(gated[1,1,1])<1e-12 and abs(gated[1,1,2])<1e-12
      and abs(gated[1,1,3]-4)<1e-12,"MobileNetV3 hard-swish and channel gate");
  assert(abs(sum(probability)-1)<1e-12 and probability[3]>probability[2] and probability[2]>probability[1],
    "stable softmax ordering and normalization");
end NeuralTests;
