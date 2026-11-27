within Tests;

model NeuralTests "Shape, convolution and probability checks for Vision.Neural"
  parameter Real image[3,3,2]={{{1,10},{2,20},{3,30}},
                               {{4,40},{5,50},{6,60}},
                               {{7,70},{8,80},{9,90}}};
  parameter Real sumKernel[1,1,2,2]=fill(1.0,1,1,2,2);
  parameter Real boxKernel[3,3,2]=fill(1.0,3,3,2);
  parameter Real pointKernel[1,1,2,1]=fill(0.1,1,1,2,1);
  Real channelSum[3,3,2];
  Real pooled[2];
  Real depthwise[2,2,2];
  Real projected[3,3,1];
  Real activated[3,3,1];
  Real swish[1,1,3];
  Real gated[1,1,3];
  Real logits[3];
  Real probability[3];
equation
  channelSum=Vision.Neural.conv2DSame(image,sumKernel);
  pooled=Vision.Neural.globalAveragePool(channelSum);
  depthwise=Vision.Neural.depthwiseConv2DSame(image,boxKernel,zeros(2),2);
  projected=Vision.Neural.conv2DSame(image,pointKernel,{0.5});
  activated=Vision.Neural.relu6(projected);
  swish=Vision.Neural.hardSwish({{{-4,0,4}}});
  gated=Vision.Neural.scaleChannels(swish,Vision.Neural.hardSigmoid({-3,0,3}));
  logits=Vision.Neural.dense({1,2},{{1,0},{0,1},{1,1}},{0,0,0});
  probability=Vision.Neural.softmax(logits);
  assert(abs(channelSum[2,2,1]-55)<1e-12 and abs(channelSum[2,2,2]-55)<1e-12,
    "1x1 channel projection");
  assert(max(abs(pooled-{55,55}))<1e-12,"global average pooling");
  assert(abs(depthwise[1,1,1]-12)<1e-12 and abs(depthwise[1,1,2]-120)<1e-12,
    "strided depthwise SAME convolution");
  assert(abs(projected[2,2,1]-6)<1e-12 and abs(activated[2,2,1]-6)<1e-12,
    "pointwise projection and ReLU6");
  assert(abs(gated[1,1,1])<1e-12 and abs(gated[1,1,2])<1e-12
      and abs(gated[1,1,3]-4)<1e-12,"MobileNetV3 hard-swish and channel gate");
  assert(abs(sum(probability)-1)<1e-12 and probability[3]>probability[2] and probability[2]>probability[1],
    "stable softmax ordering and normalization");
end NeuralTests;
