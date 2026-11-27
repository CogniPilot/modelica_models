within Vision;

package Neural "Shape-static neural-network inference primitives"
  annotation(Documentation(info = "<html>
    <h4>Run trained networks without a framework runtime</h4>
<p>This package provides deterministic Modelica inference operators for compact
vision networks. Image tensors use the same row, column, channel ordering as
<a href=\"modelica://Vision.Features\">Vision.Features</a>. Kernel tensors use
row, column, input-channel, output-channel ordering. All extents are fixed when
the model is translated, allowing Rumoca to preserve convolution stencils,
maps and matrix products for native, embedded and accelerator backends.</p>
<p>Both convolution functions return the input extent divided by stride,
rounded up. An odd amount of zero padding places the extra row or column at
the bottom or right (SAME_UPPER).</p>
<p>The package owns inference math only. Training and interchange tools may
populate parameter arrays, but deployed execution does not require Python,
ONNX, TensorFlow or PyTorch. Use <code>depthwiseConv2DSame</code> followed by a
1-by-1 <code>conv2DSame</code> to express MobileNet blocks without expanding a
dense convolution. MobileNetV3 squeeze/excitation blocks use
<code>globalAveragePool</code>, <code>dense</code>, <code>hardSigmoid</code> and
<code>scaleChannels</code>. Exporters should fold inference batch normalization
into the preceding convolution weights and bias.</p>
<p>Inputs and weights are ordinary <code>Real</code> arrays in this first
profile. Quantized storage and fused integer kernels belong in a subsequent
profile with explicit scale and zero-point contracts.</p>
    </html>"));
end Neural;
