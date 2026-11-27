within Vision.Neural;

function depthwiseConv2DSame
  "Channel-last depthwise 2-D convolution with SAME_UPPER zero padding"
  input Real x[:,:,:] "Rows, columns, input channels";
  input Real kernel[:,:,:] "Kernel rows, columns, input channels";
  input Real bias[size(x,3)] = zeros(size(x,3));
  input Integer stride(min=1) = 1;
  output Real y[div(size(x,1)+stride-1,stride),
                div(size(x,2)+stride-1,stride),size(x,3)];
protected
  Integer padRows;
  Integer padColumns;
  Integer sourceRow;
  Integer sourceColumn;
algorithm
  assert(size(kernel,3) == size(x,3),
    "depthwiseConv2DSame kernel channels must match the input tensor");
  padRows := div(max(0,(size(y,1)-1)*stride+size(kernel,1)-size(x,1)),2);
  padColumns := div(max(0,(size(y,2)-1)*stride+size(kernel,2)-size(x,2)),2);
  y := zeros(size(y,1),size(y,2),size(y,3));
  for row in 1:size(y,1) loop
    for column in 1:size(y,2) loop
      for channel in 1:size(y,3) loop
        y[row,column,channel] := bias[channel];
        for kernelRow in 1:size(kernel,1) loop
          sourceRow := (row-1)*stride + kernelRow - padRows;
          if sourceRow >= 1 and sourceRow <= size(x,1) then
            for kernelColumn in 1:size(kernel,2) loop
              sourceColumn := (column-1)*stride + kernelColumn - padColumns;
              if sourceColumn >= 1 and sourceColumn <= size(x,2) then
                y[row,column,channel] := y[row,column,channel]
                  + x[sourceRow,sourceColumn,channel]
                  * kernel[kernelRow,kernelColumn,channel];
              end if;
            end for;
          end if;
        end for;
      end for;
    end for;
  end for;
end depthwiseConv2DSame;
