within Vision.Neural;

function conv2DSame "Channel-last 2-D convolution with SAME zero padding"
  input Real x[:,:,:] "Rows, columns, input channels";
  input Real kernel[:,:,:,:] "Kernel rows, columns, input channels, output channels";
  input Real bias[size(kernel,4)] = zeros(size(kernel,4));
  input Integer stride(min=1) = 1;
  output Real y[div(size(x,1)+stride-1,stride),
                div(size(x,2)+stride-1,stride),size(kernel,4)];
protected
  Integer padRows;
  Integer padColumns;
  Integer sourceRow;
  Integer sourceColumn;
algorithm
  assert(size(kernel,3) == size(x,3),
    "conv2DSame kernel input channels must match the input tensor");
  assert(mod(size(kernel,1),2) == 1 and mod(size(kernel,2),2) == 1,
    "conv2DSame currently requires odd kernel extents");
  padRows := div(size(kernel,1)-1,2);
  padColumns := div(size(kernel,2)-1,2);
  y := zeros(size(y,1),size(y,2),size(y,3));
  for row in 1:size(y,1) loop
    for column in 1:size(y,2) loop
      for outputChannel in 1:size(y,3) loop
        y[row,column,outputChannel] := bias[outputChannel];
        for kernelRow in 1:size(kernel,1) loop
          sourceRow := (row-1)*stride + kernelRow - padRows;
          if sourceRow >= 1 and sourceRow <= size(x,1) then
            for kernelColumn in 1:size(kernel,2) loop
              sourceColumn := (column-1)*stride + kernelColumn - padColumns;
              if sourceColumn >= 1 and sourceColumn <= size(x,2) then
                for inputChannel in 1:size(x,3) loop
                  y[row,column,outputChannel] := y[row,column,outputChannel]
                    + x[sourceRow,sourceColumn,inputChannel]
                    * kernel[kernelRow,kernelColumn,inputChannel,outputChannel];
                end for;
              end if;
            end for;
          end if;
        end for;
      end for;
    end for;
  end for;
end conv2DSame;
