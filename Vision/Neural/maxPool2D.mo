within Vision.Neural;

function maxPool2D "Channel-last max pooling over complete windows"
  input Real x[:,:,:] "Rows, columns, channels";
  input Integer windowRows(min=1)=2;
  input Integer windowColumns(min=1)=2;
  input Integer stride(min=1)=2;
  output Real y[if size(x,1)>=windowRows then 1+div(size(x,1)-windowRows,stride) else 0,
                if size(x,2)>=windowColumns then 1+div(size(x,2)-windowColumns,stride) else 0,
                size(x,3)];
algorithm
  for row in 1:size(y,1) loop
    for column in 1:size(y,2) loop
      for channel in 1:size(y,3) loop
        y[row,column,channel] := max({
          x[(row-1)*stride+i,(column-1)*stride+j,channel]
          for i in 1:windowRows, j in 1:windowColumns});
      end for;
    end for;
  end for;
end maxPool2D;
